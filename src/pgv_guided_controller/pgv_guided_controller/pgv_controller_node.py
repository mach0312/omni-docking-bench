"""PGV 기반 좌표 이동 컨트롤러 (PgvControllerNode).

PGV 센서의 PoseStamped 피드백을 이용하여 로봇을 임의의 (x, y, yaw) 목표로
이동시키는 범용 ROS 2 컨트롤러.

상태 머신:
    IDLE   : 목표 없음 (정지)
    COARSE : P 제어로 목표 접근
    FINE   : cubic interpolation 기반 micro 정밀 위치 보정
    DONE   : 도달 완료 후 IDLE 전이

인터페이스 (모든 토픽은 launch arg 로 주입되는 namespace 아래에서 동작):
    Sub  goal_topic (param)          (PoseStamped)  : (x, y, yaw) 목표
    Sub  forward_topic (param)       (Bool)         : 수동 전진 hold
    Sub  backward_topic (param)      (Bool)         : 수동 후진 hold
    Sub  pose_topic (param)          (PoseStamped)  : 센서 포즈
    Pub  cmd_topic (param)           (Twist)        : 속도 명령
    Pub  state_topic (param)         (String)       : 현재 phase
    Srv  cancel_service (param)      (Trigger)      : 목표 취소
"""

import math
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


# ------------------------------------------------------------------ utils
def ang_norm(a):
    """각도를 [-pi, pi] 범위로 정규화."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def clamp(val, lo, hi):
    return max(lo, min(hi, val))


def cubic_interpolation(x0, x1, tc, ts, te):
    """3차 Hermite 보간. (position, velocity) 반환."""
    if tc < ts:
        tc = ts
    if tc > te:
        tc = te
    if te - ts < 0.1:
        te = ts + 0.1
    t = (tc - ts) / (te - ts)
    return x0 + (x1 - x0) * t * t * (3 - 2 * t), (x1 - x0) * t * (6 - 6 * t)


def yaw_from_quat(qx, qy, qz, qw):
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


# ------------------------------------------------------------------ phase
class Phase(Enum):
    IDLE = 0
    COARSE = 1
    FINE = 2
    DONE = 3


# ------------------------------------------------------------------ node
class PgvControllerNode(Node):
    def __init__(self):
        super().__init__('pgv_controller')

        # ======================== 파라미터 ========================
        # Topic namespace / name 파라미터화 (launch arg 주입)
        self.declare_parameter('pose_topic', '/pgv/raw_pose')
        self.declare_parameter('cmd_topic', '/tracing_vel')
        self.declare_parameter('goal_topic', '/pgv_controller/goal')
        self.declare_parameter('forward_topic', '/pgv_controller/go_forward')
        self.declare_parameter('backward_topic', '/pgv_controller/go_backward')
        self.declare_parameter('state_topic', '/pgv_controller/state')
        self.declare_parameter('cancel_service', '/pgv_controller/cancel')
        self.declare_parameter('holonomic', True)
        self.declare_parameter('control_rate', 50.0)

        self.declare_parameter('max_lin_vel', 0.3)
        self.declare_parameter('max_ang_vel', 0.05)
        self.declare_parameter('accel_lin', 0.8)
        self.declare_parameter('accel_ang', 1.5)

        self.declare_parameter('kp_x', 1.0)
        self.declare_parameter('kp_y', 1.0)
        self.declare_parameter('kp_yaw', 1.0)

        self.declare_parameter('pose_timeout_sec', 0.1)
        self.declare_parameter('post_timeout_grace_sec', 1.0)

        self.declare_parameter('sensor_x_offset', 0.0)
        self.declare_parameter('sensor_y_offset', 0.0)
        self.declare_parameter('sensor_yaw_offset_deg', 0.0)

        self.declare_parameter('yaw_wrap_band_deg', 12.0)
        self.declare_parameter('allow_reverse_heading', True)

        self.declare_parameter('teleop_speed', 0.15)
        self.declare_parameter('manual_timeout', 0.1)

        # COARSE → FINE 전환
        self.declare_parameter('coarse_xy_threshold', 0.01)
        self.declare_parameter('coarse_yaw_threshold_deg', 5.0)

        # FINE 허용오차
        self.declare_parameter('fine_xy_tolerance', 0.003)
        self.declare_parameter('fine_yaw_tolerance_deg', 2.0)

        # FINE micro 제어
        self.declare_parameter('fine_max_delta_xy', 0.03)
        self.declare_parameter('fine_max_delta_yaw_deg', 17.0)
        self.declare_parameter('fine_cycle_duration', 1.0)

        # ======================== 파라미터 fetch ========================
        self.pose_topic = self.get_parameter('pose_topic').value
        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.goal_topic = self.get_parameter('goal_topic').value
        self.forward_topic = self.get_parameter('forward_topic').value
        self.backward_topic = self.get_parameter('backward_topic').value
        self.state_topic = self.get_parameter('state_topic').value
        self.cancel_service = self.get_parameter('cancel_service').value
        self.holonomic = bool(self.get_parameter('holonomic').value)
        self.rate_hz = float(self.get_parameter('control_rate').value)

        self.max_v = float(self.get_parameter('max_lin_vel').value)
        self.max_w = float(self.get_parameter('max_ang_vel').value)
        self.acc_v = float(self.get_parameter('accel_lin').value)
        self.acc_w = float(self.get_parameter('accel_ang').value)

        self.kp_x = float(self.get_parameter('kp_x').value)
        self.kp_y = float(self.get_parameter('kp_y').value)
        self.kp_yaw = float(self.get_parameter('kp_yaw').value)

        self.pose_timeout = float(self.get_parameter('pose_timeout_sec').value)
        self.post_timeout_grace = float(self.get_parameter('post_timeout_grace_sec').value)

        self.sensor_x_offset = float(self.get_parameter('sensor_x_offset').value)
        self.sensor_y_offset = float(self.get_parameter('sensor_y_offset').value)
        self.sensor_yaw_offset = math.radians(float(self.get_parameter('sensor_yaw_offset_deg').value))

        self.yaw_wrap_band = math.radians(float(self.get_parameter('yaw_wrap_band_deg').value))
        self.allow_reverse_heading = bool(self.get_parameter('allow_reverse_heading').value)

        self.teleop_speed = float(self.get_parameter('teleop_speed').value)
        self.manual_timeout = float(self.get_parameter('manual_timeout').value)

        self.coarse_xy_thresh = float(self.get_parameter('coarse_xy_threshold').value)
        self.coarse_yaw_thresh = math.radians(float(self.get_parameter('coarse_yaw_threshold_deg').value))

        self.fine_xy_tol = float(self.get_parameter('fine_xy_tolerance').value)
        self.fine_yaw_tol = math.radians(float(self.get_parameter('fine_yaw_tolerance_deg').value))

        self.fine_max_dxy = float(self.get_parameter('fine_max_delta_xy').value)
        self.fine_max_dyaw = math.radians(float(self.get_parameter('fine_max_delta_yaw_deg').value))
        self.fine_cycle_dur = float(self.get_parameter('fine_cycle_duration').value)

        # ======================== I/O ========================
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5)

        self.pose_sub = self.create_subscription(PoseStamped, self.pose_topic, self._on_pose, qos)
        self.goal_sub = self.create_subscription(PoseStamped, self.goal_topic, self._on_goal, 10)
        self.fwd_sub = self.create_subscription(Bool, self.forward_topic, self._on_fwd, 10)
        self.back_sub = self.create_subscription(Bool, self.backward_topic, self._on_back, 10)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.state_pub = self.create_publisher(String, self.state_topic, 10)

        self.srv_cancel = self.create_service(Trigger, self.cancel_service, self._srv_cancel)

        # ======================== 상태 변수 ========================
        self.phase = Phase.IDLE

        # 센서 포즈
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.yaw_sign_fixed = 0
        self.last_pose_time = self.get_clock().now()

        # 타임아웃 / grace
        self.timeout_active = False
        self.resume_block_until = None
        self._last_grace_log_time = None

        # 목표
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_yaw = 0.0
        self.goal_active = False

        # 수동 hold-to-run
        self.last_fwd_true = None
        self.last_back_true = None

        # 속도 명령 이력 (slew 용)
        self.last_twist = Twist()

        # FINE micro 상태
        self._fine_active = False
        self._fine_start_time = None
        self._fine_delta_x = 0.0
        self._fine_delta_y = 0.0
        self._fine_delta_yaw = 0.0
        self._fine_sub_phase = 'xy'  # 'xy' or 'yaw' (non-holonomic)

        # 로그 throttle
        self._coarse_log_time = self.get_clock().now()

        # ======================== 제어 타이머 ========================
        period = 1.0 / self.rate_hz
        self.timer = self.create_timer(period, self._control_loop)

        self.get_logger().info(
            f"[pgv_controller] holonomic={self.holonomic}, "
            f"cmd={self.cmd_topic}, pose={self.pose_topic}")

    # ================================================================
    #  콜백
    # ================================================================
    def _on_pose(self, msg: PoseStamped):
        self.last_pose_time = rclpy.time.Time.from_msg(msg.header.stamp)

        # 센서 raw 값 그대로 사용 (goal도 센서 좌표 기준)
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y

        o = msg.pose.orientation
        yaw_raw = yaw_from_quat(o.x, o.y, o.z, o.w)
        yaw_meas = ang_norm(yaw_raw + self.sensor_yaw_offset)

        # ±π 경계 wrap band 안정화
        if abs(abs(yaw_meas) - math.pi) < self.yaw_wrap_band:
            if self.yaw_sign_fixed == 0:
                self.yaw_sign_fixed = 1 if yaw_meas >= 0.0 else -1
            self.yaw = yaw_meas
        else:
            self.yaw_sign_fixed = 0
            self.yaw = yaw_meas

    def _on_goal(self, msg: PoseStamped):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        o = msg.pose.orientation
        self.goal_yaw = yaw_from_quat(o.x, o.y, o.z, o.w)
        self.goal_active = True
        self._enter_coarse()
        self.get_logger().info(
            f"[goal] x={self.goal_x:.3f}, y={self.goal_y:.3f}, "
            f"yaw={math.degrees(self.goal_yaw):.1f} deg")

    def _on_fwd(self, msg: Bool):
        if msg.data:
            self.last_fwd_true = self.get_clock().now()

    def _on_back(self, msg: Bool):
        if msg.data:
            self.last_back_true = self.get_clock().now()

    def _srv_cancel(self, req, resp):
        self._enter_idle()
        resp.success = True
        resp.message = "Cancelled"
        return resp

    # ================================================================
    #  Phase 전이
    # ================================================================
    def _enter_idle(self):
        self.phase = Phase.IDLE
        self.goal_active = False
        self._fine_active = False
        self._publish_twist(0.0, 0.0, 0.0)

    def _enter_coarse(self):
        self.phase = Phase.COARSE
        self._fine_active = False

    def _enter_fine(self):
        self.phase = Phase.FINE
        self._fine_active = False  # 다음 tick에서 초기화

    def _enter_done(self):
        self.phase = Phase.DONE
        self.goal_active = False
        self._fine_active = False
        self._publish_twist(0.0, 0.0, 0.0)

        self.get_logger().info(
            f"[DONE] x={self.x:.4f}, y={self.y:.4f}, "
            f"yaw={math.degrees(self.yaw):.2f} deg")

        # 즉시 IDLE 전이
        self.phase = Phase.IDLE

    # ================================================================
    #  제어 루프
    # ================================================================
    def _control_loop(self):
        now = self.get_clock().now()

        # --- (A) 상태 발행 ---
        state_msg = String()
        state_msg.data = self.phase.name
        self.state_pub.publish(state_msg)

        # --- (B) Pose timeout ---
        if (now - self.last_pose_time) > Duration(seconds=self.pose_timeout):
            if not self.timeout_active:
                self.timeout_active = True
                self.resume_block_until = None
                self.get_logger().warn("[safety] pose timeout -> STOP")
            self._stop_if_moving()
            return

        # --- (C) Timeout 회복 → grace ---
        if self.timeout_active:
            self.timeout_active = False
            self.resume_block_until = now + Duration(seconds=self.post_timeout_grace)
            self._last_grace_log_time = now
            self.get_logger().warn(
                f"[safety] pose recovered; grace {self.post_timeout_grace:.2f}s")

        # --- (D) Grace 기간 ---
        if self.resume_block_until is not None and now < self.resume_block_until:
            if self._last_grace_log_time is None or \
               (now - self._last_grace_log_time) > Duration(seconds=0.25):
                remaining = (self.resume_block_until - now).nanoseconds / 1e9
                self.get_logger().info(f"[safety] grace: {remaining:.2f}s left")
                self._last_grace_log_time = now
            self._stop_if_moving()
            return

        if self.resume_block_until is not None and now >= self.resume_block_until:
            self.resume_block_until = None
            self.get_logger().info("[safety] grace ended; resuming")

        # --- (E) 수동 hold-to-run ---
        manual_dir = self._manual_active()
        if manual_dir != 0:
            self._do_manual(manual_dir)
            return

        # --- (F) 목표 없으면 정지 ---
        if not self.goal_active and self.phase == Phase.IDLE:
            self._stop_if_moving()
            return

        # --- (G) Phase 분기 ---
        if self.phase == Phase.COARSE:
            self._do_coarse()
        elif self.phase == Phase.FINE:
            self._do_fine()
        elif self.phase == Phase.DONE:
            self._stop_if_moving()

    # ================================================================
    #  COARSE
    # ================================================================
    def _do_coarse(self):
        x_err = self.goal_x - self.x
        y_err = self.goal_y - self.y
        yaw_err = self._yaw_err()
        xy_dist = math.hypot(x_err, y_err)

        # 로그 (1Hz)
        now = self.get_clock().now()
        if (now - self._coarse_log_time).nanoseconds / 1e9 >= 1.0:
            self._coarse_log_time = now
            self.get_logger().info(
                f"[COARSE] x_err={x_err:.4f}, y_err={y_err:.4f}, "
                f"yaw_err={math.degrees(yaw_err):.2f}, dist={xy_dist:.4f}")

        # COARSE → FINE 전환 판정
        if xy_dist <= self.coarse_xy_thresh and abs(yaw_err) <= self.coarse_yaw_thresh:
            self.get_logger().info("[COARSE→FINE] threshold reached")
            self._enter_fine()
            return

        if self.holonomic:
            # map frame 에러 → body frame 속도 변환
            c = math.cos(self.yaw)
            s = math.sin(self.yaw)
            vx = self.kp_x * (x_err * c + y_err * s)
            vy = self.kp_y * (-x_err * s + y_err * c)
            w = self.kp_yaw * yaw_err

            vx = self._slew(vx, self.last_twist.linear.x, self.acc_v)
            vy = self._slew(vy, self.last_twist.linear.y, self.acc_v)
            w = self._slew(w, self.last_twist.angular.z, self.acc_w)

            spd = math.hypot(vx, vy)
            if spd > self.max_v:
                s = self.max_v / max(spd, 1e-6)
                vx *= s
                vy *= s
            w = clamp(w, -self.max_w, self.max_w)

            self._publish_twist(vx, vy, w)
        else:
            # Non-holonomic
            heading_to_goal = math.atan2(y_err, x_err)
            heading_err = ang_norm(heading_to_goal - self.yaw)

            # 거리 기반 블렌딩: 멀리=heading 추종, 가까이=goal yaw 추종
            alpha = clamp(xy_dist / max(self.coarse_xy_thresh, 1e-6), 0.0, 1.0)
            eff_yaw_err = alpha * heading_err + (1.0 - alpha) * yaw_err

            v = self.kp_x * xy_dist * math.cos(heading_err)
            w = self.kp_yaw * eff_yaw_err

            v = self._slew(v, self.last_twist.linear.x, self.acc_v)
            w = self._slew(w, self.last_twist.angular.z, self.acc_w)
            v = clamp(v, -self.max_v, self.max_v)
            w = clamp(w, -self.max_w, self.max_w)

            self._publish_twist(v, 0.0, w)

    # ================================================================
    #  FINE
    # ================================================================
    def _do_fine(self):
        if self.holonomic:
            self._do_fine_holonomic()
        else:
            self._do_fine_nonholonomic()

    def _do_fine_holonomic(self):
        """통합 XY+Yaw micro control (holonomic)."""
        if not self._fine_active:
            self._fine_start_time = self.get_clock().now()
            # map frame 에러
            dx_map = self.goal_x - self.x
            dy_map = self.goal_y - self.y
            dyaw = self._yaw_err()
            # body frame으로 변환
            c = math.cos(self.yaw)
            s = math.sin(self.yaw)
            dx_body = dx_map * c + dy_map * s
            dy_body = -dx_map * s + dy_map * c
            self._fine_delta_x = clamp(dx_body, -self.fine_max_dxy, self.fine_max_dxy)
            self._fine_delta_y = clamp(dy_body, -self.fine_max_dxy, self.fine_max_dxy)
            self._fine_delta_yaw = clamp(dyaw, -self.fine_max_dyaw, self.fine_max_dyaw)
            self._fine_active = True

        elapsed = (self.get_clock().now() - self._fine_start_time).nanoseconds / 1e9
        t = elapsed / self.fine_cycle_dur

        if t < 0.2:
            # 미세 초기 속도
            dist = math.hypot(self._fine_delta_x, self._fine_delta_y)
            if dist < 1e-6:
                vx, vy = 0.0, 0.0
            else:
                vx = self._fine_delta_x / dist * 0.0001
                vy = self._fine_delta_y / dist * 0.0001
            self._publish_twist(vx, vy, 0.0)

        elif t < 1.0:
            # cubic 보간
            _, vx = cubic_interpolation(0.0, self._fine_delta_x, t, 0.2, 1.0)
            _, vy = cubic_interpolation(0.0, self._fine_delta_y, t, 0.2, 1.0)
            _, wz = cubic_interpolation(0.0, self._fine_delta_yaw, t, 0.2, 1.0)
            self._publish_twist(vx, vy, wz)

        elif t < 1.2:
            # 정지 안정화
            self._publish_twist(0.0, 0.0, 0.0)

        else:
            # 사이클 종료 → tolerance 판정
            self._fine_active = False
            self._check_fine_done()

    def _do_fine_nonholonomic(self):
        """교대 Yaw→XY micro control (non-holonomic)."""
        if not self._fine_active:
            self._fine_start_time = self.get_clock().now()
            self._fine_active = True
            # yaw부터 시작
            dyaw = self._yaw_err()
            if abs(dyaw) > self.fine_yaw_tol:
                self._fine_sub_phase = 'yaw'
                self._fine_delta_yaw = clamp(dyaw, -self.fine_max_dyaw, self.fine_max_dyaw)
            else:
                self._fine_sub_phase = 'xy'
                dx = self.goal_x - self.x
                dy = self.goal_y - self.y
                dist = math.hypot(dx, dy)
                self._fine_delta_x = clamp(dx, -self.fine_max_dxy, self.fine_max_dxy)
                # non-holo는 heading 방향으로만 이동
                self._fine_delta_y = 0.0

        elapsed = (self.get_clock().now() - self._fine_start_time).nanoseconds / 1e9
        t = elapsed / self.fine_cycle_dur

        if self._fine_sub_phase == 'yaw':
            if t < 0.3:
                self._publish_twist(0.0, 0.0, 0.0)
            elif t < 1.0:
                _, wz = cubic_interpolation(0.0, self._fine_delta_yaw, t, 0.3, 1.0)
                self._publish_twist(0.0, 0.0, wz)
            elif t < 1.3:
                self._publish_twist(0.0, 0.0, 0.0)
            else:
                self._fine_active = False
                # yaw 완료 → xy 사이클로 전환 또는 done 판정
                yaw_err = abs(ang_norm(self.goal_yaw - self.yaw))
                if yaw_err <= self.fine_yaw_tol:
                    # xy 필요한지 확인
                    xy_err = math.hypot(self.goal_x - self.x, self.goal_y - self.y)
                    if xy_err <= self.fine_xy_tol:
                        self._check_fine_done()
                    # else: 다음 tick에서 _fine_active=False이므로 xy로 재초기화
                # else: 다음 tick에서 yaw 재시도

        elif self._fine_sub_phase == 'xy':
            if t < 0.3:
                dist = math.hypot(self._fine_delta_x, self._fine_delta_y)
                if dist < 1e-6:
                    vx = 0.0
                else:
                    vx = self._fine_delta_x / dist * 0.0001
                self._publish_twist(vx, 0.0, 0.0)
            elif t < 1.3:
                _, vx = cubic_interpolation(0.0, self._fine_delta_x, t, 0.3, 1.3)
                self._publish_twist(vx, 0.0, 0.0)
            elif t < 1.6:
                self._publish_twist(0.0, 0.0, 0.0)
            else:
                self._fine_active = False
                self._check_fine_done()

    def _check_fine_done(self):
        xy_err = math.hypot(self.goal_x - self.x, self.goal_y - self.y)
        yaw_err = abs(self._yaw_err())  # reverse heading 자동 고려
        yaw_err_deg = math.degrees(yaw_err)

        if xy_err <= self.fine_xy_tol and yaw_err <= self.fine_yaw_tol:
            self.get_logger().info(
                f"[FINE] done. xy_err={xy_err:.4f}, yaw_err={yaw_err_deg:.2f}")
            self._enter_done()
        else:
            self.get_logger().info(
                f"[FINE] retry. xy_err={xy_err:.4f}, yaw_err={yaw_err_deg:.2f}")
            # _fine_active=False이므로 다음 tick에서 새 사이클 시작

    # ================================================================
    #  수동 hold-to-run
    # ================================================================
    def _manual_active(self):
        now = self.get_clock().now()
        tol = Duration(seconds=self.manual_timeout)
        fwd = self.last_fwd_true is not None and (now - self.last_fwd_true) <= tol
        bck = self.last_back_true is not None and (now - self.last_back_true) <= tol
        if fwd and bck:
            return 0
        if fwd:
            return 1
        if bck:
            return -1
        return 0

    def _do_manual(self, direction: int):
        speed = self.teleop_speed * (1.0 if direction > 0 else -1.0)
        y_err = -self.y  # y→0 유지
        yaw_err = self._yaw_err()

        if self.holonomic:
            vx = speed
            vy = self.kp_y * y_err
            vx = self._slew(vx, self.last_twist.linear.x, self.acc_v)
            vy = self._slew(vy, self.last_twist.linear.y, self.acc_v)
            spd = math.hypot(vx, vy)
            if spd > self.max_v:
                s = self.max_v / max(spd, 1e-6)
                vx *= s
                vy *= s
            self._publish_twist(vx, vy, 0.0)
        else:
            v = speed
            w = self.kp_y * y_err - self.kp_yaw * ang_norm(0.0 - self.yaw)
            v = self._slew(v, self.last_twist.linear.x, self.acc_v)
            w = self._slew(w, self.last_twist.angular.z, self.acc_w)
            v = clamp(v, -self.max_v, self.max_v)
            w = clamp(w, -self.max_w, self.max_w)
            self._publish_twist(v, 0.0, w)

    # ================================================================
    #  유틸리티
    # ================================================================
    def _yaw_err(self):
        """goal_yaw 기준 yaw 오차. reverse heading 고려."""
        if self.yaw_sign_fixed != 0 and \
           abs(abs(self.yaw) - math.pi) < self.yaw_wrap_band:
            yaw_meas = self.yaw_sign_fixed * abs(self.yaw)
        else:
            yaw_meas = self.yaw

        fwd_err = ang_norm(self.goal_yaw - yaw_meas)
        if not self.allow_reverse_heading:
            return fwd_err

        rev_target = ang_norm(self.goal_yaw + math.pi)
        rev_err = ang_norm(rev_target - yaw_meas)
        return rev_err if abs(rev_err) < abs(fwd_err) else fwd_err

    def _publish_twist(self, vx, vy, wz):
        t = Twist()
        t.linear.x = float(vx)
        t.linear.y = float(vy)
        t.angular.z = float(wz)
        self.cmd_pub.publish(t)
        self.last_twist = t

    def _stop_if_moving(self):
        lt = self.last_twist
        if lt.linear.x != 0.0 or lt.linear.y != 0.0 or lt.angular.z != 0.0:
            self._publish_twist(0.0, 0.0, 0.0)

    def _slew(self, target, current, accel):
        step = accel / self.rate_hz
        delta = target - current
        if delta > step:
            return current + step
        if delta < -step:
            return current - step
        return target

    @staticmethod
    def _slew_limit(target, current, step):
        delta = target - current
        if delta > step:
            return current + step
        if delta < -step:
            return current - step
        return target


def main(args=None):
    rclpy.init(args=args)
    node = PgvControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._publish_twist(0.0, 0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
