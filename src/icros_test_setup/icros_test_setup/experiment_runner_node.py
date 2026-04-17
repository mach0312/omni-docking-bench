"""Experiment Runner Node.

시행 자동화: FollowPath action 호출 + PGV 기반 종료 판정 + CSV 기록.
"""
import math
import csv
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

import tf2_ros

from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import JointState


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    from geometry_msgs.msg import Quaternion
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class ExperimentRunnerNode(Node):
    def __init__(self):
        super().__init__('experiment_runner')

        # Parameters
        self.declare_parameter('success_threshold', 0.003)
        self.declare_parameter('success_samples', 5)
        self.declare_parameter('settle_threshold', 0.005)
        self.declare_parameter('settle_timeout', 60.0)
        self.declare_parameter('total_timeout', 90.0)
        self.declare_parameter('post_controller_timeout', 5.0)
        self.declare_parameter('pose_topic', '/pgv/raw_pose')
        self.declare_parameter(
            'results_dir',
            os.path.expanduser('~/omni_docking_bench_ws/src/icros_test_setup/results'),
        )
        self.declare_parameter('controller_name', 'mppi')
        self.declare_parameter('repeats', 3)
        self.declare_parameter('heading_offsets_deg', [0.0])
        self.declare_parameter('sensor_offset_x', 0.0)
        self.declare_parameter('sensor_offset_y', 0.0)

        self.success_thresh = self.get_parameter('success_threshold').value
        self.success_samples = self.get_parameter('success_samples').value
        self.settle_thresh = self.get_parameter('settle_threshold').value
        self.settle_timeout = self.get_parameter('settle_timeout').value
        self.total_timeout = self.get_parameter('total_timeout').value
        self.post_controller_timeout = self.get_parameter('post_controller_timeout').value
        pose_topic = self.get_parameter('pose_topic').value
        self.results_dir = self.get_parameter('results_dir').value
        self.controller_name = self.get_parameter('controller_name').value
        self.repeats = self.get_parameter('repeats').value
        self.heading_offsets = [math.radians(h) for h in
                                self.get_parameter('heading_offsets_deg').value]
        self.sensor_offset_x = self.get_parameter('sensor_offset_x').value
        self.sensor_offset_y = self.get_parameter('sensor_offset_y').value

        os.makedirs(self.results_dir, exist_ok=True)

        self.declare_parameter('debug_monitor_log', True)
        self.debug_monitor_log = self.get_parameter('debug_monitor_log').value

        self.declare_parameter('cmd_topic', '/dsdbot_base_controller/cmd_vel')
        self.declare_parameter('joint_states_topic', '/joint_states')
        cmd_topic = self.get_parameter('cmd_topic').value
        js_topic = self.get_parameter('joint_states_topic').value

        # PGV pose subscriber
        self.pgv_x = 0.0
        self.pgv_y = 0.0
        self.pgv_yaw = 0.0
        self.pose_sub = self.create_subscription(
            PoseStamped, pose_topic, self._on_pose, 10)

        # Cmd vel subscriber (TwistStamped)
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_wz = 0.0
        self.cmd_sub = self.create_subscription(
            TwistStamped, cmd_topic, self._on_cmd, 10)

        # Joint states subscriber (for steering change metric)
        self.joint_names = []
        self.joint_positions = []
        self.js_sub = self.create_subscription(
            JointState, js_topic, self._on_joint_states, 10)

        # Timeseries buffer (filled during trial monitoring)
        self.ts_buffer = []
        self.recording = False

        # Kinematic positioning publisher (pgv_controller)
        self.positioning_pub = self.create_publisher(
            PoseStamped, '/pgv_controller/goal', 10)

        # pgv_controller cancel service client
        from std_srvs.srv import Trigger as TriggerSrv
        self.cancel_client = self.create_client(TriggerSrv, '/pgv_controller/cancel')
        self._TriggerSrv = TriggerSrv

        # TF buffer for base_link lookup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # FollowPath action client
        self.follow_path_client = ActionClient(self, FollowPath, 'follow_path')

        # Results: one folder per experiment run
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.experiment_dir = os.path.join(
            self.results_dir, f'{self.controller_name}_{timestamp}')
        self.ts_dir = os.path.join(self.experiment_dir, 'timeseries')
        os.makedirs(self.ts_dir, exist_ok=True)
        self.csv_path = os.path.join(self.experiment_dir, 'summary.csv')

        # 파라미터 스냅샷 저장
        self._save_params_snapshot()

        self.get_logger().info(
            f'[experiment_runner] controller={self.controller_name}, '
            f'results={self.csv_path}')

    def _save_params_snapshot(self):
        """실험 시작 시 모든 파라미터를 결과 디렉터리에 저장."""
        import json
        snapshot = {}
        for name in self._parameters:
            p = self.get_parameter(name)
            val = p.value
            snapshot[name] = val
        snap_path = os.path.join(self.experiment_dir, 'params.json')
        with open(snap_path, 'w') as f:
            json.dump(snapshot, f, indent=2, default=str)
        # test_experiments.yaml 복사 (Nav2 controller 파라미터)
        yaml_src = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'config', 'test_experiments.yaml')
        if os.path.exists(yaml_src):
            import shutil
            shutil.copy2(yaml_src, os.path.join(
                self.experiment_dir, 'test_experiments.yaml'))

    def _on_pose(self, msg: PoseStamped):
        self.pgv_x = msg.pose.position.x
        self.pgv_y = msg.pose.position.y
        self.pgv_yaw = yaw_from_quaternion(msg.pose.orientation)

        if self.recording:
            row = {
                't': self.get_clock().now().nanoseconds / 1e9,
                'x': self.pgv_x,
                'y': self.pgv_y,
                'yaw': self.pgv_yaw,
                'vx': self.cmd_vx,
                'vy': self.cmd_vy,
                'wz': self.cmd_wz,
            }
            # base_link position in map frame (via TF)
            try:
                tf_map_base = self.tf_buffer.lookup_transform(
                    'map', 'base_link', rclpy.time.Time())
                row['bl_x'] = tf_map_base.transform.translation.x
                row['bl_y'] = tf_map_base.transform.translation.y
                row['bl_yaw'] = yaw_from_quaternion(
                    tf_map_base.transform.rotation)
            except tf2_ros.TransformException:
                row['bl_x'] = float('nan')
                row['bl_y'] = float('nan')
                row['bl_yaw'] = float('nan')
            for name, pos in zip(self.joint_names, self.joint_positions):
                row[name] = pos
            self.ts_buffer.append(row)

    def _on_cmd(self, msg: TwistStamped):
        self.cmd_vx = msg.twist.linear.x
        self.cmd_vy = msg.twist.linear.y
        self.cmd_wz = msg.twist.angular.z

    def _on_joint_states(self, msg: JointState):
        self.joint_names = list(msg.name)
        self.joint_positions = list(msg.position)

    def make_path(self, start_x, start_y, start_yaw, goal_x, goal_y, goal_yaw,
                  step=0.001):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()

        dx = goal_x - start_x
        dy = goal_y - start_y
        dist = math.hypot(dx, dy)
        n_steps = max(int(dist / step), 1)

        for i in range(n_steps + 1):
            t = i / n_steps
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = start_x + dx * t
            pose.pose.position.y = start_y + dy * t
            dyaw = math.atan2(math.sin(goal_yaw - start_yaw),
                              math.cos(goal_yaw - start_yaw))
            yaw = start_yaw + dyaw * t
            pose.pose.orientation = quaternion_from_yaw(yaw)
            path.poses.append(pose)

        return path

    def move_to_start(self, target_x, target_y=0.0, target_yaw=0.0, timeout=10.0):
        """pgv_controller로 로봇을 시작 위치(x, y, yaw)로 이동."""
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = float(target_x)
        goal_msg.pose.position.y = float(target_y)
        goal_msg.pose.orientation = quaternion_from_yaw(target_yaw)
        self.positioning_pub.publish(goal_msg)
        self.get_logger().info(
            f'[positioning] Moving to x={target_x:.4f}, y={target_y:.4f}, '
            f'yaw={math.degrees(target_yaw):.1f}')

        t_start = self.get_clock().now()
        stable_count = 0
        required_stable = 25  # 0.5s at 50Hz

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            dist = math.hypot(target_x - self.pgv_x, target_y - self.pgv_y)
            yaw_err = abs(math.atan2(
                math.sin(target_yaw - self.pgv_yaw),
                math.cos(target_yaw - self.pgv_yaw)))

            if dist < 0.005 and yaw_err < math.radians(1.0):  # 5mm + 1° tolerance
                stable_count += 1
                if stable_count >= required_stable:
                    self.get_logger().info(
                        f'[positioning] Arrived at x={self.pgv_x:.4f}, '
                        f'y={self.pgv_y:.4f}, yaw={math.degrees(self.pgv_yaw):.1f}')
                    return True
            else:
                stable_count = 0

            elapsed = (self.get_clock().now() - t_start).nanoseconds / 1e9
            if elapsed > timeout:
                self.get_logger().warn(
                    f'[positioning] Timeout (dist={dist:.4f}m, '
                    f'x={self.pgv_x:.4f}, y={self.pgv_y:.4f})')
                return False

        return False

    def run_single_trial(self, trial_id, direction, repeat,
                         goal_x, goal_y, goal_yaw):
        self.get_logger().info(
            f'[trial {trial_id}] dir={direction}, rep={repeat}, '
            f'goal=({goal_x:.3f}, {goal_y:.3f}, {math.degrees(goal_yaw):.1f})')

        if self.controller_name == 'kinematic':
            return self._run_trial_kinematic(trial_id, goal_x, goal_y, goal_yaw)
        else:
            return self._run_trial_action(trial_id, goal_x, goal_y, goal_yaw)

    def _run_trial_kinematic(self, trial_id, goal_x, goal_y, goal_yaw):
        """kinematic: pgv_controller에 직접 goal 발행 + 모니터링."""
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = float(goal_x)
        goal_msg.pose.position.y = float(goal_y)
        goal_msg.pose.orientation = quaternion_from_yaw(goal_yaw)
        self.positioning_pub.publish(goal_msg)

        return self._monitor_trial(trial_id, goal_x, goal_y)

    def _run_trial_action(self, trial_id, goal_x, goal_y, goal_yaw):
        """MPPI/DWB: FollowPath action 호출 + 모니터링.
        goal은 센서 좌표(마커) 기준이므로, base_link 기준으로 오프셋 보정."""
        # pgv_controller 정지 (positioning 잔여 명령 차단)
        if self.cancel_client.service_is_ready():
            self.cancel_client.call_async(self._TriggerSrv.Request())

        if not self.follow_path_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('FollowPath action server not available')
            return 'ERROR', 0.0, self.pgv_x, self.pgv_y, self.pgv_yaw

        # goal: 센서 좌표 → base_link 좌표 변환
        c_goal = math.cos(goal_yaw)
        s_goal = math.sin(goal_yaw)
        bl_goal_x = goal_x - (self.sensor_offset_x * c_goal - self.sensor_offset_y * s_goal)
        bl_goal_y = goal_y - (self.sensor_offset_x * s_goal + self.sensor_offset_y * c_goal)

        # start: TF에서 현재 base_link 위치를 직접 조회
        try:
            tf_map_base = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
            bl_start_x = tf_map_base.transform.translation.x
            bl_start_y = tf_map_base.transform.translation.y
            bl_start_yaw = yaw_from_quaternion(
                tf_map_base.transform.rotation)
        except tf2_ros.TransformException as e:
            self.get_logger().error(f'TF lookup failed: {e}')
            return 'ERROR', 0.0, self.pgv_x, self.pgv_y, self.pgv_yaw

        self.get_logger().info(
            f'[action] bl_start=({bl_start_x:.4f}, {bl_start_y:.4f}, '
            f'{math.degrees(bl_start_yaw):.1f}°) '
            f'bl_goal=({bl_goal_x:.4f}, {bl_goal_y:.4f}, '
            f'{math.degrees(goal_yaw):.1f}°)')

        path = self.make_path(bl_start_x, bl_start_y, bl_start_yaw,
                              bl_goal_x, bl_goal_y, goal_yaw)
        goal_msg = FollowPath.Goal()
        goal_msg.path = path
        goal_msg.controller_id = self.controller_name.upper()

        send_goal_future = self.follow_path_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=5.0)

        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Goal rejected or server unavailable')
            return 'REJECTED', 0.0, self.pgv_x, self.pgv_y, self.pgv_yaw

        result_future = goal_handle.get_result_async()
        result = self._monitor_trial_action(
            trial_id, goal_x, goal_y, result_future)

        if not result_future.done():
            goal_handle.cancel_goal_async()
        return result

    def _monitor_trial(self, trial_id, goal_x, goal_y):
        """공통 모니터링 루프."""
        self.ts_buffer = []
        self.recording = True

        t_start = self.get_clock().now()
        settle_accumulated = 0.0
        success_count = 0
        result = 'UNKNOWN'
        last_debug_log = 0.0

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            now = self.get_clock().now()
            dt = 0.02

            elapsed = (now - t_start).nanoseconds / 1e9

            if self.debug_monitor_log and elapsed - last_debug_log >= 0.2:
                last_debug_log = elapsed
                self.get_logger().info(
                    f'[monitor] t={elapsed:.1f} '
                    f'pos=({self.pgv_x:.4f}, {self.pgv_y:.4f}, '
                    f'{math.degrees(self.pgv_yaw):.1f}°) '
                    f'vel=({self.cmd_vx:.4f}, {self.cmd_vy:.4f}, '
                    f'{self.cmd_wz:.4f})')

            dist = math.hypot(goal_x - self.pgv_x, goal_y - self.pgv_y)

            if dist < self.success_thresh:
                success_count += 1
                if success_count >= self.success_samples:
                    result = 'SUCCESS'
                    break
            else:
                success_count = 0

            if dist < self.settle_thresh:
                settle_accumulated += dt
                if settle_accumulated > self.settle_timeout:
                    result = 'TIMEOUT_SETTLE'
                    break

            if elapsed > self.total_timeout:
                result = 'TIMEOUT_TOTAL'
                break

        self.recording = False

        elapsed = (self.get_clock().now() - t_start).nanoseconds / 1e9
        final_x = self.pgv_x
        final_y = self.pgv_y
        final_yaw = self.pgv_yaw
        self.get_logger().info(
            f'[trial {trial_id}] result={result}, elapsed={elapsed:.2f}s, '
            f'final=({final_x:.4f}, {final_y:.4f})')

        self._write_timeseries(trial_id)

        return result, elapsed, final_x, final_y, final_yaw

    def _monitor_trial_action(self, trial_id, goal_x, goal_y, result_future):
        """Action 기반 모니터링.

        Phase 1 (CTRL): controller 완료까지 대기하며 센서 데이터 기록.
        Phase 2 (SETTLE): controller 완료 후 센서 수렴 여부를 판정.
        """
        self.ts_buffer = []
        self.recording = True

        t_start = self.get_clock().now()
        success_count = 0
        result = 'UNKNOWN'
        last_debug_log = 0.0
        controller_done = False
        controller_done_time = None

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            now = self.get_clock().now()
            elapsed = (now - t_start).nanoseconds / 1e9

            # controller 완료 감지
            if not controller_done and result_future.done():
                controller_done = True
                controller_done_time = now
                dist_now = math.hypot(goal_x - self.pgv_x,
                                      goal_y - self.pgv_y) * 1000
                self.get_logger().info(
                    f'[trial {trial_id}] Controller done at t={elapsed:.2f}s, '
                    f'sensor dist={dist_now:.1f}mm, '
                    f'waiting {self.post_controller_timeout}s for sensor...')

            if self.debug_monitor_log and elapsed - last_debug_log >= 0.2:
                last_debug_log = elapsed
                phase = 'SETTLE' if controller_done else 'CTRL'
                self.get_logger().info(
                    f'[monitor:{phase}] t={elapsed:.1f} '
                    f'pos=({self.pgv_x:.4f}, {self.pgv_y:.4f}, '
                    f'{math.degrees(self.pgv_yaw):.1f}\u00b0) '
                    f'vel=({self.cmd_vx:.4f}, {self.cmd_vy:.4f}, '
                    f'{self.cmd_wz:.4f})')

            dist = math.hypot(goal_x - self.pgv_x, goal_y - self.pgv_y)

            # Phase 2: controller 완료 후 센서 수렴 판정
            if controller_done:
                if dist < self.success_thresh:
                    success_count += 1
                    if success_count >= self.success_samples:
                        result = 'SUCCESS'
                        break
                else:
                    success_count = 0

                time_since_done = (now - controller_done_time).nanoseconds / 1e9
                if time_since_done > self.post_controller_timeout:
                    result = 'TIMEOUT_SETTLE'
                    self.get_logger().warn(
                        f'[trial {trial_id}] Sensor not converged after '
                        f'controller done (dist={dist*1000:.1f}mm)')
                    break

            # 절대 타임아웃 (controller가 끝나지 않는 경우 대비)
            if elapsed > self.total_timeout:
                result = 'TIMEOUT_TOTAL'
                break

        self.recording = False

        elapsed = (self.get_clock().now() - t_start).nanoseconds / 1e9
        final_x = self.pgv_x
        final_y = self.pgv_y
        final_yaw = self.pgv_yaw
        self.get_logger().info(
            f'[trial {trial_id}] result={result}, elapsed={elapsed:.2f}s, '
            f'final=({final_x:.4f}, {final_y:.4f})')

        self._write_timeseries(trial_id)

        return result, elapsed, final_x, final_y, final_yaw

    def write_csv_row(self, row):
        file_exists = os.path.isfile(self.csv_path)
        with open(self.csv_path, 'a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=[
                'trial_id', 'controller', 'direction_deg', 'heading_offset_deg',
                'repeat',
                'start_x', 'start_y', 'start_yaw_deg',
                'goal_x', 'goal_y', 'goal_yaw_deg',
                'final_x', 'final_y', 'final_yaw_deg',
                'result', 'elapsed_sec'])
            if not file_exists:
                writer.writeheader()
            writer.writerow(row)

    def _write_timeseries(self, trial_id):
        if not self.ts_buffer:
            return
        ts_path = os.path.join(self.ts_dir, f'trial_{trial_id:03d}.csv')
        t0 = self.ts_buffer[0]['t']
        # Detect joint columns from first row
        base_fields = ['t', 'x', 'y', 'yaw', 'vx', 'vy', 'wz',
                       'bl_x', 'bl_y', 'bl_yaw']
        joint_fields = sorted(k for k in self.ts_buffer[0]
                              if k not in base_fields)
        fields = base_fields + joint_fields
        with open(ts_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            for row in self.ts_buffer:
                out = {
                    't': round(row['t'] - t0, 4),
                    'x': round(row['x'], 6),
                    'y': round(row['y'], 6),
                    'yaw': round(row['yaw'], 6),
                    'vx': round(row['vx'], 6),
                    'vy': round(row['vy'], 6),
                    'wz': round(row['wz'], 6),
                    'bl_x': round(row['bl_x'], 6) if not math.isnan(row['bl_x']) else '',
                    'bl_y': round(row['bl_y'], 6) if not math.isnan(row['bl_y']) else '',
                    'bl_yaw': round(row['bl_yaw'], 6) if not math.isnan(row['bl_yaw']) else '',
                }
                for jf in joint_fields:
                    out[jf] = round(row.get(jf, 0.0), 6)
                writer.writerow(out)
        self.get_logger().info(
            f'[trial {trial_id}] timeseries: {len(self.ts_buffer)} samples -> {ts_path}')


def main(args=None):
    rclpy.init(args=args)
    node = ExperimentRunnerNode()

    # Wait for pose data
    node.get_logger().info('[experiment_runner] Waiting for PGV pose...')
    while rclpy.ok() and node.pgv_x == 0.0 and node.pgv_y == 0.0:
        rclpy.spin_once(node, timeout_sec=0.5)
    node.get_logger().info('[experiment_runner] PGV pose received, starting in 1s...')
    import time
    time.sleep(1.0)

    # Generate trials: 8 directions x headings x repeats
    start_distance = 0.03  # 3cm from goal
    n_directions = 8
    repeats = node.repeats
    heading_offsets = node.heading_offsets
    goal_x = 0.0
    goal_y = 0.0
    goal_yaw = 0.0

    # 센서 오프셋: base_link ↔ sensor 변환에 사용
    ox = node.sensor_offset_x
    oy = node.sensor_offset_y

    # base_link 기준 목표 위치 (센서 목표 + heading=0에서의 역변환)
    c_g = math.cos(goal_yaw)
    s_g = math.sin(goal_yaw)
    bl_goal_x = goal_x - ox * c_g + oy * s_g
    bl_goal_y = goal_y - ox * s_g - oy * c_g

    node.get_logger().info(
        f'[experiment_runner] base_link goal=({bl_goal_x:.4f}, {bl_goal_y:.4f}), '
        f'sensor goal=({goal_x}, {goal_y}), offset=({ox}, {oy})')

    trials = []
    trial_id = 0
    for rep in range(repeats):
        for d in range(n_directions):
            angle = d * (2.0 * math.pi / n_directions)
            # base_link 기준 시작점: base_link 목표에서 3cm 떨어진 위치
            bl_start_x = bl_goal_x + start_distance * math.cos(angle)
            bl_start_y = bl_goal_y + start_distance * math.sin(angle)
            for h_offset in heading_offsets:
                start_yaw = h_offset
                # base_link → sensor 좌표 변환
                c = math.cos(start_yaw)
                s = math.sin(start_yaw)
                start_x = bl_start_x + ox * c - oy * s
                start_y = bl_start_y + ox * s + oy * c
                trial_id += 1
                trials.append({
                    'trial_id': trial_id,
                    'direction_deg': round(math.degrees(angle), 1),
                    'heading_offset_deg': round(math.degrees(h_offset), 1),
                    'repeat': rep + 1,
                    'start_x': start_x,
                    'start_y': start_y,
                    'start_yaw_target': start_yaw,
                    'goal_x': goal_x,
                    'goal_y': goal_y,
                    'goal_yaw': goal_yaw,
                })

    node.get_logger().info(
        f'[experiment_runner] Running {len(trials)} trials '
        f'({n_directions} dirs x {len(heading_offsets)} headings x {repeats} reps)')

    try:
        for trial in trials:
            if not rclpy.ok():
                break

            # (1) Kinematic으로 시작 위치(x)로 이동
            h_info = f', heading={trial["heading_offset_deg"]:.0f}' if trial['heading_offset_deg'] != 0 else ''
            node.get_logger().info(
                f'\n=== Trial {trial["trial_id"]}/{len(trials)} '
                f'(dir={trial["direction_deg"]:.0f} deg{h_info}, rep={trial["repeat"]}) ===')
            max_positioning_attempts = 3
            pos_ok = False
            for attempt in range(max_positioning_attempts):
                pos_ok = node.move_to_start(
                    trial['start_x'], trial['start_y'], trial['start_yaw_target'])
                if pos_ok:
                    break
                node.get_logger().warn(
                    f'[trial {trial["trial_id"]}] Positioning attempt '
                    f'{attempt+1}/{max_positioning_attempts} failed')

            if not pos_ok:
                node.get_logger().error(
                    f'[trial {trial["trial_id"]}] Positioning failed after '
                    f'{max_positioning_attempts} attempts, SKIPPING')
                node.write_csv_row({
                    'trial_id': trial['trial_id'],
                    'controller': node.controller_name,
                    'direction_deg': trial['direction_deg'],
                    'heading_offset_deg': trial['heading_offset_deg'],
                    'repeat': trial['repeat'],
                    'start_x': node.pgv_x,
                    'start_y': node.pgv_y,
                    'start_yaw_deg': round(math.degrees(node.pgv_yaw), 2),
                    'goal_x': trial['goal_x'],
                    'goal_y': trial['goal_y'],
                    'goal_yaw_deg': math.degrees(trial['goal_yaw']),
                    'final_x': node.pgv_x,
                    'final_y': node.pgv_y,
                    'final_yaw_deg': round(math.degrees(node.pgv_yaw), 2),
                    'result': 'SKIP_POSITIONING',
                    'elapsed_sec': 0.0,
                })
                continue

            # (2) 안정화 대기
            time.sleep(0.3)

            # (3) 실제 시작 위치 기록
            actual_start_x = node.pgv_x
            actual_start_y = node.pgv_y
            actual_start_yaw = node.pgv_yaw
            node.get_logger().info(
                f'[trial {trial["trial_id"]}] actual start: '
                f'x={actual_start_x:.4f}, y={actual_start_y:.4f}, '
                f'yaw={math.degrees(actual_start_yaw):.1f} deg')

            # (4) 시행 실행 (설정된 컨트롤러로 goal까지 이동)
            result, elapsed, final_x, final_y, final_yaw = node.run_single_trial(
                trial['trial_id'],
                trial['direction_deg'],
                trial['repeat'],
                trial['goal_x'],
                trial['goal_y'],
                trial['goal_yaw'])

            # (5) CSV 기록
            node.write_csv_row({
                'trial_id': trial['trial_id'],
                'controller': node.controller_name,
                'direction_deg': trial['direction_deg'],
                'heading_offset_deg': trial['heading_offset_deg'],
                'repeat': trial['repeat'],
                'start_x': round(actual_start_x, 6),
                'start_y': round(actual_start_y, 6),
                'start_yaw_deg': round(math.degrees(actual_start_yaw), 2),
                'goal_x': trial['goal_x'],
                'goal_y': trial['goal_y'],
                'goal_yaw_deg': math.degrees(trial['goal_yaw']),
                'final_x': round(final_x, 6),
                'final_y': round(final_y, 6),
                'final_yaw_deg': round(math.degrees(final_yaw), 2),
                'result': result,
                'elapsed_sec': round(elapsed, 3),
            })

            # (6) 시행 간 간격
            time.sleep(0.2)

        node.get_logger().info(
            f'[experiment_runner] All {len(trials)} trials complete. '
            f'CSV: {node.csv_path}')

        node.get_logger().info(
            f'[experiment_runner] All {len(trials)} trials complete. '
            f'CSV: {node.csv_path}')

    except KeyboardInterrupt:
        node.get_logger().info('[experiment_runner] Interrupted by user')
    finally:
        exp_dir = node.experiment_dir
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
        # 완료/중단 관계없이 현재까지 기록된 데이터로 분석 실행
        _run_analyzer(exp_dir)


def _run_analyzer(experiment_dir):
    from icros_test_setup.experiment_analyzer import analyze
    analyze(experiment_dir)
