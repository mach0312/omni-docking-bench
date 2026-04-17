"""Dummy PGV Node.

PGV 센서 없이 테스트하기 위한 더미 노드.
dsdbot_base_controller의 odom을 받아서 /pgv/raw_pose로 변환 발행.
선택적으로 센서 오프셋, 노이즈를 시뮬레이션.

사용 환경:
  - PGV 미연결 목업 테스트
  - dsdbot_base_controller만 연결된 상태
  - 시뮬레이션 아닌 실물 컨트롤러 + 더미 센서
"""
import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class DummyPGVNode(Node):
    def __init__(self):
        super().__init__('dummy_pgv_node')

        # 파라미터
        self.declare_parameter('odom_topic', '/dsdbot_base_controller/odom')
        self.declare_parameter('pose_topic', '/pgv/raw_pose')
        self.declare_parameter('publish_rate', 50.0)  # Hz (PGV 실제 주기와 동일)

        # 센서 오프셋 시뮬레이션 (로봇 기준 센서 위치)
        self.declare_parameter('sim_sensor_offset_x', 0.0)   # [m]
        self.declare_parameter('sim_sensor_offset_y', 0.51)    # [m]
        self.declare_parameter('sim_sensor_yaw_offset_deg', 0.0)

        # 노이즈 시뮬레이션
        self.declare_parameter('pos_noise_std', 0.0002)  # [m] PGV 정밀도 수준
        self.declare_parameter('yaw_noise_std_deg', 0.05)  # [deg]

        # 초기 위치 오프셋 (map 프레임에서의 시작 위치)
        self.declare_parameter('init_x', 0.0)
        self.declare_parameter('init_y', 0.0)
        self.declare_parameter('init_yaw_deg', 0.0)

        # map→odom TF 발행
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')

        odom_topic = self.get_parameter('odom_topic').value
        pose_topic = self.get_parameter('pose_topic').value
        rate = self.get_parameter('publish_rate').value

        self.offset_x = self.get_parameter('sim_sensor_offset_x').value
        self.offset_y = self.get_parameter('sim_sensor_offset_y').value
        self.offset_yaw = math.radians(self.get_parameter('sim_sensor_yaw_offset_deg').value)
        self.pos_noise = self.get_parameter('pos_noise_std').value
        self.yaw_noise = math.radians(self.get_parameter('yaw_noise_std_deg').value)
        self.init_x = self.get_parameter('init_x').value
        self.init_y = self.get_parameter('init_y').value
        self.init_yaw = math.radians(self.get_parameter('init_yaw_deg').value)
        self.publish_tf = self.get_parameter('publish_tf').value
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value

        # odom 수신
        self.latest_odom = None
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self._on_odom, 10)

        # PGV pose 발행
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=5)
        self.pose_pub = self.create_publisher(PoseStamped, pose_topic, qos)

        # map→odom TF broadcaster (노이즈 반영)
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
            self.current_pose_pub = self.create_publisher(
                PoseStamped, '/current_pose', 10)

        # 주기적 발행 (odom 주기와 무관하게 PGV 주기로)
        self.timer = self.create_timer(1.0 / rate, self._publish_pose)

        self.get_logger().info(
            f'[dummy_pgv] odom={odom_topic} -> pose={pose_topic} @ {rate}Hz')
        self.get_logger().info(
            f'[dummy_pgv] sensor_offset=({self.offset_x}, {self.offset_y}), '
            f'noise_pos={self.pos_noise*1000:.1f}mm, noise_yaw={math.degrees(self.yaw_noise):.2f}deg')

    def _on_odom(self, msg: Odometry):
        self.latest_odom = msg

    def _publish_pose(self):
        if self.latest_odom is None:
            return

        odom = self.latest_odom

        # odom에서 로봇 위치 추출
        robot_x = odom.pose.pose.position.x + self.init_x
        robot_y = odom.pose.pose.position.y + self.init_y
        robot_yaw = yaw_from_quaternion(odom.pose.pose.orientation) + self.init_yaw

        # 센서 위치 계산 (로봇 기준 오프셋 적용)
        # PGV가 로봇 왼쪽에 있으면 offset_y > 0
        sensor_x = robot_x + (self.offset_x * math.cos(robot_yaw)
                               - self.offset_y * math.sin(robot_yaw))
        sensor_y = robot_y + (self.offset_x * math.sin(robot_yaw)
                               + self.offset_y * math.cos(robot_yaw))
        sensor_yaw = robot_yaw + self.offset_yaw

        # 노이즈 추가
        if self.pos_noise > 0:
            sensor_x += random.gauss(0.0, self.pos_noise)
            sensor_y += random.gauss(0.0, self.pos_noise)
        if self.yaw_noise > 0:
            sensor_yaw += random.gauss(0.0, self.yaw_noise)

        # PoseStamped 발행 (PGV raw = 센서 위치 기준, map 프레임)
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = sensor_x
        pose.pose.position.y = sensor_y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(sensor_yaw / 2.0)
        pose.pose.orientation.w = math.cos(sensor_yaw / 2.0)

        self.pose_pub.publish(pose)

        # map→odom TF 발행 (노이즈 반영된 센서 데이터 기반)
        if self.publish_tf:
            # 노이즈 센서 → base_link 역변환 (pgv_localization과 동일 수식)
            bl_x = sensor_x - self.offset_x * math.cos(sensor_yaw) + self.offset_y * math.sin(sensor_yaw)
            bl_y = sensor_y - self.offset_x * math.sin(sensor_yaw) - self.offset_y * math.cos(sensor_yaw)
            bl_yaw = sensor_yaw

            # odom→base_link (odom에서 직접 추출, TF lookup 불필요)
            ob_x = odom.pose.pose.position.x
            ob_y = odom.pose.pose.position.y
            ob_yaw = yaw_from_quaternion(odom.pose.pose.orientation)

            # base_link의 map상 위치 발행
            cp = PoseStamped()
            cp.header = pose.header
            cp.pose.position.x = bl_x
            cp.pose.position.y = bl_y
            cp.pose.orientation.z = math.sin(bl_yaw / 2.0)
            cp.pose.orientation.w = math.cos(bl_yaw / 2.0)
            self.current_pose_pub.publish(cp)

            # map→odom = map→base_link × inverse(odom→base_link)
            cos_ob = math.cos(ob_yaw)
            sin_ob = math.sin(ob_yaw)
            inv_x = -(cos_ob * ob_x + sin_ob * ob_y)
            inv_y = -(-sin_ob * ob_x + cos_ob * ob_y)
            inv_yaw = -ob_yaw

            cos_bl = math.cos(bl_yaw)
            sin_bl = math.sin(bl_yaw)
            mo_x = bl_x + cos_bl * inv_x - sin_bl * inv_y
            mo_y = bl_y + sin_bl * inv_x + cos_bl * inv_y
            mo_yaw = bl_yaw + inv_yaw

            tf_msg = TransformStamped()
            tf_msg.header.stamp = pose.header.stamp
            tf_msg.header.frame_id = self.map_frame
            tf_msg.child_frame_id = self.odom_frame
            tf_msg.transform.translation.x = mo_x
            tf_msg.transform.translation.y = mo_y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation.z = math.sin(mo_yaw / 2.0)
            tf_msg.transform.rotation.w = math.cos(mo_yaw / 2.0)
            self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DummyPGVNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
