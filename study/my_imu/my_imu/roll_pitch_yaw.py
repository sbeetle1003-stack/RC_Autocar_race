#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion


class ImuNode(Node):

    def __init__(self):
        super().__init__('imu_print')

        # 최신 IMU 데이터 저장
        self.imu_msg = None

        # '/imu/data' 토픽 구독
        self.subscription = self.create_subscription(
            Imu, '/imu/data', self.callback, qos_profile_sensor_data)

        # IMU 데이터 출력 Timer
        # 1초마다 Roll, Pitch, Yaw 출력
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info(
            '----- IMU node started -----')

    def callback(self, msg):
        # IMU의 Orientation 데이터 저장
        #
        # Quaternion
        # x, y, z, w
        self.imu_msg = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]

    def timer_callback(self):
        # IMU 데이터가 아직 수신되지 않았으면 종료
        if self.imu_msg is None:
            return

        # Quaternion → Euler Angle 변환
        roll, pitch, yaw = euler_from_quaternion(self.imu_msg)

        # Roll, Pitch, Yaw 출력
        self.get_logger().info(
            f'Roll: {roll:.4f}, Pitch: {pitch:.4f}, Yaw: {yaw:.4f}')


def main(args=None):

    # ROS2 초기화
    rclpy.init(args=args)

    # IMU 노드 생성
    node = ImuNode()

    try:
        # 메시지 수신 및 Timer 실행
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        # 노드 종료
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

