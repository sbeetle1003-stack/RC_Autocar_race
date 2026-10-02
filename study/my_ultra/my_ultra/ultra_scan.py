#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32MultiArray


class UltraNode(Node):

    def __init__(self):
        super().__init__('ultra_node')

        # QoS 설정
        # 초음파 센서는 최신 데이터가 중요하므로 Best Effort 사용
        qos_profile = QoSProfile(
            depth=3, reliability=ReliabilityPolicy.BEST_EFFORT)

        # 초음파 데이터 구독
        self.subscription = self.create_subscription(
            Int32MultiArray, 'xycar_ultrasonic', self.callback, qos_profile)

        # 가장 최근에 받은 초음파 데이터
        self.ultra_data = None

        # 1초마다 초음파 데이터 출력
        self.timer = self.create_timer(
            1.0, self.timer_callback)

        self.get_logger().info(
            '----- Ultrasonic node started -----')

    def callback(self, msg):

        # 수신된 최신 데이터만 저장
        self.ultra_data = msg.data

    def timer_callback(self):

        # 아직 데이터를 받지 못했다면 출력하지 않음
        if self.ultra_data is not None:

            # 1초마다 최신 초음파 데이터 출력
            self.get_logger().info(
                f'Ultrasonic Data: {self.ultra_data}')


def main(args=None):

    # ROS2 초기화
    rclpy.init(args=args)

    # UltraNode 생성
    node = UltraNode()

    try:
        # ROS2 이벤트 처리
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        # 노드 종료
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
