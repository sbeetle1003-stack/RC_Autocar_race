#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class StudentNode(Node):

    def __init__(self):
        super().__init__('student')

        # 'memo' 토픽 구독
        self.subscription = self.create_subscription(
            Int32, 'memo', self.callback, 10)

        # 수신한 데이터 저장
        self.data = 0

        # 1초마다 수신한 데이터 출력
        self.timer = self.create_timer(
            1.0, self.control_loop)

        self.get_logger().info(
            '----- Student node started -----')

    def callback(self, msg):
        # 수신한 메시지 저장
        self.data = msg.data

    def control_loop(self):
        # 현재 저장된 데이터 출력
        self.get_logger().info(
            f'Received data : {self.data}')


def main(args=None):

    # ROS2 초기화
    rclpy.init(args=args)

    # Student 노드 생성
    node = StudentNode()

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
