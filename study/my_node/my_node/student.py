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

        self.get_logger().info(
            '----- Student node started -----')

    def callback(self, msg):
        # 메시지를 받을 때마다 바로 출력
        self.get_logger().info(
            f'Received data : {msg.data}')


def main(args=None):

    # ROS2 초기화
    rclpy.init(args=args)

    # Student 노드 생성
    node = StudentNode()

    try:
        # 메시지 수신 및 callback 처리
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        # 노드 종료
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
