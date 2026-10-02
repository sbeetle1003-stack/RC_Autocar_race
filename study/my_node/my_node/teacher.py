#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class TeacherNode(Node):

    def __init__(self):
        super().__init__('teacher')

        # 'memo' 토픽으로 Int32 메시지를 발행
        self.publisher = self.create_publisher(
            Int32, 'memo', 10)

        # 카운터 초기값
        self.count = 1

        # 1초마다 control_loop() 실행
        self.timer = self.create_timer(
            1.0, self.control_loop)

        self.get_logger().info(
            '----- Teacher node started -----')

    def control_loop(self):

        # Int32 메시지 생성
        msg = Int32()

        # 현재 카운터 값을 메시지에 저장
        msg.data = self.count

        # 메시지 발행
        self.publisher.publish(msg)

        # 발행한 값 출력
        self.get_logger().info(
            f'Publishing: {self.count}')

        # 카운터 증가
        self.count += 1


def main(args=None):

    # ROS2 초기화
    rclpy.init(args=args)

    # Teacher 노드 생성
    node = TeacherNode()

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