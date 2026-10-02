#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from xycar_msgs.msg import XycarMotor


class DriverNode(Node):

    def __init__(self):
        super().__init__('driver')

        self.motor_publisher = self.create_publisher(
            XycarMotor, 'xycar_motor', 1)

        self.motor_msg = XycarMotor()

        # speed 파라미터 선언
        self.declare_parameter('speed', 10.0)

        # speed 파라미터 읽기
        self.speed = self.get_parameter('speed').value

        self.get_logger().info(
            f'----- Speed: {self.speed} -----')

        # 차량 초기 상태: 핸들 중앙 + 정지
        self.drive(angle=0, speed=0)

        # 현재 상태
        self.is_driving = False

        # 현재 상태가 시작된 시간
        self.state_start_time = self.get_clock().now()

        # 0.1초마다 차량 제어
        self.control_timer = self.create_timer(
            0.1, self.control_loop)

        self.get_logger().info(
            '----- Xycar self-driving node started -----')

    def drive(self, angle, speed):
        self.motor_msg.angle = float(angle)
        self.motor_msg.speed = float(speed)

        self.motor_publisher.publish(self.motor_msg)

    def control_loop(self):

        # 현재 상태에서 경과한 시간(초)
        elapsed = (
            self.get_clock().now() - self.state_start_time
        ).nanoseconds / 1e9

        if self.is_driving:

            # 주행 상태: 3초
            self.drive(angle=0, speed=self.speed)

            if elapsed >= 3.0:
                self.is_driving = False
                self.state_start_time = self.get_clock().now()

        else:

            # 정지 상태: 2초
            self.drive(angle=0, speed=0)

            if elapsed >= 2.0:
                self.is_driving = True
                self.state_start_time = self.get_clock().now()


def main(args=None):

    rclpy.init(args=args)
    node = DriverNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        # 종료 전에 차량 정지
        node.drive(angle=0, speed=0)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

