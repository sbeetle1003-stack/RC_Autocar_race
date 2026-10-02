#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from xycar_msgs.msg import XycarMotor


class MyUltraDriveNode(Node):

    def __init__(self):
        super().__init__('my_ultra_drive')

        # /xycar_motor publisher
        self.motor_publisher = self.create_publisher(
            XycarMotor, 'xycar_motor', 1)

        # /ultradrive_result subscriber
        self.subscription = self.create_subscription(
            Float32MultiArray, 'ultradrive_result',
            self.callback, 1)

        self.motor_msg = XycarMotor()
        self.speed = 12.0

        # 시작할 때 정지
        self.drive(angle=0, speed=0)

        self.get_logger().info('----- My ultra drive node started -----')

    def drive(self, angle, speed):

        self.motor_msg.angle = float(angle)
        self.motor_msg.speed = float(speed)

        self.motor_publisher.publish(self.motor_msg)

    def callback(self, msg):

        angle = float(msg.data[0])

        # 차량 주행
        self.drive(angle=angle, speed=self.speed)


def main(args=None):

    rclpy.init(args=args)

    node = MyUltraDriveNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        # 종료 시 차량 정지
        node.drive(angle=0, speed=0)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
