#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from xycar_msgs.msg import XycarMotor


# =============================================
# AR Drive 결과를 차량 제어로 변환하는 Node
# =============================================
class MyARDriveNode(Node):

    def __init__(self):

        super().__init__('my_ar_drive')

        # =============================================
        # 고정 주행 속도
        # =============================================
        self.Fix_Speed = 10.0

        # =============================================
        # /ardrive_result Subscriber
        # [Angle, ID, X, Y, Z]
        # =============================================
        self.subscription = self.create_subscription(
            Float32MultiArray, 'ardrive_result', self.drive_callback, 1)

        # =============================================
        # /xycar_motor Publisher
        # =============================================
        self.publisher = self.create_publisher(
            XycarMotor, 'xycar_motor', 1)

        # =============================================
        # 초기 정지
        # =============================================
        self.drive(0.0, 0.0)

        self.get_logger().info('----- My AR Drive node started -----')

    # =============================================
    # /ardrive_result Callback
    # =============================================
    def drive_callback(self, msg):

        # =============================================
        # 데이터가 없는 경우
        # =============================================
        if len(msg.data) < 5:
            return

        # =============================================
        # ardrive_result
        # [Angle, ID, X, Y, Z]
        # =============================================
        angle = msg.data[0]

        tag_id = msg.data[1]
        x_pos = msg.data[2]
        y_pos = msg.data[3]
        z_pos = msg.data[4]

        # ============================================= 
        # 차량 속도 결정 
        # Z값이 20cm 이하이면 정차 
        # Z값이 20cm보다 크면 정상 주행 
        # ============================================= 
        if z_pos <= 20.0: 
            speed = 0.0 
        else: 
            speed = self.Fix_Speed

        # =============================================
        # 차량 제어
        # =============================================
        self.drive(angle, speed)

        # =============================================
        # 상태 출력
        # =============================================
        self.get_logger().info(
            f'ID={int(tag_id)} '
            f'X={x_pos:.1f} '
            f'Z={z_pos:.1f} '
            f'Angle={angle:.1f} '
        )

    # =============================================
    # /xycar_motor Publish
    # =============================================
    def drive(self, angle, speed):

        motor_msg = XycarMotor()

        motor_msg.angle = float(angle)
        motor_msg.speed = float(speed)

        self.publisher.publish(motor_msg)


# =============================================
# Main
# =============================================
def main(args=None):

    rclpy.init(args=args)
    node = MyARDriveNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        # 프로그램 종료 시 차량 정지
        node.drive(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
