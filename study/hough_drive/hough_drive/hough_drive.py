#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# =============================================
# Hough Drive Node
#
# /lanedrive_result
#     [angle, speed]
#
#        ↓
#
# /xycar_motor
#     XycarMotor
# =============================================

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from xycar_msgs.msg import XycarMotor


# =============================================
# Hough Drive Node
# =============================================

class HoughDriveNode(Node):

    def __init__(self):

        super().__init__('hough_drive')

        # =====================================
        # 차선 주행 결과 구독
        # =====================================

        self.subscription = self.create_subscription(Float32MultiArray, '/lanedrive_result', self.drive_callback, 1)

        # =====================================
        # Xycar Motor 명령 발행
        # =====================================

        self.publisher = self.create_publisher(XycarMotor, '/xycar_motor', 1)

        self.get_logger().info('Hough Drive Node Started')

    # =========================================
    # /lanedrive_result Callback
    # =========================================

    def drive_callback(self, msg):

        data = msg.data

        # =====================================
        # 데이터 검사
        # =====================================

        if len(data) < 2:

            self.get_logger().warning(f'Invalid lanedrive_result data: {len(data)} values')

            return

        # =====================================
        # angle / speed 추출
        #
        # data[0] : angle
        # data[1] : speed
        # =====================================

        angle = data[0]
        speed = data[1]

        # =====================================
        # Xycar angle 범위 제한
        # -100 : 최대 좌회전
        #    0 : 직진
        # +100 : 최대 우회전
        # =====================================

        angle = max(-100.0, min(100.0, angle))

        # =====================================
        # Xycar speed 범위 제한
        # -100 : 최대 후진
        #    0 : 정지
        # +100 : 최대 전진
        # =====================================

        speed = max(-100.0, min(100.0, speed))

        # =====================================
        # Xycar Motor 메시지 생성
        # =====================================

        motor_msg = XycarMotor()
        motor_msg.angle = angle
        motor_msg.speed = speed

        # =====================================
        # 모터 명령 발행
        # =====================================

        self.publisher.publish(motor_msg)

        # =====================================
        # 터미널 출력
        # =====================================

        self.get_logger().info(f'Angle={angle:.1f}, Speed={speed:.1f}')


# =============================================
# main
# =============================================

def main(args=None):

    rclpy.init(args=args)

    node = HoughDriveNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        # =====================================
        # 종료 시 차량 정지
        # =====================================

        motor_msg = XycarMotor()
        motor_msg.angle = 0.0
        motor_msg.speed = 0.0
        node.publisher.publish(motor_msg)

        node.destroy_node()
        rclpy.shutdown()


# =============================================
# 프로그램 시작
# =============================================

if __name__ == '__main__':
    main()
