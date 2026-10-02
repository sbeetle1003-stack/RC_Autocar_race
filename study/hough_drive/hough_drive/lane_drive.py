#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# =============================================
# Lane Drive Node
#
# /lanedetect_result
#     [found, x_left, x_right, x_midpoint, error]
#
# /lanedrive_result
#     [angle, speed]
# =============================================

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray


# =============================================
# Lane Drive Node
# =============================================

class LaneDriveNode(Node):

    def __init__(self):

        super().__init__('lane_drive')

        # =====================================
        # 차선 검출 결과 구독
        # =====================================

        self.subscription = self.create_subscription(Float32MultiArray, '/lanedetect_result', self.lane_callback, 1)

        # =====================================
        # 차선 주행 결과 발행
        # =====================================

        self.publisher = self.create_publisher(Float32MultiArray, '/lanedrive_result', 1)

        # =====================================
        # 자이카 angle 범위
        #
        # -100 : 최대 좌회전
        #    0 : 직진
        # +100 : 최대 우회전
        # =====================================

        self.max_angle = 100.0

        # =====================================
        # 자이카 speed 범위
        #
        # -100 : 최대 후진
        #    0 : 정지
        # +100 : 최대 전진
        # =====================================

        self.max_speed = 100.0

        # =====================================
        # 실제 주행에서 사용할 최대 속도
        # =====================================

        self.drive_speed = 12.0

        self.get_logger().info('Lane Drive Node Started')

    # =========================================
    # 차선 검출 결과 Callback
    # =========================================

    def lane_callback(self, msg):

        data = msg.data

        # =====================================
        # 데이터 검사
        # =====================================

        if len(data) < 5:
            self.get_logger().warning(f'Invalid lanedetect_result data: {len(data)} values')
            return

        # =====================================
        # 차선 정보 추출
        #
        # [found, x_left, x_right, x_midpoint, error]
        # =====================================

        found = data[0]
        x_left = data[1]
        x_right = data[2]
        x_midpoint = data[3]
        error = data[4]

        # =====================================
        # 차선 검출 실패
        # =====================================

        if found < 0.5:

            angle = 0.0
            speed = 0.0

            self.publish_result(angle, speed)

            self.get_logger().warning('Lane Detect Failed -> STOP')

            return

        # =====================================
        # 조향각 계산
        #
        # error가 음수 -> 왼쪽 조향
        # error가 양수 -> 오른쪽 조향
        #
        # error의 최대값을 약 +/-320으로 보고
        # angle을 +/-100 범위로 변환
        # =====================================

        angle = error * 100.0 / 320.0

        # =====================================
        # angle 범위 제한
        # =====================================

        angle = max(-100.0, min(100.0, angle))

        # =====================================
        # 조향 정도에 따른 속도 조절
        #
        # 직선 주행 -> 빠르게
        # 완만한 곡선 -> 중간 속도
        # 급커브 -> 느리게
        # =====================================

        abs_error = abs(error)

        if abs_error < 20:
            speed = self.drive_speed

        elif abs_error < 50:
            speed = self.drive_speed * 0.8

        elif abs_error < 80:
            speed = self.drive_speed * 0.6

        else:
            speed = self.drive_speed * 0.4

        # =====================================
        # 결과 발행
        # =====================================

        self.publish_result(angle, speed)

        # =====================================
        # 터미널 출력
        # =====================================

        self.get_logger().info(f'L={x_left:.1f}, R={x_right:.1f}, Mid={x_midpoint:.1f}, Error={error:.1f} -> Angle={angle:.1f}, Speed={speed:.1f}')

    # =========================================
    # /lanedrive_result 발행
    #
    # [angle, speed]
    # =========================================

    def publish_result(self, angle, speed):

        result_msg = Float32MultiArray()
        result_msg.data = [float(angle), float(speed)]

        self.publisher.publish(result_msg)


# =============================================
# main
# =============================================

def main(args=None):

    rclpy.init(args=args)

    node = LaneDriveNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


# =============================================
# 프로그램 시작
# =============================================

if __name__ == '__main__':
    main()
