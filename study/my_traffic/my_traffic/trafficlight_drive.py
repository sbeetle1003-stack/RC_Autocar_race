#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================
# 본 프로그램은 자이트론에서 제작한 것입니다.
# 상업라이센스에 의해 제공되므로 무단배포 및 상업적 이용을 금합니다.
# 교육과 실습 용도로만 사용가능하며 외부유출은 금지됩니다.
# =============================================

import rclpy
from rclpy.node import Node

import cv2

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from xycar_msgs.msg import XycarMotor

from cv_bridge import CvBridge

from rclpy.qos import QoSProfile, ReliabilityPolicy


# =============================================
# Traffic Light Drive Logic
# =============================================
class TrafficLightDriveLogic:

    def __init__(self):

        # -----------------------------------------
        # 차량 주행 속도
        # -----------------------------------------
        self.speed = 10.0

        # -----------------------------------------
        # 직진 주행 시간
        # -----------------------------------------
        self.drive_time = 2.0

        # -----------------------------------------
        # 차량 상태
        #
        # STOP  : 정지 및 신호 확인
        # DRIVE : 2초 직진
        # -----------------------------------------
        self.state = 'STOP'

        # -----------------------------------------
        # DRIVE 시작 시간
        # -----------------------------------------
        self.start_time = None

    # =============================================
    # 신호등 결과 처리
    # =============================================
    def check_traffic_light(self, data, current_time):

        # -----------------------------------------
        # 데이터 형식 확인
        #
        # [Found, Red, Yellow, Left, Blue]
        # -----------------------------------------
        if len(data) != 5:

            return

        # -----------------------------------------
        # 신호등 발견 여부
        # -----------------------------------------
        found = data[0]

        # -----------------------------------------
        # Blue 신호
        #
        # 현재 3구 신호등에서는
        # Blue = 진행 신호로 사용
        # -----------------------------------------
        blue = data[4]

        # =========================================
        # STOP 상태
        # =========================================
        if self.state == 'STOP':

            # -------------------------------------
            # 신호등이 발견되고 Blue이면
            # 직진 시작
            # -------------------------------------
            if found == 1.0 and blue == 1.0:

                self.state = 'DRIVE'

                self.start_time = current_time

    # =============================================
    # 현재 차량 상태 계산
    # =============================================
    def get_motor_command(self, current_time):

        # =========================================
        # STOP 상태
        # =========================================
        if self.state == 'STOP':

            return 0.0, 0.0

        # =========================================
        # DRIVE 상태
        # =========================================
        if self.state == 'DRIVE':

            # -------------------------------------
            # 직진 시작 후 경과 시간
            # -------------------------------------
            elapsed_time = (
                current_time - self.start_time
            )

            # -------------------------------------
            # 2초 동안 직진
            # -------------------------------------
            if elapsed_time < self.drive_time:

                return 0.0, self.speed

            # -------------------------------------
            # 2초가 지나면 정지
            # -------------------------------------
            self.state = 'STOP'
            self.start_time = None

            return 0.0, 0.0

        # -----------------------------------------
        # 예외적인 경우
        # -----------------------------------------
        return 0.0, 0.0


# =============================================
# ROS2 Node
# =============================================
class TrafficLightDriveNode(Node):

    def __init__(self):

        super().__init__('trafficlight_drive')

        # -----------------------------------------
        # Traffic Light Drive Logic
        # -----------------------------------------
        self.logic = TrafficLightDriveLogic()

        # -----------------------------------------
        # CvBridge
        # -----------------------------------------
        self.bridge = CvBridge()

        # -----------------------------------------
        # Traffic Light QoS
        # -----------------------------------------
        qos_profile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # =========================================
        # /tflight_result Subscribe
        # =========================================
        self.result_subscription = self.create_subscription(
            Float32MultiArray,
            '/tflight_result',
            self.result_callback,
            1
        )

        # =========================================
        # /tflight_image Subscribe
        #
        # 디버깅용 영상
        # =========================================
        self.image_subscription = self.create_subscription(
            Image,
            '/tflight_image',
            self.image_callback,
            1
        )

        # =========================================
        # /xycar_motor Publisher
        # =========================================
        self.motor_publisher = self.create_publisher(
            XycarMotor,
            '/xycar_motor',
            1
        )

        # =========================================
        # 0.1초마다 차량 제어
        # =========================================
        self.timer = self.create_timer(
            0.1,
            self.control_callback
        )

        self.get_logger().info(
            'Traffic Light Drive started'
        )

    # =============================================
    # /tflight_result Callback
    # =============================================
    def result_callback(self, msg):

        # -----------------------------------------
        # 현재 ROS 시간
        # -----------------------------------------
        current_time = self.get_clock().now()

        # -----------------------------------------
        # 신호등 결과 처리
        # -----------------------------------------
        self.logic.check_traffic_light(
            msg.data,
            current_time.nanoseconds / 1e9
        )

    # =============================================
    # /tflight_image Callback
    # =============================================
    def image_callback(self, msg):

        try:

            # -------------------------------------
            # ROS Image -> OpenCV
            # -------------------------------------
            image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8')

            # -------------------------------------
            # 검출 결과 영상 표시
            # -------------------------------------
            cv2.imshow(
                'Traffic Light Drive',
                image
            )

            cv2.waitKey(1)

        except Exception as e:

            self.get_logger().error(
                f'Image conversion error: {e}'
            )

    # =============================================
    # 차량 제어 Callback
    # =============================================
    def control_callback(self):

        # -----------------------------------------
        # 현재 ROS 시간
        # -----------------------------------------
        current_time = self.get_clock().now()

        current_time = (
            current_time.nanoseconds / 1e9
        )

        # -----------------------------------------
        # 차량 명령 계산
        # -----------------------------------------
        angle, speed = self.logic.get_motor_command(
            current_time
        )

        # -----------------------------------------
        # Xycar Motor 메시지
        # -----------------------------------------
        motor_msg = XycarMotor()

        motor_msg.angle = angle
        motor_msg.speed = speed

        # -----------------------------------------
        # 차량 명령 발행
        # -----------------------------------------
        self.motor_publisher.publish(
            motor_msg
        )

    # =============================================
    # 종료 처리
    # =============================================
    def destroy_node(self):

        # -----------------------------------------
        # 종료 전에 차량 정지
        # -----------------------------------------
        motor_msg = XycarMotor()

        motor_msg.angle = 0.0
        motor_msg.speed = 0.0

        self.motor_publisher.publish(
            motor_msg
        )

        # -----------------------------------------
        # OpenCV 창 닫기
        # -----------------------------------------
        cv2.destroyAllWindows()

        super().destroy_node()


# =============================================
# main
# =============================================
def main(args=None):

    rclpy.init(args=args)

    node = TrafficLightDriveNode()

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

