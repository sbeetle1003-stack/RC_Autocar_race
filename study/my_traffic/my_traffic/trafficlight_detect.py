#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================
# 본 프로그램은 자이트론에서 제작한 것입니다.
# 상업라이센스에 의해 제공되므로 무단배포 및 상업적 이용을 금합니다.
# 교육과 실습 용도로만 사용가능하며 외부유출은 금지됩니다.
# =============================================

import rclpy
from rclpy.node import Node

import numpy as np
import cv2

from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge

from rclpy.qos import QoSProfile, ReliabilityPolicy


# =============================================
# 프로그램에서 사용할 색상
# =============================================
Blue =   (255, 0, 0)
Green =  (0, 255, 0)
Red =    (0, 0, 255)
Yellow = (0, 255, 255)


# =============================================
# Traffic Light 검출 Logic
# =============================================
class TrafficLightLogic:

    def __init__(self, logger):

        self.logger = logger

    # =============================================
    # 신호등 검출
    #
    # return:
    #
    #   found
    #       신호등 발견 여부
    #
    #   color
    #       'Red'
    #       'Yellow'
    #       'Blue'
    #       'Black'
    #
    #   result_image
    #       검출 결과가 그려진 영상
    # =============================================
    def check_traffic_light(self, image):

        MIN_RADIUS, MAX_RADIUS = 20, 30

        # -----------------------------------------
        # 입력 영상 확인
        # -----------------------------------------
        if image.size == 0:

            return False, 'Black', image

        # -----------------------------------------
        # 원본 이미지 복사
        # -----------------------------------------
        cimg = image.copy()

        # -----------------------------------------
        # ROI 중심
        # -----------------------------------------
        Center_X, Center_Y = 320, 100

        # -----------------------------------------
        # ROI 크기
        # -----------------------------------------
        XX, YY = 220, 80

        # -----------------------------------------
        # ROI 영역 표시
        # -----------------------------------------
        cv2.rectangle(
            cimg,
            (Center_X - XX, Center_Y - YY),
            (Center_X + XX, Center_Y + YY),
            Green,
            2
        )

        # -----------------------------------------
        # ROI 이미지 추출
        # -----------------------------------------
        roi_img = cimg[
            Center_Y - YY:Center_Y + YY,
            Center_X - XX:Center_X + XX
        ]

        # -----------------------------------------
        # Gray 변환
        # -----------------------------------------
        img = cv2.cvtColor(
            roi_img, cv2.COLOR_BGR2GRAY)

        # -----------------------------------------
        # Blur
        # -----------------------------------------
        blur = cv2.GaussianBlur(
            img, (5, 5), 0)

        # -----------------------------------------
        # Hough Circle 검출
        #
        # 기존 값 그대로 유지
        # -----------------------------------------
        circles = cv2.HoughCircles(
            blur,
            cv2.HOUGH_GRADIENT,
            1,
            20,
            param1=50,
            param2=25,
            minRadius=MIN_RADIUS,
            maxRadius=MAX_RADIUS
        )

        # =========================================
        # Circle 검출 결과가 있는 경우
        # =========================================
        if circles is not None:

            # -------------------------------------
            # 정수값으로 변환
            # -------------------------------------
            circles = np.round(
                circles[0, :]).astype("int")

            # -------------------------------------
            # Y 좌표 기준 정렬
            # -------------------------------------
            y_circles = sorted(
                circles,
                key=lambda circle: circle[1]
            )

            # -------------------------------------
            # X 좌표 기준 정렬
            #
            # 기존 코드 그대로 유지
            # -------------------------------------
            circles = sorted(
                circles,
                key=lambda circle: circle[0]
            )

            # -------------------------------------
            # 발견된 원 표시
            # -------------------------------------
            for i, (x, y, r) in enumerate(circles):

                cv2.circle(
                    cimg,
                    (x + Center_X - XX,
                     y + Center_Y - YY),
                    r,
                    Green,
                    2
                )

                # 원 번호 표시
                cv2.putText(
                    cimg,
                    str(i),
                    (x + Center_X - XX - 10,
                     y + Center_Y - YY - r - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    Green,
                    2
                )

        # =========================================
        # 정확히 3개의 원이 발견된 경우
        # =========================================
        if (circles is not None) and (len(circles) == 3):

            # -------------------------------------
            # 밝기 저장
            # -------------------------------------
            brightness_values = []

            # -------------------------------------
            # 각각의 원 처리
            # -------------------------------------
            for i, (x, y, r) in enumerate(circles):

                # ---------------------------------
                # 밝기 계산 영역
                # ---------------------------------
                roi = img[
                    y - (r // 2):y + (r // 2),
                    x - (r // 2):x + (r // 2)
                ]

                # ---------------------------------
                # 평균 밝기
                # ---------------------------------
                mean_value = round(
                    np.mean(roi), -1)

                brightness_values.append(
                    mean_value
                )

                self.logger.info(
                    f"Circle {i} at "
                    f"({x},{y}), R={r}: "
                    f"B={mean_value}"
                )

                # ---------------------------------
                # 밝기 계산 영역 표시
                # ---------------------------------
                cv2.rectangle(
                    cimg,
                    ((x - (r // 2)) + Center_X - XX,
                     (y - (r // 2)) + Center_Y - YY),
                    ((x + (r // 2)) + Center_X - XX,
                     (y + (r // 2)) + Center_Y - YY),
                    Red,
                    2
                )

            # -------------------------------------
            # 가장 밝은 원
            # -------------------------------------
            brightest_idx = np.argmax(
                brightness_values)

            brightest_circle = circles[brightest_idx]

            x, y, r = brightest_circle

            # -------------------------------------
            # 가장 밝은 원 로그
            # -------------------------------------
            self.logger.info(
                f" --- Circle {brightest_idx} "
                f"is the brightest."
            )

            # -------------------------------------
            # 가장 밝은 원을 노란색으로 표시
            # -------------------------------------
            cv2.circle(
                cimg,
                (x + Center_X - XX,
                 y + Center_Y - YY),
                r,
                Yellow,
                3
            )

            # =====================================
            # 신호등 상태 표시
            # =====================================
            colors = [
                'Red',
                'Yellow',
                'Blue'
            ]

            color = colors[brightest_idx]

            # -------------------------------------
            # 화면 상단에 판정 결과 표시
            # -------------------------------------
            cv2.putText(
                cimg,
                f'Traffic Light : {color}',
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                Yellow,
                2
            )

            cv2.putText(
                cimg,
                'Detected',
                (20, 65),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                Green,
                2
            )

            # -------------------------------------
            # 결과 영상 반환
            # -------------------------------------
            return True, color, cimg

        # =========================================
        # 신호등을 찾지 못한 경우
        # =========================================
        cv2.putText(
            cimg,
            'Traffic Light : NOT FOUND',
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            Red,
            2
        )

        return False, 'Black', cimg


# =============================================
# ROS2 Node
# =============================================
class TrafficLightDetectorNode(Node):

    def __init__(self):

        super().__init__('trafficlight_detector')

        # -----------------------------------------
        # Traffic Light Logic
        # -----------------------------------------
        self.logic = TrafficLightLogic(
            self.get_logger()
        )

        # -----------------------------------------
        # CvBridge
        # -----------------------------------------
        self.bridge = CvBridge()

        # -----------------------------------------
        # Camera QoS
        # -----------------------------------------
        qos_profile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # -----------------------------------------
        # /image_raw Subscribe
        # -----------------------------------------
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            qos_profile
        )

        # =========================================
        # /tflight_result Publisher
        #
        # [Found, Red, Yellow, Left, Blue]
        # =========================================
        self.result_publisher = self.create_publisher(
            Float32MultiArray,
            '/tflight_result',
            1
        )

        # =========================================
        # /tflight_image Publisher
        #
        # 검출 결과가 그려진 영상
        # =========================================
        self.image_publisher = self.create_publisher(
            Image,
            '/tflight_image',
            1
        )

        self.get_logger().info(
            'Traffic Light Detector started'
        )

    # =============================================
    # Camera Callback
    # =============================================
    def image_callback(self, msg):

        # -----------------------------------------
        # ROS Image -> OpenCV
        # -----------------------------------------
        try:

            image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8')

        except Exception as e:

            self.get_logger().error(
                f'CvBridge error: {e}'
            )

            return

        # -----------------------------------------
        # Traffic Light 검출
        # -----------------------------------------
        found, color, result_image = (
            self.logic.check_traffic_light(image)
        )

        # =========================================
        # /tflight_result 데이터
        #
        # [Found, Red, Yellow, Left, Blue]
        # =========================================
        result_data = [
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        ]

        # -----------------------------------------
        # 신호등 발견
        # -----------------------------------------
        if found:

            result_data[0] = 1.0

            # -------------------------------------
            # Red
            # -------------------------------------
            if color == 'Red':

                result_data[1] = 1.0

            # -------------------------------------
            # Yellow
            # -------------------------------------
            elif color == 'Yellow':

                result_data[2] = 1.0

            # -------------------------------------
            # Blue
            # -------------------------------------
            elif color == 'Blue':

                result_data[4] = 1.0

            # -------------------------------------
            # Left
            #
            # 현재 알고리즘에서는 미사용
            # -------------------------------------
            result_data[3] = 0.0

        # =========================================
        # 결과 메시지 생성
        # =========================================
        result_msg = Float32MultiArray()

        result_msg.data = result_data

        # -----------------------------------------
        # /tflight_result 발행
        # -----------------------------------------
        self.result_publisher.publish(result_msg)

        # =========================================
        # 검출 결과 영상 발행
        # =========================================
        try:

            # -------------------------------------
            # /tflight_image를 구독하는 노드가
            # 있을 때만 이미지 메시지를 생성한다.
            # -------------------------------------
            if self.image_publisher.get_subscription_count() > 0:

                image_msg = self.bridge.cv2_to_imgmsg(
                    result_image, encoding='bgr8')

                image_msg.header = msg.header

                self.image_publisher.publish(
                    image_msg
                )

        except Exception as e:

            self.get_logger().error(
                f'Image conversion error: {e}'
            )


# =============================================
# main
# =============================================
def main(args=None):

    rclpy.init(args=args)

    node = TrafficLightDetectorNode()

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

