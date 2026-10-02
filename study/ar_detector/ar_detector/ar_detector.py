#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
import cv2, os
import numpy as np
import apriltag

from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy

# =============================================
# AprilTag 검출 및 위치 계산
# =============================================
class ARDetectorLogic:

    def __init__(self):

        # AprilTag detector
        self.detector = apriltag.Detector()

        # -----------------------------------------
        # 카메라 내부 파라미터
        # -----------------------------------------
        self.camera_matrix = np.array([
            [371.42821, 0., 310.49805],
            [0., 372.60371, 235.74201],
            [0., 0., 1.]
        ])

        # -----------------------------------------
        # AprilTag 실제 크기 (cm)
        # -----------------------------------------
        self.tag_size = 9.5

    # =============================================
    # AprilTag C 라이브러리에서 출력하는 stderr 경고를 차단하면서
    # AprilTag 검출을 수행
    # =============================================
    def detect_ar_silent(self, gray):

        devnull = os.open(os.devnull, os.O_WRONLY)
        old_stderr = os.dup(2)

        try:        
            os.dup2(devnull, 2)
            detections = self.detector.detect(gray)

        finally:
            os.dup2(old_stderr, 2)
            os.close(old_stderr)
            os.close(devnull)

        return detections

    # =============================================
    # AprilTag 검출
    # =============================================
    def detect(self, image):

        # -----------------------------------------
        # BGR -> Gray
        # -----------------------------------------
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # -----------------------------------------
        # AprilTag 검출
        # -----------------------------------------
        #detections = self.detector.detect(gray_image)
        detections = self.detect_ar_silent(gray_image)
        results = []

        # =============================================
        # 검출된 Tag 처리
        # =============================================
        for detection in detections:

            # -----------------------------------------
            # Tag 중심점
            # -----------------------------------------
            center = detection.center

            # -----------------------------------------
            # Tag 좌측 세로 길이(pixel)
            # -----------------------------------------
            left_pixels = abs(detection.corners[0][1] - detection.corners[3][1])

            # -----------------------------------------
            # Tag 우측 세로 길이(pixel)
            # -----------------------------------------
            right_pixels = abs(detection.corners[1][1] - detection.corners[2][1])

            # -----------------------------------------
            # Tag 평균 크기(pixel)
            # -----------------------------------------
            tag_size_pixels = (left_pixels + right_pixels) // 2

            # -----------------------------------------
            # 잘못된 값 방지
            # -----------------------------------------
            if tag_size_pixels <= 0:
                continue

            # =========================================
            # Z 거리 계산
            # =========================================
            distance = ((self.camera_matrix[0, 0] * self.tag_size) / tag_size_pixels) * 100 / 126
            distance_to_tag_cm = (distance * 1.1494 - 14.94)

            # =========================================
            # X 위치 계산
            # 640x480 영상의 중심 X = 320
            # =========================================
            x_offset_pixels = center[0] - 320

            x_offset_cm = ((x_offset_pixels * self.tag_size) / tag_size_pixels)

            # =========================================
            # Y 위치 계산
            # 640x480 영상의 중심 Y = 240
            # 화면 위쪽 : 음수
            # 화면 아래쪽 : 양수
            # =========================================
            y_offset_pixels = center[1] - 240

            y_offset_cm = ((y_offset_pixels * self.tag_size) / tag_size_pixels)

            # =========================================
            # 결과 저장
            # =========================================
            results.append({
                'id': int(detection.tag_id),
                'x': float(x_offset_cm),
                'y': float(y_offset_cm),
                'z': float(distance_to_tag_cm),
                'center': center,
                'corners': detection.corners
            })

        # -----------------------------------------
        # Z가 가까운 Tag부터 정렬
        # -----------------------------------------
        results.sort(key=lambda item: item['z'])
        return results


# =============================================
# ROS2 Node
# =============================================
class ARDetector(Node):

    def __init__(self):

        super().__init__('ar_detector')

        # -----------------------------------------
        # AprilTag 검출 및 위치 계산 객체
        # -----------------------------------------
        self.logic = ARDetectorLogic()

        # -----------------------------------------
        # CvBridge
        # -----------------------------------------
        self.bridge = CvBridge()

        # -----------------------------------------
        # 카메라 이미지 구독
        # -----------------------------------------
        #self.subscription = self.create_subscription(
        #    Image, '/image_raw', self.image_callback, qos_profile_sensor_data)

        qos_profile = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.subscription = self.create_subscription(
            Image, '/image_raw', self.image_callback, qos_profile)

        # -----------------------------------------
        # AprilTag 위치 결과 발행
        # [ID, X, Y, Z, ID, X, Y, Z, ...]
        # -----------------------------------------
        self.publisher = self.create_publisher(
            Float32MultiArray, '/ardetect_result', 1)

        self.get_logger().info('AR Detector started')

    # =============================================
    # Camera callback
    # =============================================
    def image_callback(self, msg):

        # -----------------------------------------
        # ROS Image -> OpenCV image
        # -----------------------------------------
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        except Exception as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        # -----------------------------------------
        # AprilTag 검출
        # -----------------------------------------
        results = self.logic.detect(image)

        # -----------------------------------------
        # 결과 데이터 생성
        # [ID, X, Y, Z, ...]
        # -----------------------------------------
        result_data = []

        for result in results:

            result_data.append(float(result['id']))
            result_data.append(result['x'])
            result_data.append(result['y'])
            result_data.append(result['z'])

        # -----------------------------------------
        # 결과 메시지 생성
        # -----------------------------------------
        result_msg = Float32MultiArray()
        result_msg.data = result_data

        # -----------------------------------------
        # 결과 발행
        # Tag가 없으면: []
        # -----------------------------------------
        self.publisher.publish(result_msg)


# =============================================
# main
# =============================================
def main(args=None):

    rclpy.init(args=args)
    node = ARDetector()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':

    main()
