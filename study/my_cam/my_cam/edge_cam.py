#!/usr/bin/env python3

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraNode(Node):

    def __init__(self):
        super().__init__('camera')

        # ROS Image 메시지를 OpenCV 이미지로 변환하기 위한 객체
        self.bridge = CvBridge()

        # 최신 카메라 영상을 저장
        self.cv_image = None

        # '/image_raw' 토픽 구독
        #self.subscription = self.create_subscription(
        #    Image, '/image_raw', self.image_callback, qos_profile_sensor_data)

        qos_profile = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.subscription = self.create_subscription(
        Image, '/image_raw', self.image_callback, qos_profile)

        # 영상 처리 Timer
        # 0.1초마다 process_image() 실행 → 약 10 FPS
        self.timer = self.create_timer(0.1, self.process_image)

        self.get_logger().info('----- Camera node started -----')

    def image_callback(self, msg):
        # 카메라 영상 수신
        # Callback에서는 영상만 저장한다.
        self.cv_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='bgr8')

    def process_image(self):
        # 카메라 영상이 아직 수신되지 않았으면 종료
        if self.cv_image is None:
            return

        # 최신 영상 복사
        image = self.cv_image.copy()

        # ----------------------------------------------------
        # 1. Grayscale 변환
        # ----------------------------------------------------
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # ----------------------------------------------------
        # 2. Gaussian Blur
        # ----------------------------------------------------
        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        # ----------------------------------------------------
        # 3. Canny Edge 검출
        # ----------------------------------------------------
        edge = cv2.Canny(blur, 60, 70)

        # ----------------------------------------------------
        # 영상 출력
        # ----------------------------------------------------
        cv2.imshow('Original', image)
        cv2.imshow('Grayscale', gray)
        cv2.imshow('Gaussian Blur', blur)
        cv2.imshow('Canny Edge', edge)

        # 키보드 입력 확인
        key = cv2.waitKey(1) & 0xFF

        # q 키를 누르면 프로그램 종료
        if key == ord('q'):
            rclpy.shutdown()


def main(args=None):

    # ROS2 초기화
    rclpy.init(args=args)

    # Camera 노드 생성
    node = CameraNode()

    try:
        # 메시지 수신 및 Timer 실행
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        # OpenCV 창 닫기
        cv2.destroyAllWindows()

        # 노드 종료
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

