#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# =============================================
# ROS2 Humble Lane Detect Node
# =============================================
import numpy as np
import cv2
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge

# =============================================
# 색상
# =============================================
Blue = (255, 0, 0)
Green = (0, 255, 0)
Red = (0, 0, 255)
Yellow = (0, 255, 255)

# =============================================
# 차선 인식 프로그램에서 사용할 상수
# =============================================
CAM_FPS = 30
WIDTH, HEIGHT = 640, 480
ROI_START_ROW = 250
ROI_END_ROW = 450
ROI_HEIGHT = ROI_END_ROW - ROI_START_ROW
L_ROW = 110
View_Center = WIDTH // 2

SHOW_IMAGE = True

# =============================================
# 이전 차선 위치
# =============================================
prev_x_left, prev_x_right = 100, 540

# =============================================
# 카메라 영상에서 차선을 찾는 함수
# =============================================
def lane_detect(image):

    global prev_x_left, prev_x_right

    display_img = image.copy()

    # =========================================
    # 원본 이미지 전처리
    # =========================================

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur_gray = cv2.GaussianBlur(gray, (5, 5), 0)
    edge_img = cv2.Canny(np.uint8(blur_gray), 60, 75)

    # =========================================
    # ROI 영역 추출
    # =========================================

    roi_img = image[ROI_START_ROW:ROI_END_ROW, 0:WIDTH]
    line_draw_img = roi_img.copy()

    # =========================================
    # ROI 이미지 전처리
    # =========================================

    gray = cv2.cvtColor(roi_img, cv2.COLOR_BGR2GRAY)
    blur_gray = cv2.GaussianBlur(gray, (5, 5), 0)
    edge_img = cv2.Canny(np.uint8(blur_gray), 60, 75)

    # =========================================
    # HoughLinesP 선분 검출
    # =========================================

    all_lines = cv2.HoughLinesP(edge_img, 1, math.pi / 180, 50, 50, 20)

    if all_lines is None:
        return False, 0, 0

    # =========================================
    # 선분 기울기 필터링
    # =========================================

    slopes = []
    filtered_lines = []

    for line in all_lines:

        x1, y1, x2, y2 = line[0]

        if x2 == x1:
            slope = 1000.0
        else:
            slope = float(y2 - y1) / float(x2 - x1)

        if 0.2 < abs(slope):
            slopes.append(slope)
            filtered_lines.append(line[0])

    if len(filtered_lines) == 0:
        return False, 0, 0

    # =========================================
    # 왼쪽 / 오른쪽 차선 선분 분류
    # =========================================

    left_lines = []
    right_lines = []

    Margin = 0

    for j in range(len(slopes)):

        Line = filtered_lines[j]
        slope = slopes[j]
        x1, y1, x2, y2 = Line

        if slope < 0 and x2 < WIDTH / 2 - Margin:
            left_lines.append(Line.tolist())

        elif slope > 0 and x1 > WIDTH / 2 + Margin:
            right_lines.append(Line.tolist())

    # =========================================
    # 왼쪽 대표직선 계산
    # y = m_left * x + b_left
    # =========================================

    m_left, b_left = 0.0, 0.0
    x_sum, y_sum, m_sum = 0.0, 0.0, 0.0

    size = len(left_lines)

    if size != 0:

        for line in left_lines:

            x1, y1, x2, y2 = line
            x_sum += x1 + x2
            y_sum += y1 + y2

            if x2 != x1:
                m_sum += float(y2 - y1) / float(x2 - x1)
            else:
                m_sum += 0

        x_avg = x_sum / (size * 2)
        y_avg = y_sum / (size * 2)
        m_left = m_sum / size
        b_left = y_avg - m_left * x_avg

        if m_left != 0.0 and SHOW_IMAGE == True :

            x1 = int((0.0 - b_left) / m_left)
            x2 = int((ROI_HEIGHT - b_left) / m_left)
            cv2.line(line_draw_img, (x1, 0), (x2, ROI_HEIGHT), Blue, 2)

    # =========================================
    # 오른쪽 대표직선 계산
    # y = m_right * x + b_right
    # =========================================

    m_right, b_right = 0.0, 0.0
    x_sum, y_sum, m_sum = 0.0, 0.0, 0.0

    size = len(right_lines)

    if size != 0:

        for line in right_lines:

            x1, y1, x2, y2 = line
            x_sum += x1 + x2
            y_sum += y1 + y2

            if x2 != x1:
                m_sum += float(y2 - y1) / float(x2 - x1)
            else:
                m_sum += 0

        x_avg = x_sum / (size * 2)
        y_avg = y_sum / (size * 2)
        m_right = m_sum / size
        b_right = y_avg - m_right * x_avg

        if m_right != 0.0 and SHOW_IMAGE == True:

            x1 = int((0.0 - b_right) / m_right)
            x2 = int((ROI_HEIGHT - b_right) / m_right)
            cv2.line(line_draw_img, (x1, 0), (x2, ROI_HEIGHT), Blue, 2)

    # =========================================
    # 기준 수평선과 대표직선의 교점 계산
    # =========================================

    if m_left == 0.0:
        x_left = prev_x_left
    else:
        x_left = int((L_ROW - b_left) / m_left)

    if m_right == 0.0:
        x_right = prev_x_right
    else:
        x_right = int((L_ROW - b_right) / m_right)

    # =========================================
    # 한쪽 차선만 검출된 경우 반대쪽 차선 추정
    # =========================================

    if m_left == 0.0 and m_right != 0.0:
        x_left = x_right - 380

    if m_left != 0.0 and m_right == 0.0:
        x_right = x_left + 380

    # =========================================
    # 이전 차선 위치 업데이트
    # =========================================

    prev_x_left = x_left
    prev_x_right = x_right

    # =========================================
    # 차선 중앙
    # =========================================

    x_midpoint = (x_left + x_right) // 2

    # =========================================
    # 차선 위치 표시
    # =========================================

    if SHOW_IMAGE == True:
        cv2.line(line_draw_img, (0, L_ROW), (WIDTH, L_ROW), Yellow, 2)
        cv2.rectangle(line_draw_img, (x_left - 5, L_ROW - 5), (x_left + 5, L_ROW + 5), Green, 4)
        cv2.rectangle(line_draw_img, (x_right - 5, L_ROW - 5), (x_right + 5, L_ROW + 5), Green, 4)
        cv2.rectangle(line_draw_img, (x_midpoint - 5, L_ROW - 5), (x_midpoint + 5, L_ROW + 5), Blue, 4)
        cv2.rectangle(line_draw_img, (View_Center - 5, L_ROW - 5), (View_Center + 5, L_ROW + 5), Red, 4)

        # =========================================
        # ROI 영상을 원본 영상에 삽입
        # =========================================

        display_img[ROI_START_ROW:ROI_END_ROW, 0:WIDTH] = line_draw_img

        cv2.imshow("Lanes positions", display_img)
        cv2.waitKey(1)

    # =========================================
    # 기존 함수의 반환값 유지
    # =========================================

    return True, x_left, x_right


# =============================================
# ROS2 Lane Detect Node
# =============================================
class LaneDetectNode(Node):

    def __init__(self):

        super().__init__('lane_detect')

        # =====================================
        # CvBridge
        # =====================================
        self.bridge = CvBridge()

        # =====================================
        # 카메라 이미지 구독
        # =====================================
        self.subscription = self.create_subscription(Image, '/image_raw', self.image_callback, 10)

        # =====================================
        # 차선 검출 결과 발행
        # =====================================
        self.publisher = self.create_publisher(Float32MultiArray, '/lanedetect_result', 10)

        self.get_logger().info('Lane Detect Node Started')

    # =========================================
    # 카메라 이미지 Callback
    # =========================================
    def image_callback(self, msg):

        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

        # =====================================
        # 차선 검출
        # =====================================
        found, x_left, x_right = lane_detect(image)

        # =====================================
        # 차선 검출 성공
        # =====================================
        if found:

            x_midpoint = (x_left + x_right) // 2
            error = x_midpoint - View_Center

        # =====================================
        # 차선 검출 실패
        # =====================================
        else:

            x_left = 0
            x_right = 0
            x_midpoint = 0
            error = 0

        # =====================================
        # ROS2 메시지
        #
        # [found, x_left, x_right, x_midpoint, error]
        #
        # found = 1.0 : 검출 성공
        # found = 0.0 : 검출 실패
        # =====================================
        result_msg = Float32MultiArray()
        result_msg.data = [float(found), float(x_left), float(x_right), float(x_midpoint), float(error)]

        self.publisher.publish(result_msg)

        # =====================================
        # 터미널 출력
        # =====================================
        if found:
            self.get_logger().info(f'Lane Detect: L={x_left}, R={x_right}, Mid={x_midpoint}, Error={error}')
        else:
            self.get_logger().warning('Lane Detect Failed')


# =============================================
# main
# =============================================

def main(args=None):

    rclpy.init(args=args)
    node = LaneDetectNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


# =============================================
# 프로그램 시작
# =============================================
if __name__ == '__main__':
    main()

