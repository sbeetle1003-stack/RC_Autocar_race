#!/usr/bin/env python
# -*- coding: utf-8 -*- 15
#=============================================
# 본 프로그램은 자이트론에서 제작한 것입니다.
# 상업라이센스에 의해 제공되므로 무단배포 및 상업적 이용을 금합니다.
# 교육과 실습 용도로만 사용가능하며 외부유출은 금지됩니다.
#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import numpy as np
import cv2, time, math, os

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
WIDTH, HEIGHT = 640, 480  # 카메라 이미지 가로x세로 크기
Blue =  (255,0,0) # 파란색
Green = (0,255,0) # 녹색
Red =   (0,0,255) # 빨간색
Yellow = (0,255,255) # 노란색
View_Center = WIDTH//2  # 화면의 중앙값 = 카메라 위치

# 허용 범위 설정 (예: 50픽셀)
TOLERANCE = 50
prev_x_left, prev_x_right = None, None
#=============================================
# 차선인식 프로그램에서 사용할 상수 선언부
#=============================================
ROI_START_ROW = 280  # 차선을 찾을 ROI 영역의 시작 Row값
ROI_END_ROW = 400  # 차선을 찾을 ROT 영역의 끝 Row값
ROI_HEIGHT = ROI_END_ROW - ROI_START_ROW  # ROI 영역의 세로 크기  
L_ROW = 50  # 차선의 위치를 찾기 위한 ROI 안에서의 기준 Row값 


#=========================================
#  사다리꼴 모양으로 이미지 자르기
#=========================================
def apply_trapezoid_mask(img):
    mask = np.zeros_like(img)
    height, width = img.shape[:2]

    # 사다리꼴 모양의 좌표 정의
    trapezoid = np.array([

        [width * 2//6, height * 1//6],
        [width * 4//6, height * 1//6],
        [width,        height * 5//6],
        [0,            height * 5//6]

    ], np.int32)

    # 사다리꼴 모양으로 마스크 이미지 채우기
    cv2.fillPoly(mask, [trapezoid], 255)

    cv2.imshow("Mask", mask)
    
    # 마스크를 사용하여 이미지 자르기
    masked_img = cv2.bitwise_and(img, mask)
    return masked_img
    
#=============================================
# 카메라 이미지에서 차선을 찾아 그 위치를 반환하는 함수
#=============================================
def lane_detect(image, lane_row):

    # ---------------------------------------------
    # 1. 그레이스케일 변환 및 가우시안 블러
    # ---------------------------------------------
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # ---------------------------------------------
    # 2. Canny 엣지 감지
    # ---------------------------------------------
    edges = cv2.Canny(blurred, 50, 150)

    #cv2.imshow("Edge", edges)
    #cv2.waitKey(1)

    # ---------------------------------------------
    # 3. 관심영역(ROI) 설정
    # ---------------------------------------------
    height, width = edges.shape

    roi = np.array([[(0, height), (0, height - 100),
                     (width // 2 - 50, lane_row - 80),
                     (width // 2 + 50, lane_row - 80),
                     (width, height - 100), (width, height)]], dtype=np.int32)

    mask = np.zeros_like(edges)
    cv2.fillPoly(mask, roi, 255)
    masked_edges = cv2.bitwise_and(edges, mask)

    cv2.imshow("Masked", masked_edges)
    cv2.waitKey(1)

    # ---------------------------------------------
    # 4. 허프 변환을 이용한 선분 검출
    # ---------------------------------------------
    lines = cv2.HoughLinesP(masked_edges, rho=1, theta=np.pi / 180, threshold=50, minLineLength=50, maxLineGap=50)

    # ---------------------------------------------
    # 5. 좌우 차선 선분 저장 공간
    # ---------------------------------------------
    x_left, x_right = None, None
    left_lines, right_lines = [], []

    # ---------------------------------------------
    # 6. Hough 선분을 좌우 차선으로 분류
    # ---------------------------------------------
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            slope = (y2 - y1) / (x2 - x1) if x2 != x1 else float('inf')

            if slope < -0.2:
                left_lines.append((x1, y1, x2, y2))

            elif slope > 0.2:
                right_lines.append((x1, y1, x2, y2))

    # ---------------------------------------------
    # 7. 여러 Hough 선분을 하나의 직선으로 근사
    # ---------------------------------------------
    def extrapolate(lines, y_target):
        if not lines:
            return None, None

        x_coords, y_coords = [], []

        for x1, y1, x2, y2 in lines:
            x_coords += [x1, x2]
            y_coords += [y1, y2]

        poly = np.polyfit(y_coords, x_coords, 1)
        x_target = int(np.polyval(poly, y_target))

        return x_target, poly

    # ---------------------------------------------
    # 8. 좌우 차선을 각각 하나의 직선으로 근사
    # ---------------------------------------------
    x_left, left_poly = extrapolate(left_lines, lane_row)
    x_right, right_poly = extrapolate(right_lines, lane_row)

    # ---------------------------------------------
    # 9. 원본 이미지 복사
    # ---------------------------------------------
    line_image = image.copy()

    # ---------------------------------------------
    # 10. 왼쪽 Hough 선분 표시 - 빨간색
    # ---------------------------------------------
    for x1, y1, x2, y2 in left_lines:
        cv2.line(line_image, (x1, y1), (x2, y2), Red, 2)

    # ---------------------------------------------
    # 11. 오른쪽 Hough 선분 표시 - 노란색
    # ---------------------------------------------
    for x1, y1, x2, y2 in right_lines:
        cv2.line(line_image, (x1, y1), (x2, y2), Yellow, 2)

    # ---------------------------------------------
    # 12. 1차원 직선 근사 결과 표시 - 파란색
    # ---------------------------------------------
    y_bottom = height

    # 왼쪽 근사 직선
    if left_poly is not None:
        x1 = int(np.polyval(left_poly, lane_row-80))
        x2 = int(np.polyval(left_poly, y_bottom))
        cv2.line(line_image, (x1, lane_row-80), (x2, y_bottom), Blue, 3)

    # 오른쪽 근사 직선
    if right_poly is not None:
        x1 = int(np.polyval(right_poly, lane_row-80))
        x2 = int(np.polyval(right_poly, y_bottom))
        cv2.line(line_image, (x1, lane_row-80), (x2, y_bottom), Blue, 3)

    # ---------------------------------------------
    # 13. Hough 선분 + 근사 직선 화면 출력
    # ---------------------------------------------
    #cv2.imshow("Hough Lines", line_image)
    #cv2.waitKey(1)

    # ---------------------------------------------
    # 14. 디버깅용 Lane Detection 화면
    # ---------------------------------------------
    debug_image = line_image.copy()

    # 왼쪽 차선 기준점
    if x_left is not None:
        #cv2.line(debug_image, (x_left, lane_row), (x_left, height), Blue, 2)
        cv2.rectangle(debug_image, (x_left - 5, lane_row - 5), (x_left + 5, lane_row + 5), Green, 4)

    # 오른쪽 차선 기준점
    if x_right is not None:
        #cv2.line(debug_image, (x_right, lane_row), (x_right, height), Blue, 2)
        cv2.rectangle(debug_image, (x_right - 5, lane_row - 5), (x_right + 5, lane_row + 5), Green, 4)
    
    # ---------------------------------------------
    # 15. 기준 수평선 표시
    # ---------------------------------------------
    cv2.line(debug_image, (0, lane_row), (width, lane_row), Yellow, 2)

    x_midpoint = (x_left + x_right) // 2 
    cv2.rectangle(debug_image, (x_midpoint-5,lane_row-5), (x_midpoint+5,lane_row+5), Blue, 4)
    cv2.rectangle(debug_image, (View_Center-5,lane_row-5), (View_Center+5,lane_row+5), Red, 4)
 
    # ---------------------------------------------
    # 16. Lane Detection 화면 출력
    # ---------------------------------------------
    cv2.imshow("Lane Detection", debug_image)
    cv2.waitKey(1)

    # ---------------------------------------------
    # 17. 차선 검출 성공 여부
    # ---------------------------------------------
    found = x_left is not None and x_right is not None

    return found, x_left if x_left is not None else -1, x_right if x_right is not None else -1




#=============================================
# 실질적인 메인 함수 
#=============================================
def start():

    cap = cv2.VideoCapture("xycar_track1.mp4")
    #cap = cv2.VideoCapture("road_video1.mp4")
    #cap = cv2.VideoCapture("road_video2.mp4")

    while True:
        ret, image = cap.read()
        if ret:
            found, x_left, x_right = lane_detect(image, 360)
            
            #cv2.imshow('CAM image', image)
            cv2.waitKey(1)

        if not ret:
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

#=============================================
# 메인함수를 호출합니다.
# start() 함수가 실질적인 메인함수입니다.
#=============================================
if __name__ == '__main__':
    start()
