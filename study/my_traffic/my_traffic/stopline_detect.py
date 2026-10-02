#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#=============================================
# 본 프로그램은 자이트론에서 제작한 것입니다.
# 상업라이센스에 의해 제공되므로 무단배포 및 상업적 이용을 금합니다.
# 교육과 실습 용도로만 사용가능하며 외부유출은 금지됩니다.
#=============================================
import rclpy
from rclpy.node import Node
import numpy as np
import cv2, os, math, time

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
Blue =  (255,0,0) # 파란색
Green = (0,255,0) # 녹색
Red =   (0,0,255) # 빨간색
Yellow = (0,255,255) # 노란색

#=============================================
# 신호등의 파란불을 체크해서 True/False 값을 반환
#=============================================
class StoplineDetector:
    def __init__(self):
        pass
                
    def check_stopline(self, image):
        # 정지선이 있는지 체크하고 True/False를 반환

        #if image.size == 0:
        #    return False

        # 원본이미지를 복제한 후에 특정영역(ROI Area)을 잘라내기
        cimg = image.copy()
        roi_img = cimg[300:480, 0:640]

        # HSV 포맷으로 변환하고 특정 범위를 정해서 흑백 이진화 이미지로 변환
        hsv_image = cv2.cvtColor(roi_img, cv2.COLOR_BGR2HSV)
        upper_white = np.array([255, 255, 255])
        lower_white = np.array([0, 0, 180])
        binary_img = cv2.inRange(hsv_image, lower_white, upper_white)

        # 흑백이진화 이미지에서 특정영역을 잘라내서 정지선 체크용 이미지로 만들기
        stopline_check_img = binary_img[100:120, 200:440]
		
        # 흑백이진화 이미지를 칼라이미지로 바꾸고 정지선 체크용 이미지 영역을 녹색사각형으로 표시
        img = cv2.cvtColor(binary_img, cv2.COLOR_GRAY2BGR)
        cv2.rectangle(img, (200, 100), (440, 120), Green, 3)
        
        # 정지선 체크용 이미지에서 흰색 점의 개수 카운트하기
        stopline_count = cv2.countNonZero(stopline_check_img)

        # cv2.putText(img, text, org, fontFace, fontScale, color, thickness, lineType)
        cv2.putText(img, str(stopline_count),(20,40),cv2.FONT_HERSHEY_SIMPLEX,1,Yellow,2,cv2.LINE_AA)
        
        # 원래 이미지와 흑백이진화 이미지를 위아래로 붙여서 하나의 이미지로 만들어 표시
        combined_img = cv2.vconcat([cimg, img])
        cv2.imshow('Stopline Check', combined_img)
        cv2.waitKey(1)
        
        # 사각형 안의 흰색 점이 기준치 이상이면 정지선을 발견한 것으로 한다
        if stopline_count > 2500:
            return True
        else:
            return False
        
