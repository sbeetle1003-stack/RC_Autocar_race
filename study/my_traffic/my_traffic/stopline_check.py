#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import cv2, os, time

Blue =  (255,0,0)
Green = (0,255,0)
Red =   (0,0,255)
Yellow = (0,255,255)

class StoplineDetectorNode(Node):
    def __init__(self):
        super().__init__('stopline_detector')
        
        # 변수 초기화
        self.image = np.empty(shape=[0])
        self.stopline_num = 1
        self.image_received = False
        
        # 카메라토픽을 구독하여 이미지 수신
        self.subscription = self.create_subscription(CompressedImage,
            '/image_raw/compressed', self.img_callback, qos_profile_sensor_data)

        # 카메라토픽이 처음 수신될 때까지 대기했다가 수신완료후 메시지 출력
        self.get_logger().info("Waiting for Camera data...")
        self.wait_for_CAM_message()
        self.get_logger().info("Camera Ready!")
         
        # 카메라의 노출값 설정
        self.cam_exposure(120)
                
    def img_callback(self, data: CompressedImage):
        try:
            np_arr = np.frombuffer(data.data, np.uint8)
            self.image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.image_received = True
        except Exception as e:
            self.get_logger().error(f"Image Decode Error: {e}")
            return 
        
    def wait_for_CAM_message(self):
        # 초기화 이후에  카메라 토픽이 수신될 때까지 기다리는 함수
        while self.image is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            
    def cam_exposure(self, value):
        os.system('v4l2-ctl -d /dev/videoCAM -c auto_exposure=1')
        os.system(f'v4l2-ctl -d /dev/videoCAM -c exposure_time_absolute={value}')
                        
    #=============================================
    # 카메라 이미지에서 정지선을 찾는 함수
    #=============================================
    def check_stopline(self, image):
        
        if image.size == 0:
            return False

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

    def main_loop(self):
        self.get_logger().info(f"Stopline Checking starts...")
        count = 1
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.image_received:
                if self.check_stopline(self.image) == True:
                    self.get_logger().info(f"Found stopline - {count}")
                    count = count + 1
                    self.image_received = False            
     
def main(args=None):
    # ROS2를 초기화합니다.
    rclpy.init(args=args)
    
    # StoplineDetectorNode 인스턴스를 생성합니다.
    node = StoplineDetectorNode()
    
    try:
        # while 루프를 돕니다.
        node.main_loop()
    except KeyboardInterrupt:
        # 사용자 인터럽트 (Ctrl+C)가 발생하면 예외 처리
        pass
    finally:
        # 노드를 종료하고 ROS2를 정리합니다.
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

# 스크립트가 직접 실행될 때 main() 함수를 호출합니다.
if __name__ == '__main__':
    main()

