#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#=============================================
# 본 프로그램은 자이트론에서 제작한 것입니다.
# 상업라이센스에 의해 제공되므로 무단배포 및 상업적 이용을 금합니다.
# 교육과 실습 용도로만 사용가능하며 외부유출은 금지됩니다.
#=============================================
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import XycarMotor
from sensor_msgs.msg import CompressedImage
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import cv2, os, math, time

from my_traffic.stopline_detect import StoplineDetector

#=============================================
# ROS2 Node 클래스 정의
#=============================================
class DriverNode(Node):

    #=============================================
    # 클래스 생성 초기화 함수
    #=============================================
    def __init__(self):

        super().__init__('driver')
         
        # 상수값 및 초기값 설정
        self.new_angle = 0  # 모터조향각 초기값
        self.new_speed = 0  # 모터속도 초기값
        self.image = None  # 카메라 토픽 데이터를 저장할 변수
        self.motor_msg = XycarMotor()  # 모터토픽 메시지     
        self.image_received = False   
        self.stopline_driver = StoplineDetector()  
        
        # ROS2 Publisher & Subscriber 설정
        self.motor_pub = self.create_publisher(XycarMotor,'xycar_motor',10)
        self.subscription = self.create_subscription(CompressedImage,
            '/image_raw/compressed', self.img_callback, qos_profile_sensor_data)

        # 카메라토픽이 처음 수신될 때까지 대기했다가 수신완료후 메시지 출력
        self.get_logger().info("Waiting for Camera data...")
        self.wait_for_CAM_message()
        self.get_logger().info("Camera Ready!")
        
        # 모터를 초기상태(핸들 똑바로 정지)로 설정합니다. 2초 대기합니다.
        self.drive(angle=0,speed=0)
        self.stop_car(2)
		
        self.get_logger().info("Track Driver Node Initialized")
        
    #=============================================
    # 카메라 토픽을 수신하는 Subscriber 함수
    #=============================================
    def img_callback(self, data: CompressedImage):
        try:
            np_arr = np.frombuffer(data.data, np.uint8)
            self.image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.image_received = True
        except Exception as e:
            self.get_logger().error(f"Image Decode Error: {e}")
            return 
           
    #=============================================
    # 모터제어 토픽을 발행하는 Publisher 함수
    #=============================================
    def drive(self, angle, speed):
        self.motor_msg.angle = float(angle)
        self.motor_msg.speed = float(speed)
        self.motor_pub.publish(self.motor_msg)

    #=============================================
    # 차량을 duration(1초 단위) 동안 정차시키는 함수
    #=============================================
    def stop_car(self, duration):
        for _ in range(int(duration*10)):
            self.drive(angle=0, speed=0)
            time.sleep(0.1)
            
    #=============================================
    # 카메라 노출값을 세팅하는 함수
    #=============================================
    def cam_exposure(self, value):
        os.system('v4l2-ctl -d /dev/videoCAM -c auto_exposure=1')
        os.system(f'v4l2-ctl -d /dev/videoCAM -c exposure_time_absolute={value}')
        
    #=============================================
    # 초기화 이후에 카메라토픽이 수신될 때까지 기다리는 함수
    #=============================================
    def wait_for_CAM_message(self):
        while self.image is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            
    #=============================================
    # 메인 루프
    #=============================================
    def main_loop(self):
    
        STOP_LINE = 1
        count = 0
        # 아래 while 블럭중에 처음에 어디로 들어갈지 결정합니다. 
        drive_mode = STOP_LINE
        
        # 카메라 노출값을 설정합니다.
        self.cam_exposure(120)
        		
        self.get_logger().info("======================================")
        self.get_logger().info("  S T A R T    D R I V I N G ...      ")
        self.get_logger().info("======================================")

        #===================================
        # Outer While Loop
        #===================================
        while rclpy.ok():
        		
            # ======================================
            # 전방에서 정지선을 찾고 정차합니다.
            # 곧바로 다음 순서로 넘어갑니다.  
            # ======================================
            while drive_mode == STOP_LINE:
                                  
                # 잠깐씩 쉬었다가 다음으로 넘어갑니다.  
                # 개발단계에서의 편의를 위해 삽입된 코드입니다. 
                # 코드 개발이 끝난후 꼭 필요하지 않으면 삭제하세요.             
                #time.sleep(0.03)

                # 차량을 똑바로 앞으로 전진시킵니다. 
                self.new_angle = 0        
                self.new_speed = 15
                self.drive(self.new_angle, self.new_speed)
                
                # 정지선이 있는지 체크합니다. 
                rclpy.spin_once(self, timeout_sec=0.04)  
                if self.image_received:         
                    flag = self.stopline_driver.check_stopline(self.image)
                    self.image_received = False
                        
                # 정지선을 찾았으면 다음 모드로 넘어갑니다.
                if (flag == True):
                    self.get_logger().info("#===============================#") 
                    self.get_logger().info(f"#  Stop_Line found! - {count}         #") 
                    self.get_logger().info("#===============================#") 
                    self.stop_car(2.0)  # 차량을 멈춥니다.
                    count = count+1 
     
#=============================================
# 메인 함수
#=============================================
def main(args=None):
      
    rclpy.init(args=args)
    node = DriverNode()
	
    try:
        # main_loop() 함수를 호출하여 실행합니다.
        node.main_loop()
    except KeyboardInterrupt:
        # 사용자 인터럽트 (Ctrl+C)가 발생하면 예외를 처리합니다.
        pass
    finally:
        # 노드를 종료하고 ROS2를 정리합니다.
        node.drive(angle=0, speed=0)
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

