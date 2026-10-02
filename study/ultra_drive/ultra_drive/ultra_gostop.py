#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import XycarMotor


class UltraDriverNode(Node):

    def __init__(self):
        super().__init__('ultra_driver')

        # 모터 토픽 발행
        self.motor_publisher = self.create_publisher(
            XycarMotor, 'xycar_motor', 1)

        # QoS 설정
        # 초음파 센서는 최신 데이터가 중요하므로 Best Effort 사용
        qos_profile = QoSProfile(
            depth=3, reliability=ReliabilityPolicy.BEST_EFFORT)

        # 초음파 데이터 구독
        self.subscription = self.create_subscription(
            Int32MultiArray, 'xycar_ultrasonic', self.ultra_callback, qos_profile)

        # 모터 메시지
        self.motor_msg = XycarMotor()

        # 초음파 데이터 저장
        self.ultra_msg = None

        # 차량 초기 상태
        self.drive(angle=0, speed=0)

        # 0.1초마다 차량 제어
        self.timer = self.create_timer(
            0.1, self.control_loop)

    def ultra_callback(self, msg):

        # 최신 초음파 데이터 저장
        self.ultra_msg = msg.data

    def drive(self, angle, speed):

        # 모터 메시지 설정
        self.motor_msg.angle = float(angle)
        self.motor_msg.speed = float(speed)

        # 모터 명령 발행
        self.motor_publisher.publish(self.motor_msg)

    def control_loop(self):

        # 초음파 데이터가 아직 없으면 대기
        if self.ultra_msg is None:
            return

        # 가까운 장애물 감지
        if 0 < self.ultra_msg[2] < 20:

            # 정지
            self.drive(angle=0, speed=0)

        else:

            # 전진
            self.drive(angle=0, speed=12)


def main(args=None):

    # ROS2 초기화
    rclpy.init(args=args)

    # 노드 생성
    node = UltraDriverNode()

    try:
        # ROS2 이벤트 처리
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        # 차량 정지
        node.drive(angle=0, speed=0)

        # 노드 종료
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
