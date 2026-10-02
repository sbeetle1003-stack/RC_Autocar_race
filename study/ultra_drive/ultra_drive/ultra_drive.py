#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray


# =============================================
# 초음파 조향 계산 클래스
# =============================================
class UltraDriverLogic:

    def calc_angle(self, ultra_msg):

        left_cm = float(ultra_msg[1])
        right_cm = float(ultra_msg[3])

        if left_cm > right_cm:
            angle = -50
        elif left_cm < right_cm:
            angle = 50
        else:
            angle = 0
            
        return angle


# =============================================
# ROS2 초음파 주행 노드
# =============================================
class UltraDriveNode(Node):

    def __init__(self):

        super().__init__('ultra_drive')

        # QoS 설정
        # 초음파 센서는 최신 데이터가 중요하므로 Best Effort 사용
        qos_profile = QoSProfile(
            depth=3, reliability=ReliabilityPolicy.BEST_EFFORT)

        # 초음파 데이터 구독
        self.subscription = self.create_subscription(
            Int32MultiArray, 'xycar_ultrasonic', self.ultra_callback, qos_profile)

        # /ultradrive_result publisher
        self.publisher = self.create_publisher(
            Float32MultiArray, 'ultradrive_result', 1)

        # 초음파 조향 계산 객체
        self.ultra_driver = UltraDriverLogic()

        self.get_logger().info('----- Ultra drive node started -----')

    # =============================================
    # 초음파 콜백
    # =============================================
    def ultra_callback(self, msg):

        # 초음파 거리
        left_cm = float(msg.data[1])
        right_cm = float(msg.data[3])

        # 조향각 계산
        angle = self.ultra_driver.calc_angle(msg.data)

        # 결과 메시지
        result_msg = Float32MultiArray()

        result_msg.data = [float(angle), left_cm, right_cm]

        # /ultradrive_result 발행
        self.publisher.publish(result_msg)


def main(args=None):

    rclpy.init(args=args)

    node = UltraDriveNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

