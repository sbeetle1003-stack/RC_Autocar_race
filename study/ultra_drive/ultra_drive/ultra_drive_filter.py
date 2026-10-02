#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray


# =============================================
# 이동평균필터 클래스
# =============================================
class MovingAverage:

    def __init__(self, n):
        self.samples = n
        self.data = []
        self.weights = list(range(1, n + 1))

    # 새로운 샘플 추가
    def add_sample(self, new_sample):
        if len(self.data) < self.samples:
            self.data.append(new_sample)
        else:
            self.data.pop(0)
            self.data.append(new_sample)

    # 저장된 샘플 개수
    def get_sample_count(self):
        return len(self.data)

    # 단순 이동평균
    def get_mavg(self):
        if not self.data:
            return 0.0

        return float(sum(self.data)) / len(self.data)

    # 중앙값
    def get_mmed(self):
        if not self.data:
            return 0.0

        return float(np.median(self.data))

    # 가중 이동평균
    def get_wmavg(self):
        if not self.data:
            return 0.0

        s = sum(x * w for x, w in zip(self.data, self.weights[:len(self.data)]))

        return float(s) / sum(self.weights[:len(self.data)])

    # 최소값
    def get_min(self):
        if not self.data:
            return 0.0

        return float(min(self.data))

    # 최대값
    def get_max(self):
        if not self.data:
            return 0.0

        return float(max(self.data))


# =============================================
# 초음파 조향 계산 클래스
# =============================================
class UltraDriverLogic:

    def calc_angle(self, left_cm, right_cm):

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

        # 이동평균필터 (n = 3)
        self.left_filter = MovingAverage(3)
        self.right_filter = MovingAverage(3)

        self.get_logger().info('----- Ultra drive node started -----')

    # =============================================
    # 초음파 콜백
    # =============================================
    def ultra_callback(self, msg):

        # -----------------------------------------
        # 원본 초음파 거리
        # -----------------------------------------
        left_raw = float(msg.data[1])
        right_raw = float(msg.data[3])

        # -----------------------------------------
        # 이동평균필터에 샘플 추가
        # -----------------------------------------
        self.left_filter.add_sample(left_raw)
        self.right_filter.add_sample(right_raw)

        # -----------------------------------------
        # 이동평균 계산
        # -----------------------------------------
        left_cm = self.left_filter.get_mavg()
        right_cm = self.right_filter.get_mavg()

        # -----------------------------------------
        # 조향각 계산
        # -----------------------------------------
        angle = self.ultra_driver.calc_angle(left_cm, right_cm)

        # -----------------------------------------
        # 결과 메시지
        # [조향각, 필터링된 왼쪽 거리, 필터링된 오른쪽 거리]
        # -----------------------------------------
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

