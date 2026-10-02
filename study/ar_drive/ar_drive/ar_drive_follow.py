#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

# =============================================
# AR Drive Node
# =============================================
class ARDriveNode(Node):

    def __init__(self):

        super().__init__('ar_drive')

        # =============================================
        # /ardetect_result Subscriber
        # [ID, X, Y, Z, ID, X, Y, Z, ...]
        # =============================================
        self.subscription = self.create_subscription(
            Float32MultiArray, 'ardetect_result',
            self.ar_callback, 1)

        # =============================================
        # /ardrive_result Publisher
        # [Angle, ID, X, Y, Z]
        # =============================================
        self.publisher = self.create_publisher(
            Float32MultiArray, 'ardrive_result', 1)

        self.get_logger().info('----- AR Drive node started -----')

    # =============================================
    # AR Tag 결과 Callback
    # =============================================
    def ar_callback(self, msg):

        # =============================================
        # AR Tag가 없는 경우
        # =============================================
        if len(msg.data) < 4:
            return

        # =============================================
        # 첫 번째 AR Tag 정보
        # [ID, X, Y, Z]
        # =============================================
        tag_id = msg.data[0]
        x_pos = msg.data[1]
        y_pos = msg.data[2]
        z_pos = msg.data[3]

        # =============================================
        # AR Tag까지의 거리 계산
        # X, Z는 cm 단위
        # =============================================
        distance = math.sqrt(x_pos ** 2 + z_pos ** 2)

        # =============================================
        # 거리별 조향 비율 결정
        # =============================================
        if distance > 100.0:
            new_angle = (x_pos + 0.0) * 1.0

        elif distance > 50.0:
            new_angle = (x_pos + 20.0) * 2.0

        else:
            new_angle = (x_pos + 30.0) * 3.0

        # =============================================
        # ardrive_result 메시지
        # [조향각, ID, X, Y, Z]
        # =============================================
        result_msg = Float32MultiArray()

        result_msg.data = [float(new_angle), float(tag_id),
            float(x_pos), float(y_pos), float(z_pos)]

        # =============================================
        # 결과 발행
        # =============================================
        self.publisher.publish(result_msg)

        # =============================================
        # 상태 출력
        # =============================================
        #self.get_logger().info(
        #    f'ID={int(tag_id)} '
        #    f'X={x_pos:.1f} cm '
        #    f'Y={y_pos:.1f} cm '
        #    f'Z={z_pos:.1f} cm '
        #    f'Distance={distance:.1f} cm '
        #    f'Angle={new_angle:.1f}'
        #)


# =============================================
# Main
# =============================================
def main(args=None):

    rclpy.init(args=args)
    node = ARDriveNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
