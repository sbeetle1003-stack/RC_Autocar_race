#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray


# =============================================
# Traffic Light Result Viewer
# =============================================
class MyTrafficLight(Node):

    def __init__(self):

        super().__init__('my_tflight')

        # -----------------------------------------
        # /tflight_result Subscribe
        #
        # [Found, Red, Yellow, Left, Blue]
        # -----------------------------------------
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/tflight_result',
            self.tflight_callback,
            1
        )

        self.get_logger().info(
            'Traffic Light Viewer started'
        )

    # =============================================
    # Traffic Light Result Callback
    # =============================================
    def tflight_callback(self, msg):

        data = msg.data

        # -----------------------------------------
        # 데이터 길이 확인
        # -----------------------------------------
        if len(data) != 5:

            self.get_logger().warning(
                f'Invalid tflight_result data: '
                f'{len(data)} values'
            )

            return

        # -----------------------------------------
        # 결과 데이터
        # -----------------------------------------
        found = int(data[0])
        red = int(data[1])
        yellow = int(data[2])
        left = int(data[3])
        blue = int(data[4])

        # -----------------------------------------
        # 상태 문자열
        # -----------------------------------------
        found_text = 'YES' if found else 'NO'
        red_text = 'ON' if red else 'OFF'
        yellow_text = 'ON' if yellow else 'OFF'
        left_text = 'ON' if left else 'OFF'
        blue_text = 'ON' if blue else 'OFF'

        # -----------------------------------------
        # 화면 출력
        # -----------------------------------------
        print()
        print('========================================')
        print('       Traffic Light Result')
        print('========================================')
        print(f'Found  : {found_text}')
        print(f'Red    : {red_text}')
        print(f'Yellow : {yellow_text}')
        print(f'Left   : {left_text}')
        print(f'Blue   : {blue_text}')
        print('========================================')


# =============================================
# main
# =============================================
def main(args=None):

    rclpy.init(args=args)

    node = MyTrafficLight()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


# =============================================
# 프로그램 시작
# =============================================
if __name__ == '__main__':

    main()

