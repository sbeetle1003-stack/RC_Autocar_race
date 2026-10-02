#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# =============================================
# Traffic Light Image Viewer
#
# /tflight_image 토픽을 구독하여
# 교통신호등 검출 결과가 표시된 영상을 화면에 출력한다.
# =============================================

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2


# =============================================
# Traffic Light Image Viewer Logic
# =============================================

class TrafficLightImageViewer:

    def __init__(self):
        self.bridge = CvBridge()

    def convert_image(self, msg):
        """
        ROS Image 메시지를 OpenCV 이미지로 변환
        """

        image = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        return image


# =============================================
# ROS2 Node
# =============================================

class TrafficLightImageNode(Node):

    def __init__(self):
        super().__init__('my_tflight_image')

        self.logic = TrafficLightImageViewer()

        # -----------------------------------------
        # /tflight_image 구독
        # -----------------------------------------
        self.subscription = self.create_subscription(
            Image,
            '/tflight_image',
            self.image_callback,
            1
        )

        self.get_logger().info(
            'Traffic Light Image Viewer started.'
        )

    # ---------------------------------------------
    # Image Callback
    # ---------------------------------------------

    def image_callback(self, msg):

        try:
            image = self.logic.convert_image(msg)

            cv2.imshow(
                'Traffic Light Detection',
                image
            )

            cv2.waitKey(1)

        except Exception as e:

            self.get_logger().error(
                f'Image conversion error: {e}'
            )


# =============================================
# Main
# =============================================

def main(args=None):

    rclpy.init(args=args)

    node = TrafficLightImageNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
