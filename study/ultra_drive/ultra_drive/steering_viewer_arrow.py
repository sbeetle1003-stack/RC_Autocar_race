#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from xycar_msgs.msg import XycarMotor

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import collections
import numpy as np


# 조향각 데이터를 저장
arrow_data = collections.deque(maxlen=200)
current_x = 0.0


class SteeringViewer(Node):

    def __init__(self):
        super().__init__('xycar_steering_visualizer')

        # xycar_motor 토픽 구독
        self.subscription = self.create_subscription(
            XycarMotor,
            'xycar_motor',
            self.motor_callback,
            10
        )

    # 조향각을 받아서 저장
    def motor_callback(self, msg):
        global arrow_data, current_x
        steering_angle = msg.angle
        angle_degrees = -steering_angle * 0.2
        arrow_data.append((current_x, angle_degrees))
        current_x += 1.0


# 그래프 업데이트
def animate(i):
    ax.clear()

    if arrow_data:
        for x_start, angle_deg in arrow_data:
            length = 1.0
            angle_rad = np.deg2rad(angle_deg)
            dx = length * np.cos(angle_rad)
            dy = length * np.sin(angle_rad)

            ax.arrow(
                x_start, 0, dx, dy,
                width=0.2,
                head_width=0.8,
                head_length=0.8,
                fc='blue',
                ec='blue'
            )

        min_x = arrow_data[0][0]
        max_x = arrow_data[-1][0]
        ax.set_xlim(min_x, max_x + 2.0)

        max_y = length * np.sin(np.deg2rad(20))
        ax.set_ylim(-max_y - 0.5, max_y + 0.5)

    ax.set_title('Real-time Steering Angle Arrow Visualization')
    ax.set_xlabel('Time Steps')
    ax.set_ylabel('Steering Direction')

    ax.axhline(
        y=0,
        color='r',
        linestyle='--',
        linewidth=1
    )

    ax.grid(True)
    ax.set_aspect('equal', adjustable='box')


def main(args=None):
    global ax

    rclpy.init(args=args)
    node = SteeringViewer()

    fig, ax = plt.subplots()

    # 그래프 업데이트 50ms = 20Hz
    ani = FuncAnimation(
        fig,
        animate,
        interval=50
    )

    # ROS2와 matplotlib를 함께 처리
    while rclpy.ok() and plt.fignum_exists(fig.number):
        rclpy.spin_once(
            node,
            timeout_sec=0.01
        )
        plt.pause(0.01)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
