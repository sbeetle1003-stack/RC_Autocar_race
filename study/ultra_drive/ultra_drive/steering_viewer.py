#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from xycar_msgs.msg import XycarMotor
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import collections
import time

# 전역 변수로 데이터 리스트 선언
steering_angle_data = collections.deque(maxlen=200)
x_data = collections.deque(maxlen=200)

# 전역 변수로 ROS2 노드와 Executor 선언
steering_visualizer_node = None
executor = None

# ROS2 노드 클래스 정의
class SteeringVisualizer(Node):

    def __init__(self):
        super().__init__('xycar_steering_visualizer')
        self.subscription = self.create_subscription(
            XycarMotor,
            'xycar_motor',
            self.motor_callback,
            10)

    def motor_callback(self, msg):
        global steering_angle_data, x_data
        
        steering_angle_data.append(-msg.angle)
        x_data.append(time.time())

# matplotlib 그래프 업데이트 함수
def animate(i):
    global steering_visualizer_node, executor
    
    # ROS2 메시지 콜백을 처리 (메인 스레드에서)
    if steering_visualizer_node is not None:
        executor.spin_once(timeout_sec=0)
    
    ax.clear()
    
    if steering_angle_data:
        start_time = x_data[0] if x_data else 0
        relative_x_data = [t - start_time for t in x_data]
        
        ax.plot(relative_x_data, steering_angle_data, 'b-', label='Steering Angle (Left:+, Right:-)')
        
        ax.set_ylim(-105.0, 105.0)
        ax.set_ylabel('Steering Angle (Left:+100.0 ~ Right:-100.0)')
        ax.set_xlabel('Time (s)')
        ax.set_title('Real-time Xycar Steering Angle Visualization')
        ax.grid(True)
        ax.legend()
        ax.axhline(y=0, color='r', linestyle='--', linewidth=1, label='Straight (0.0)')

def main(args=None):
    global ax, steering_visualizer_node, executor
    
    rclpy.init(args=args)
    steering_visualizer_node = SteeringVisualizer()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(steering_visualizer_node)

    # matplotlib figure 및 axes 초기화
    fig, ax = plt.subplots()
    ani = FuncAnimation(fig, animate, interval=50)
    
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        steering_visualizer_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
