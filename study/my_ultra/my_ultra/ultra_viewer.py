#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32MultiArray
import matplotlib.pyplot as plt
import numpy as np

class UltraVisualizer(Node):
    def __init__(self):
        super().__init__('ultra_visualizer')

        # QoS 설정
        # 초음파 센서는 최신 데이터가 중요하므로 Best Effort 사용
        qos_profile = QoSProfile(
            depth=3, reliability=ReliabilityPolicy.BEST_EFFORT)

        # 초음파 데이터 구독
        self.subscription = self.create_subscription(
            Int32MultiArray, 'xycar_ultrasonic', self.callback, qos_profile)

        # 초기화
        self.ranges = None

        # Matplotlib 설정
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.ax.set_xlim(-150, 150) # X축 범위: -150cm ~ 150cm
        self.ax.set_ylim(-150, 150) # Y축 범위: -150cm ~ 150cm
        self.ax.set_aspect('equal') # X, Y축 스케일 동일하게 설정
        self.ax.set_xlabel("X (cm)")
        self.ax.set_ylabel("Y (cm)")
        self.ax.set_title("Ultrasonic Sensor Obstacle Map")
        self.ax.grid(True) # 그리드 표시

        # Xycar 위치 표시 (중앙 0,0)
        self.ax.plot(0, 0, 'ks', markersize=10, label='Xycar (Center)') # 검은색 사각형으로 Xycar 표시

        # 각 센서의 물리적인 배치 각도 (라디안)
        # 차량이 위쪽(Y+ 방향)을 바라보고, 첫 번째 센서가 왼쪽(X- 방향)을 보고,
        # 시계 방향으로 45도씩 배치되어 세 번째 센서가 정면(Y+ 방향)을 가리킨다고 가정
        self.sensor_angles = np.array([
            np.pi,          # 센서 0: 왼쪽 (180도)
            np.pi * (2/3),  # 센서 1: 왼쪽-정면 (135도)
            np.pi / 2,      # 센서 2: 정면 (90도)
            np.pi * (1/3),  # 센서 3: 정면-오른쪽 (45도)
            0,              # 센서 4: 오른쪽 (0도)
            -np.pi * (1/3), # 센서 5: 오른쪽-후방 (-45도)
            -np.pi / 2,     # 센서 6: 후방 (-90도)
            -np.pi * (2/3)  # 센서 7: 후방-왼쪽 (-135도)
        ])
            
        '''
        self.sensor_angles = np.array([
            np.pi,          # 센서 0: 왼쪽 (180도)
            np.pi * (3/4),  # 센서 1: 왼쪽-정면 (135도)
            np.pi / 2,      # 센서 2: 정면 (90도)
            np.pi * (1/4),  # 센서 3: 정면-오른쪽 (45도)
            0,              # 센서 4: 오른쪽 (0도)
            -np.pi * (1/4), # 센서 5: 오른쪽-후방 (-45도)
            -np.pi / 2,     # 센서 6: 후방 (-90도)
            -np.pi * (3/4)  # 센서 7: 후방-왼쪽 (-135도)
        ])
        '''
        
        # 초음파 센서로 감지된 장애물 위치를 나타낼 플롯 객체 초기화
        self.normal_obstacles, = self.ax.plot([], [], 'ro', markersize=12, label='Obstacles (< 100cm)') # 빨간색 원 마커
        self.exact_100cm_obstacles, = self.ax.plot([], [], 'kx', markersize=10, label='Obstacles (= 100cm)') # 검은색 X 마커

        # Xycar 중앙과 장애물을 연결할 점선 객체 초기화
        # 'b--'는 파란색 점선을 의미합니다.
        self.lines, = self.ax.plot([], [], 'b--', linewidth=1, label='Sensor Beams') 
        # 범례에 'Sensor Beams'를 추가했으니, 필요에 따라 조정하세요.

        self.ax.legend() # 범례 표시
        plt.ion()  # 인터랙티브 모드 활성화
        plt.show()

        self.get_logger().info("Ultra Visualizer Ready ----------")

        # 0.1초마다 실행되는 타이머 설정 (그래프 업데이트 주기)
        self.create_timer(0.1, self.timer_callback)

    def ultra_callback(self, msg):
        """초음파 데이터를 저장하는 콜백 함수"""
        self.ranges = np.array(msg.data)

    def timer_callback(self):
        """초음파 데이터를 주기적으로 업데이트하고 그래프를 그리는 함수"""
        if self.ranges is not None:
            # 100cm인 센서와 아닌 센서 분리
            is_100cm = (self.ranges == 100)
            not_100cm = (self.ranges != 100)

            # 100cm인 경우의 x, y 좌표
            x_100cm = self.ranges[is_100cm] * np.cos(self.sensor_angles[is_100cm])
            y_100cm = self.ranges[is_100cm] * np.sin(self.sensor_angles[is_100cm])

            # 100cm가 아닌 경우의 x, y 좌표
            x_normal = self.ranges[not_100cm] * np.cos(self.sensor_angles[not_100cm])
            y_normal = self.ranges[not_100cm] * np.sin(self.sensor_angles[not_100cm])

            # 모든 감지된 장애물 지점의 X, Y 좌표를 합칩니다.
            # 이 지점들로부터 Xycar 중앙(0,0)까지 선을 그립니다.
            all_x_coords = np.concatenate((x_100cm, x_normal))
            all_y_coords = np.concatenate((y_100cm, y_normal))

            # 점선을 그리기 위한 x, y 좌표 리스트 생성
            # 각 장애물 지점과 Xycar 중앙(0,0)을 연결합니다.
            line_x_data = []
            line_y_data = []
            for i in range(len(all_x_coords)):
                line_x_data.extend([0, all_x_coords[i], np.nan]) # Xycar 중앙, 장애물, 구분자(nan)
                line_y_data.extend([0, all_y_coords[i], np.nan]) # Xycar 중앙, 장애물, 구분자(nan)
            
            # 각 플롯 객체 업데이트
            self.exact_100cm_obstacles.set_data(x_100cm, y_100cm)
            self.normal_obstacles.set_data(x_normal, y_normal)
            self.lines.set_data(line_x_data, line_y_data) # 점선 업데이트
            
            # Matplotlib 캔버스 업데이트 (블로킹 방지)
            self.fig.canvas.draw_idle()
            plt.pause(0.01) # Matplotlib 이벤트 루프에 시간을 줌

            # # 디버깅을 위한 로그 출력 (필요시 주석 해제)
            # ranges_cm = [int(distance) for distance in self.ranges]
            # self.get_logger().info(f"Ultrasonic distance values (cm): {ranges_cm}")

def main(args=None):
    rclpy.init(args=args)
    node = UltraVisualizer()

    try:
        rclpy.spin(node) # 노드를 계속 실행하며 콜백 처리
    except KeyboardInterrupt:
        pass # Ctrl+C 예외 처리
    finally:
        node.destroy_node() # 노드 소멸
        rclpy.shutdown() # ROS2 종료

if __name__ == '__main__':

    main()
