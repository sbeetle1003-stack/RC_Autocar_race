#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


# =============================================
# AR Tag 결과 출력 노드
# =============================================
class MyARDetectNode(Node):

    def __init__(self):

        super().__init__('my_ar')

        # =============================================
        # 출력 주기 설정
        # 0.5초 = 2 Hz
        # 1.0초 = 1 Hz
        # =============================================
        self.timer_period = 1.0

        # =============================================
        # 가장 최근에 받은 AR Tag 결과
        # =============================================
        self.ar_data = []

        # =============================================
        # /ardetect_result Subscriber
        # 새로운 메시지가 도착하면 데이터만 저장
        # =============================================
        self.subscription = self.create_subscription(
            Float32MultiArray, '/ardetect_result', self.ar_callback, 1)

        # =============================================
        # Timer : 지정한 주기마다 AR Tag 정보 출력
        # =============================================
        self.timer = self.create_timer(
            self.timer_period,
            self.timer_callback
        )

        self.get_logger().info('----- My AR Detect node started -----')

        self.get_logger().info(f'AR Tag output period: {self.timer_period:.2f} sec')

    # =============================================
    # AR Tag 결과 Callback
    #
    # 토픽이 도착할 때마다 출력하지 않고
    # 최신 데이터만 저장
    # =============================================
    def ar_callback(self, msg):

        self.ar_data = list(msg.data)

    # =============================================
    # Timer Callback
    #
    # 지정된 주기마다 실행
    # =============================================
    def timer_callback(self):

        data = self.ar_data

        # =============================================
        # Tag가 없는 경우 : ar_detector에서 []가 들어온 경우
        # =============================================
        if not data:

            self.get_logger().info('----- No AR Tag -----')
            return

        # =============================================
        # 데이터 개수 확인
        # Tag 하나당 [ID, X, Y, Z] 4개의 데이터 사용
        # =============================================
        if len(data) % 4 != 0:

            self.get_logger().warning(f'Invalid ardetect_result data: {len(data)} values')
            return

        # =============================================
        # Tag 개수
        # =============================================
        tag_count = len(data) // 4

        self.get_logger().info(f'----- Detected {tag_count} Tag(s) -----')

        # =============================================
        # Tag 정보 출력
        # =============================================
        for i in range(tag_count):

            index = i * 4

            tag_id = int(data[index])
            x_cm = data[index + 1]
            y_cm = data[index + 2]
            z_cm = data[index + 3]

            self.get_logger().info(
                f'Tag #{tag_id}: '
                f'X={x_cm:.1f} cm, '
                f'Y={y_cm:.1f} cm, '
                f'Z={z_cm:.1f} cm'
            )


# =============================================
# Main
# =============================================
def main(args=None):

    rclpy.init(args=args)
    node = MyARDetectNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':

    main()
