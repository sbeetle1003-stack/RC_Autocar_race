#!/usr/bin/env python3

import cv2
import apriltag
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy

# =============================================
# AR Tag 결과 표시 노드
# =============================================
class MyARDetectNode(Node):

    def __init__(self):

        super().__init__('my_ar_detect')

        # =============================================
        # CvBridge
        # =============================================
        self.bridge = CvBridge()

        # =============================================
        # AprilTag Detector
        #
        # 화면에 Tag의 모서리를 표시하기 위해 사용
        # =============================================
        self.detector = apriltag.Detector()

        # =============================================
        # 화면 출력 및 AR 정보 출력 주기 설정
        #
        # 0.1초 = 10 Hz
        # 0.2초 = 5 Hz
        # 0.5초 = 2 Hz
        # 1.0초 = 1 Hz
        # =============================================
        self.timer_period = 1.0

        # =============================================
        # 최근 카메라 영상 저장
        # =============================================
        self.latest_image = None

        # =============================================
        # 최근 AR Tag 결과 저장
        #
        # {
        #     tag_id: {
        #         'x': X,
        #         'y': Y,
        #         'z': Z
        #     }
        # }
        # =============================================
        self.tag_results = {}

        # -----------------------------------------
        # 카메라 이미지 구독
        # -----------------------------------------
        #self.subscription = self.create_subscription(
        #    Image, '/image_raw', self.image_callback, qos_profile_sensor_data)

        qos_profile = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.subscription = self.create_subscription(
            Image, '/image_raw', self.image_callback, qos_profile)

        # =============================================
        # /ardetect_result Subscriber
        # =============================================
        self.result_subscription = self.create_subscription(
            Float32MultiArray, '/ardetect_result', self.result_callback, 1)

        # =============================================
        # 화면 표시 및 AR 정보 출력 Timer
        # =============================================
        self.timer = self.create_timer(
            self.timer_period, self.timer_callback)

        self.get_logger().info(
            '----- My AR Detect node started -----')

        self.get_logger().info(
            f'Display timer period: {self.timer_period:.2f} sec')

    # =============================================
    # /ardetect_result Callback
    #
    # 메시지가 도착할 때마다 화면이나 터미널을
    # 처리하지 않고 최신 결과만 저장
    # =============================================
    def result_callback(self, msg):

        data = msg.data

        # =============================================
        # 데이터 개수 확인
        #
        # Tag 하나당
        # [ID, X, Y, Z]
        # 4개의 데이터 사용
        # =============================================
        if len(data) % 4 != 0:

            self.get_logger().warning(
                f'Invalid ardetect_result data: '
                f'{len(data)} values')

            return

        # =============================================
        # 이전 결과 삭제
        #
        # []가 들어오면 이전 Tag 정보도 모두 삭제됨
        # =============================================
        self.tag_results.clear()

        # =============================================
        # 모든 Tag 정보 저장
        # =============================================
        for i in range(0, len(data), 4):

            tag_id = int(data[i])
            x_cm = float(data[i + 1])
            y_cm = float(data[i + 2])
            z_cm = float(data[i + 3])

            self.tag_results[tag_id] = {
                'x': x_cm,
                'y': y_cm,
                'z': z_cm
            }

    # =============================================
    # /image_raw Callback
    #
    # 새로운 영상이 들어오면
    # 최신 영상만 저장
    # =============================================
    def image_callback(self, msg):

        try:

            image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8')

            # =============================================
            # 최신 영상 저장
            # =============================================
            self.latest_image = image

        except Exception as e:

            self.get_logger().error(
                f'Image conversion error: {e}')

    # =============================================
    # Timer Callback
    #
    # 지정된 주기마다
    # 1. AR 정보 터미널 출력
    # 2. 화면 처리
    # 3. 화면 표시
    # =============================================
    def timer_callback(self):

        # =============================================
        # AR Tag 정보 터미널 출력
        # =============================================
        if not self.tag_results:

            self.get_logger().info(
                '----- No AR Tag -----')

        else:

            tag_count = len(self.tag_results)

            self.get_logger().info(
                f'----- Detected {tag_count} Tag(s) -----')

            # =============================================
            # Tag 정보 출력
            # =============================================
            for tag_id, result in self.tag_results.items():

                x_cm = result['x']
                y_cm = result['y']
                z_cm = result['z']

                self.get_logger().info(
                    f'Tag #{tag_id}: '
                    f'X={x_cm:.1f} cm, '
                    f'Y={y_cm:.1f} cm, '
                    f'Z={z_cm:.1f} cm'
                )

        # =============================================
        # 아직 카메라 영상이 들어오지 않은 경우
        # =============================================
        if self.latest_image is None:
            return

        # =============================================
        # 최신 영상 사용
        #
        # 원본 영상을 직접 수정하지 않기 위해 복사
        # =============================================
        image = self.latest_image.copy()

        # =============================================
        # 화면 표시용 영상
        # =============================================
        display_image = image.copy()

        # =============================================
        # Gray Image
        # =============================================
        gray_image = cv2.cvtColor(
            image,
            cv2.COLOR_BGR2GRAY)

        # =============================================
        # AprilTag 검출
        #
        # 모서리 위치를 화면에 그리기 위해 사용
        # =============================================
        detections = self.detector.detect(
            gray_image)

        # =============================================
        # 검출된 모든 Tag 처리
        # =============================================
        for detection in detections:

            tag_id = int(detection.tag_id)

            # =============================================
            # Tag 모서리
            # =============================================
            for corner in detection.corners:

                x = int(corner[0])
                y = int(corner[1])

                # -----------------------------------------
                # 초록색 빈 원
                # -----------------------------------------
                cv2.circle(
                    display_image,
                    (x, y),
                    5,
                    (0, 255, 0),
                    2)

            # =============================================
            # Tag 중심
            # =============================================
            center_x = int(detection.center[0])
            center_y = int(detection.center[1])

            # -----------------------------------------
            # 빨간색 채워진 원
            # -----------------------------------------
            cv2.circle(
                display_image,
                (center_x, center_y),
                5,
                (0, 0, 255),
                -1)

            # =============================================
            # /ardetect_result에서 위치 정보 가져오기
            # =============================================
            if tag_id in self.tag_results:

                x_cm = self.tag_results[tag_id]['x']
                y_cm = self.tag_results[tag_id]['y']
                z_cm = self.tag_results[tag_id]['z']

                # =============================================
                # Z 거리 표시
                # =============================================
                z_text = f'Z = {z_cm:.1f} cm'

                cv2.putText(
                    display_image,
                    z_text,
                    (center_x + 10, center_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 0, 255),
                    1,
                    cv2.LINE_AA)

            # =============================================
            # Tag ID 표시
            # =============================================
            id_text = f'ID: {tag_id}'

            cv2.putText(
                display_image,
                id_text,
                (center_x + 10, center_y - 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
                cv2.LINE_AA)

        # =============================================
        # 카메라 영상 화면 출력
        # =============================================
        cv2.imshow(
            'AR Tag Detection',
            display_image)

        cv2.waitKey(1)


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

        cv2.destroyAllWindows()

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()
