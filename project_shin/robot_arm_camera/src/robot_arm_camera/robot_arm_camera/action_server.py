import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from user_interface.action import Bool  # Bool 액션 메시지 타입
from sensor_msgs.msg import Image  # 이미지 메시지 타입
from rclpy.qos import QoSProfile
import cv2
from cv_bridge import CvBridge


class ActionServerNode(Node):
    def __init__(self):
        super().__init__('action_server')
        self.img = None
        self.bridge = CvBridge()  # cv_bridge 초기화
        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)  # ArUco 딕셔너리
        self.aruco_parameters = cv2.aruco.DetectorParameters()  # ArUco 탐지 파라미터 설정
        
        # 액션 서버 생성
        self.action_server = ActionServer(
            self,
            Bool,
            "bool_action",
            execute_callback=self.execute_callback
        )
        
        # 이미지 토픽에 대한 구독 생성
        self.create_subscription(
            Image,  # 메시지 타입
            '/camera/camera/color/image_raw',  # 이미지 토픽 이름
            self.img_callback,  # 콜백 함수
            QoSProfile(depth=10)  # QoS 설정
        )

        self.timer = self.create_timer(1.0, self.timer_callback)  # 1초마다 타이머 콜백

    def img_callback(self, msg):
        """이미지 메시지를 처리하는 콜백 함수"""
        self.img = msg
        self.get_logger().info(f"Received image with size: {msg.width} x {msg.height}")

    def timer_callback(self):
        """주기적으로 실행되는 콜백"""
        if self.img is None:
            self.get_logger().warning("No image received yet.")

    def execute_callback(self, goal_handle):
        """액션 실행 콜백 함수"""
        self.get_logger().info("Executing action callback")

        # 이미지가 아직 수신되지 않았다면 대기
        if self.img is None:
            self.get_logger().warning("No image received yet.")
            result = Bool.Result()
            result.success = False
            return result

        # ROS 이미지를 OpenCV 이미지로 변환
        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.img, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting ROS image to OpenCV: {e}")
            result = Bool.Result()
            result.success = False
            return result

        # ArUco 마커 탐지
        corners, marker_ids, rejected = cv2.aruco.detectMarkers(cv_image, self.aruco_dictionary, parameters=self.aruco_parameters)

        if marker_ids is not None:
            # 마커가 탐지되었을 경우 마커에 사각형을 그려줍니다.
            cv2.aruco.drawDetectedMarkers(cv_image, corners, marker_ids)
            
            # 이미지 윈도우에 표시
            cv2.imshow('Detected ArUco Markers', cv_image)
            cv2.waitKey(1)  # 1ms 대기하여 윈도우가 업데이트되도록 합니다
            self.get_logger().info(f"Detected markers: {marker_ids}")
            
            result = Bool.Result()
            result.success = True  # 마커를 성공적으로 탐지했다고 가정
        else:
            self.get_logger().info("No markers detected.")
            result = Bool.Result()
            result.success = False  # 마커를 탐지하지 못했다고 가정

        # 액션 서버 상태 업데이트
        goal_handle.succeed()
        return result


def main():
    rclpy.init()
    node = ActionServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
    finally:
        cv2.destroyAllWindows()  # 종료 시 모든 OpenCV 윈도우 닫기


if __name__ == '__main__':
    main()
