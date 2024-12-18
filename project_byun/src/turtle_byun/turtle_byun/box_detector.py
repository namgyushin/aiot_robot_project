import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class BoxDetector(Node):
    def __init__(self):
        super().__init__('box_detector')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # 카메라 토픽
            self.image_callback,
            10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # 메시지를 OpenCV 이미지로 변환
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 파란색 범위 정의 (HSV 색상 공간에서 파란색)
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([100, 150, 0])
        upper_blue = np.array([140, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # 마스크 적용해서 파란색 부분 찾기
        result = cv2.bitwise_and(cv_image, cv_image, mask=mask)

        # 상자 위치 찾기 (Contour 사용)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500:  # 상자의 크기를 결정하는 조건
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)

        # 결과를 화면에 띄우기
        cv2.imshow('Detected Blue Box', cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    box_detector = BoxDetector()
    rclpy.spin(box_detector)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
