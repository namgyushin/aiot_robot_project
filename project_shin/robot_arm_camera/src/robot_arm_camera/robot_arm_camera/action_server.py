import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from user_interface.action import Bool
from sensor_msgs.msg import Image  # 이미지 메시지 타입
from rclpy.qos import QoSProfile


class Action_server(Node):
    def __init__(self):
        super().__init__('action_server')
        self.img = None
        self.action_server = ActionServer(self,
                                          bool,
                                          "true_false",
                                          execute_callback=self.execute_callback)
        
        self.create_subscription(
            Image,  # 메시지 타입
            '/camera/image',  # 이미지 토픽 이름
            self.img_callback,  # 콜백 함수
            QoSProfile(depth=10)  # QoS 설정
        )

    def img_callback(self, msg):
        """이미지 메시지를 처리하는 콜백 함수"""
        self.img = msg
        self.get_logger().info("이미지를 받았습니다. 사이즈: %d x %d" % (msg.width, msg.height))
        # 여기서 이미지를 처리하거나 객체 인식 관련 코드 추가 가능

    
    def execute_callback(self, goal_handle):
        
        # 컨베이어, 터틀봇 도착했다는 신호 확인
        # 토픽 섭스크라이브 - 이미지 : 객체 인식
        # 텔레옵 클라이언트로 최종 이동 명령



def main():
    rclpy.init()
    node = Action_server()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
    

if __name__ == '__main__':
    main() 