#!/usr/bin/env python3
import rclpy

import cv2
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from rclpy.qos import qos_profile_sensor_data

ARUCO_DICT = {
  "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
  "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
  "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
  "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
  "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
  "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
  "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
  "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
  "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
  "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
  "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
  "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
  "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
  "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
  "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
  "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
  "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}

class Object_Detection_Aruco(Node):
  
  def __init__(self):
    super().__init__("Object_Detection_Aruco")
    print("Object_Detection_Aruco")

    self.declare_parameter("aruco_dictionary_name", "DICT_4X4_1000")
    self.declare_parameter("aruco_marker_side_length", 0.003)

    aruco_dictionary_name = self.get_parameter("aruco_dictionary_name").get_parameter_value().string_value
    self.aruco_marker_side_length = self.get_parameter("aruco_marker_side_length").get_parameter_value().double_value

    self.cv_bridge = CvBridge()

    self.image_pub = self.create_publisher(Image, "/detect/image_raw", 10)

    self.marker_pub = self.create_publisher(TransformStamped, "/detected/marker", 10)
    
    self.info_sub = self.create_subscription(
      Image,
      "/camera/camera/color/image_raw",
      self.image_callback,
      qos_profile_sensor_data
    )
    self.info_sub = self.create_subscription(
      CameraInfo,
      "/camera/camera/color/camera_info",
      self.info_callback,
      qos_profile_sensor_data
    )

    self.declare_parameter('camera_frame', 'camera_link')
    self.declare_parameter('base_frame', 'link1')

    self.camera_frame = self.get_parameter('camera_frame').value
    self.base_frame = self.get_parameter('base_frame').value

    self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_dictionary_name])
    self.aruco_parameters = cv2.aruco.DetectorParameters()

    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)
    self.tf_broadcaster = TransformBroadcaster(self)

    self.info_msg = None
    self.intrinsic_mat = None
    self.distortion = None
    self.transform = None

    self.blue_BGR = (255, 0, 0)

    self.marker_size = 26
    self.marker_3d_edges = np.array([
      [0, 0, 0],
      [0, self.marker_size, 0],
      [self.marker_size, self.marker_size, 0],
      [self.marker_size, 0, 0]
    ], dtype='float32').reshape((4, 1, 3))

  def info_callback(self, info_msg):
    self.info_msg = info_msg
    self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
    self.distortion = np.array(self.info_msg.d)

    try:
      self.transform = self.tf_buffer.lookup_transform(
        target_frame='link1',
        source_frame='camera_link',
        time=rclpy.time.Time()
      )
    except Exception as e:
      self.get_logger().error(f"depth_callback Exception : {e}")
      return
       
    if(self.transform == None):
      self.get_logger().info(f"depth_callback transform None")
      return

    self.destroy_subscription(self.info_sub)


  def image_callback(self, msg_color: Image):
    if(self.info_msg == None):
      print("info_msg none")
      return
    
    current_frame = self.cv_bridge.imgmsg_to_cv2(msg_color)
    
    corners, marker_ids, _ = cv2.aruco.detectMarkers(
      current_frame, self.aruco_dictionary, parameters=self.aruco_parameters
    )

    if marker_ids is None:
      return

    for i, marker_id in enumerate(marker_ids):
      corner = corners[i]
      corner = np.array(corner).reshape((4, 2))
      (topLeft, topRight, bottomRight, bottomLeft) = corner

      topRightPoint    = (int(topRight[0]),      int(topRight[1]))
      topLeftPoint     = (int(topLeft[0]),       int(topLeft[1]))
      bottomRightPoint = (int(bottomRight[0]),   int(bottomRight[1]))
      bottomLeftPoint  = (int(bottomLeft[0]),    int(bottomLeft[1]))

      cv2.circle(current_frame, topLeftPoint, 4, self.blue_BGR, -1)
      cv2.circle(current_frame, topRightPoint, 4, self.blue_BGR, -1)
      cv2.circle(current_frame, bottomRightPoint, 4, self.blue_BGR, -1)
      cv2.circle(current_frame, bottomLeftPoint, 4, self.blue_BGR, -1)

      _, _, tvec = cv2.solvePnP(self.marker_3d_edges, corners[i], self.intrinsic_mat, self.distortion)
      self.broadcast_transform_aruco(tvec, marker_id[0])

    msg = self.cv_bridge.cv2_to_imgmsg(current_frame, "bgr8")
    msg.header.frame_id = 'camera_link'
    self.image_pub.publish(msg)


  def broadcast_transform_aruco(self, tvec, id):
    child_frame_id = f'target_{id}'

    t = TransformStamped()
    t.header.stamp = self.get_clock().now().to_msg()
    t.header.frame_id = 'camera_color_frame'
    t.child_frame_id = child_frame_id

    t.transform.translation.x = round(tvec[2][0]/1000, 2)
    t.transform.translation.y = -round(tvec[0][0]/1000, 2)
    t.transform.translation.z = -round(tvec[1][0]/1000, 2)

    t.transform.rotation.x = 0.0
    t.transform.rotation.y = 0.0
    t.transform.rotation.z = 0.0
    t.transform.rotation.w = 1.0

    self.tf_broadcaster.sendTransform(t)
    self.marker_pub.publish(t)

    # try:
    #   transform = self.tf_buffer.lookup_transform(
    #     target_frame='link1',
    #     source_frame=child_frame_id,
    #     time=rclpy.time.Time()
    #   )
    #   print(transform)
    # except Exception as e:
    #   self.get_logger().error(f"depth_callback Exception : {e}")



def main():
  rclpy.init()
  node = Object_Detection_Aruco()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    node.get_logger().info('Stopped by Keyboard')
  finally :
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
  main()
