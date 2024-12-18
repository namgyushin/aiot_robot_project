#!/usr/bin/env python3
import rclpy

import cv2
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np
import message_filters
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image
from ultralytics import YOLO
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from rclpy.qos import qos_profile_sensor_data

model = YOLO('yolov8n.pt')

class Object_Detection(Node):
  
  def __init__(self):
    super().__init__("Object_Detection")
    print("Object_Detection")
    self.cv_bridge = CvBridge()

    self.spatial = rs.spatial_filter()
    self.temporal = rs.temporal_filter()
    
    self.image_sub = message_filters.Subscriber(self, Image, "/camera/camera/color/image_raw")
    self.depth_sub = message_filters.Subscriber(self, Image, "/camera/camera/depth/image_rect_raw")

    
    self.info_sub = self.create_subscription(
      CameraInfo, 
      "/camera/camera/depth/camera_info", 
      self.info_callback, 
      qos_profile_sensor_data
    )

    self.sync = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.depth_sub], queue_size=10, slop=0.1)
    self.sync.registerCallback(self.sub_merge_callback)


    self.declare_parameter('camera_frame', 'camera_link')
    self.declare_parameter('base_frame', 'link1')

    self.camera_frame = self.get_parameter('camera_frame').value
    self.base_frame = self.get_parameter('base_frame').value

    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)
    self.tf_broadcaster = TransformBroadcaster(self)

    self.info_msg = None
    self.intrinsic_mat = None
    self.distortion = None
    self.transform = None

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
    
    self.get_logger().info(
      f"Transform from 'camera_link' to 'link1':\n"
      f"Translation: x={self.transform.transform.translation.x}, "
      f"y={self.transform.transform.translation.y}, "
      f"z={self.transform.transform.translation.z}\n"
      f"Rotation: x={self.transform.transform.rotation.x}, "
      f"y={self.transform.transform.rotation.y}, "
      f"z={self.transform.transform.rotation.z}, "
      f"w={self.transform.transform.rotation.w}"
    )

    self.destroy_subscription(self.info_sub)


  def convert_depth_to_phys_coord_using_realsense(self, x, y, depth):
    intrinsics = rs.intrinsics()
    intrinsics.width = self.info_msg.width
    intrinsics.height = self.info_msg.height
    intrinsics.ppx = self.info_msg.k[2]
    intrinsics.ppy = self.info_msg.k[5]
    intrinsics.fx = self.info_msg.k[0]
    intrinsics.fy = self.info_msg.k[4]
    intrinsics.model  = rs.distortion.none
    intrinsics.coeffs = [i for i in self.info_msg.d]  
    return rs.rs2_deproject_pixel_to_point(intrinsics, [x, y], depth) 


  def broadcast_transform(self, pose):
    # TransformStamped 메시지 생성
    t = TransformStamped()
    t.header.stamp = self.get_clock().now().to_msg()
    t.header.frame_id = 'camera_link'
    t.child_frame_id = 'robot_base'

    pose_tf = np.array(pose)

    # Translation (x, y, z)
    t.transform.translation.x = pose_tf[2]/1000
    t.transform.translation.y = -pose_tf[0]/1000
    t.transform.translation.z = -pose_tf[1]/1000

    # Rotation (쿼터니언)
    t.transform.rotation.x = 0.0
    t.transform.rotation.y = 0.0
    t.transform.rotation.z = 0.0
    t.transform.rotation.w = 1.0

    # Transform 발행
    self.tf_broadcaster.sendTransform(t)



  def sub_merge_callback(self, msg_color: Image, msg_depth: Image):

    if(self.info_msg == None):
      print("info_msg none")
      return

    cv_color_image = self.cv_bridge.imgmsg_to_cv2(msg_color)
    cv_color_image = cv2.cvtColor(cv_color_image, cv2.COLOR_BGR2RGB)

    cv_depth_image = self.cv_bridge.imgmsg_to_cv2(msg_depth)

    results = model(cv_color_image, verbose=False)

    for result in results:
      boxes = result.boxes
      for box in boxes:
        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
        cls_id = int(box.cls[0].cpu().numpy())
        class_name = model.names[cls_id]

        if class_name != "cell phone" :
          continue
     
        confidence = box.conf[0].cpu().numpy()

        if confidence < 0.5:
          continue

        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)

        object_depth = np.median(cv_depth_image[y1:y2, x1:x2])
        label = f"{object_depth}mm"

        
        pose = self.convert_depth_to_phys_coord_using_realsense(cx, cy, int(object_depth))

        self.broadcast_transform(pose)

        cv2.rectangle(cv_color_image, (x1, y1), (x2, y2), (252, 119, 30), 2)
        cv2.putText(cv_color_image, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (252, 119, 30), 2)

    # cv2.imshow("Color Image", cv_color_image)
    # cv2.waitKey(33)










def main():
  rclpy.init()
  node = Object_Detection()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    node.get_logger().info('Stopped by Keyboard')
  finally :
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
  main()
