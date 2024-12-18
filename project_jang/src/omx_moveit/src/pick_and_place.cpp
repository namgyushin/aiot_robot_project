#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "std_msgs/msg/string.hpp"
#include "omx_moveit_msgs/msg/pick_and_place_status.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include "realtime_tools/realtime_buffer.h"
#include "sensor_msgs/msg/joint_state.hpp"

namespace pick_and_place
{

static const rclcpp::Logger LOGGER = rclcpp::get_logger("PickAndPlace");

class PickAndPlace {

public:
  PickAndPlace(const rclcpp::Node::SharedPtr& node) : node_(node) {
    RCLCPP_INFO(LOGGER, "PickAndPlace");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    marker_sub_ = node_->create_subscription<geometry_msgs::msg::TransformStamped>(
      "/detected/marker",
      10,
      std::bind(&PickAndPlace::marker_sub_callback, this, std::placeholders::_1)
    );

    pickup_status_sub_ = node_->create_subscription<omx_moveit_msgs::msg::PickAndPlaceStatus>(
      "/pickup/status",
      10,
      std::bind(&PickAndPlace::pickup_status_sub_callback, this, std::placeholders::_1)
    );

    drop_status_sub_ = node_->create_subscription<omx_moveit_msgs::msg::PickAndPlaceStatus>(
      "/drop/status",
      10,
      std::bind(&PickAndPlace::drop_status_sub_callback, this, std::placeholders::_1)
    );

    joint_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, 
      std::bind(&PickAndPlace::joint_state_callback, this, std::placeholders::_1)
    );


    pickup_timer_ = node_->create_wall_timer(std::chrono::milliseconds(500), std::bind(&PickAndPlace::send_pickup, this));
    drop_timer_ = node_->create_wall_timer(std::chrono::milliseconds(500), std::bind(&PickAndPlace::send_drop, this));
    pickup_pub_ = node_->create_publisher<geometry_msgs::msg::TransformStamped>("/pickup", 10);
    drop_pub_ = node_->create_publisher<std_msgs::msg::String>("/drop", 10);
    
    init_standby_position();
  }

  void init_standby_position(){
    standby_joint_state.header.stamp = clock_.now();
    standby_joint_state.header.frame_id = "";

    standby_joint_state.name = {"joint1", "joint2", "joint3", "joint4", "virtual_roll_joint", "virtual_yaw_joint"};

    standby_joint_state.position = {0.0, -0.68, 0.034, 1.60, 0.0, 0.0};
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr marker_sub_;
  rclcpp::Subscription<omx_moveit_msgs::msg::PickAndPlaceStatus>::SharedPtr pickup_status_sub_;
  rclcpp::Subscription<omx_moveit_msgs::msg::PickAndPlaceStatus>::SharedPtr drop_status_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr pickup_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr drop_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  sensor_msgs::msg::JointState standby_joint_state;

  rclcpp::TimerBase::SharedPtr pickup_timer_;
  rclcpp::TimerBase::SharedPtr drop_timer_;

  std::shared_ptr<geometry_msgs::msg::TransformStamped> traform_msg_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::TransformStamped>> traform_msg_external_point_ptr_;


  rclcpp::Clock clock_;
  rclcpp::Time now_ = clock_.now(); 

  bool is_new_msg_ = false;
  bool is_arm_pickup_runing_ = false;
  bool is_pickup_ = false;
  bool is_standby_ = false;

  void send_pickup(){
    if(is_pickup_){
      return;
    }

    if(is_new_msg_){
      traform_msg_ = *traform_msg_external_point_ptr_.readFromRT();
      is_new_msg_ = false;
    }

    if(traform_msg_ == nullptr){
      return;
    }

    pickup_pub_->publish(*traform_msg_);
    traform_msg_external_point_ptr_.reset();
  }


  void pickup_status_sub_callback(const omx_moveit_msgs::msg::PickAndPlaceStatus msg) {
    bool done = msg.done;
    bool success = msg.success;

    if(done){
      traform_msg_ = nullptr;
      is_arm_pickup_runing_ = false;
    }

    if(success){
      is_pickup_ = true;
    }
  }


  void send_drop(){
    if(!is_pickup_){
      return;
    }

    std_msgs::msg::String msg;

    drop_pub_->publish(msg);
  }


  void drop_status_sub_callback(const omx_moveit_msgs::msg::PickAndPlaceStatus msg) {
    bool success = msg.success;

    if(success){
      is_pickup_ = false;
    }
  }



  void marker_sub_callback(const geometry_msgs::msg::TransformStamped msg) {
    if(is_arm_pickup_runing_ || is_pickup_){
      return;
    }

    if(!is_standby_){
      return;
    }

    is_arm_pickup_runing_ = true;

    geometry_msgs::msg::TransformStamped transformStamped;

    try {
			transformStamped = tf_buffer_->lookupTransform(
        "link1", 
        msg.child_frame_id,
        tf2::TimePointZero
      );
    } catch (tf2::TransformException & ex) {
      is_arm_pickup_runing_ = false;
      return;
    }

    if(transformStamped.child_frame_id.empty()){
      is_arm_pickup_runing_ = false;
      return;
    }

    is_new_msg_ = true;
    auto transformStamped_ptr = std::make_shared<geometry_msgs::msg::TransformStamped>(transformStamped);
    traform_msg_external_point_ptr_.writeFromNonRT(transformStamped_ptr);
  }




  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    is_standby_ = is_joint_state_standby(*msg);
  }


  bool is_joint_state_standby(const sensor_msgs::msg::JointState &current) {
    // 위치 값 비교
    const double threshold = 1e-2;

    const auto& points = standby_joint_state.position;
    const auto& joint_names = standby_joint_state.name;

    for (size_t i = 0; i < points.size(); ++i) {
      auto it = std::find(joint_names.begin(), joint_names.end(), current.name[i]);
      if (it == joint_names.end()){
        continue;
      }

      size_t index = std::distance(joint_names.begin(), it);

      if (std::fabs(points[index] - current.position[i]) >= threshold) {
        return false;
      }
    }

    return true;
  }




};

}



int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;

  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("pick_and_place", node_options);
  pick_and_place::PickAndPlace pickAndPlace(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
