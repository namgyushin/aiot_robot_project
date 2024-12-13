#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/msg/pose.hpp>

using namespace std;

class PickAndPlace : public rclcpp::Node {

public:
  PickAndPlace() : Node("pick_and_place") {
    arm_sub_ = create_subscription<std_msgs::msg::String>(
      "/arm/goal_state",
      10,
      std::bind(&PickAndPlace::arm_sub_callback, this, std::placeholders::_1)
    );

    gripper_sub_ = create_subscription<std_msgs::msg::String>(
      "/gripper/goal_state",
      10,
      std::bind(&PickAndPlace::gripper_sub_callback, this, std::placeholders::_1)
    );
  }
  

  void initialize_move_group() {
    move_group_arm_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), PLANNING_GROUP_ARM);
    move_group_gripper_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), PLANNING_GROUP_GRIPPER);    
  }


private:
  const std::string PLANNING_GROUP_ARM = "arm";
  const std::string PLANNING_GROUP_GRIPPER = "gripper";

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr arm_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gripper_sub_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_gripper_;


// ros2 topic pub /arm/goal_state std_msgs/msg/String "data: 'standby'"
// ros2 topic pub /gripper/goal_state std_msgs/msg/String "data: 'open'"


  void arm_sub_callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(get_logger(), "arm_sub_callback");

    move_group_arm_->setNamedTarget(msg->data);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if(success) {
      move_group_arm_->execute(plan);
    } else {
      RCLCPP_ERROR(get_logger(), "Planning failed!");
    }
  }


  void gripper_sub_callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(get_logger(), "gripper_sub_callback");

    move_group_gripper_->setNamedTarget(msg->data);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_gripper_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if(success) {
      move_group_gripper_->execute(plan);
    } else {
      RCLCPP_ERROR(get_logger(), "Planning failed!");
    }
  }

};


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PickAndPlace>();
  node->initialize_move_group();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
