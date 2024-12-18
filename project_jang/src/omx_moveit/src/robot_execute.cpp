#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/msg/pose.hpp>
#include "omx_moveit_msgs/srv/arm_plan.hpp"
#include "omx_moveit_msgs/srv/arm_plan_exe.hpp"

namespace robot_execute
{

static const rclcpp::Logger LOGGER = rclcpp::get_logger("RobotExecute");

using namespace std;

class RobotExecute {

public:
  RobotExecute(const rclcpp::Node::SharedPtr& node) : 
    node_(node),
    move_group_arm_(node, PLANNING_GROUP_ARM),
    move_group_gripper_(node, PLANNING_GROUP_GRIPPER)
  {
    RCLCPP_INFO(LOGGER, "RobotExecute");

    arm_command_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/arm/command",
      10,
      std::bind(&RobotExecute::arm_command_sub_callback, this, std::placeholders::_1)
    );

    gripper_command_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/gripper/command",
      10,
      std::bind(&RobotExecute::gripper_command_sub_callback, this, std::placeholders::_1)
    );

    arm_plan_service_ = node->create_service<omx_moveit_msgs::srv::ArmPlan>(
      "/arm/plan",
      std::bind(
        &RobotExecute::arm_plan_service_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2
      )
    );

    arm_execute_service_ = node->create_service<omx_moveit_msgs::srv::ArmPlanExe>(
      "/arm/plan/execute",
      std::bind(
        &RobotExecute::arm_execute_service_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2
      )
    );
  }

private:
  rclcpp::Node::SharedPtr node_;
  const std::string PLANNING_GROUP_ARM = "arm";
  const std::string PLANNING_GROUP_GRIPPER = "gripper";

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr arm_command_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gripper_command_sub_;
  moveit::planning_interface::MoveGroupInterface move_group_arm_;
  moveit::planning_interface::MoveGroupInterface move_group_gripper_;
  rclcpp::Service<omx_moveit_msgs::srv::ArmPlan>::SharedPtr arm_plan_service_;
  rclcpp::Service<omx_moveit_msgs::srv::ArmPlanExe>::SharedPtr arm_execute_service_;

  void arm_plan_service_callback(
    const std::shared_ptr<omx_moveit_msgs::srv::ArmPlan::Request> request,
    std::shared_ptr<omx_moveit_msgs::srv::ArmPlan::Response> response
  ) {
    auto translation = request->target_data.translation;
    auto rotation = request->target_data.rotation;

    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = translation.x;
    target_pose.position.y = translation.y;
    target_pose.position.z = translation.z;
    target_pose.orientation.x = rotation.x;
    target_pose.orientation.y = rotation.y;
    target_pose.orientation.z = rotation.z;
    target_pose.orientation.w = rotation.w;

    move_group_arm_.setPoseTarget(target_pose);
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_arm_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    response->result = success;

    if(success) {
      RCLCPP_INFO(LOGGER, "arm_plan_service_callback success");
      response->planning_time = plan.planning_time_;
      response->start_state = plan.start_state_;
      response->trajectory = plan.trajectory_;
    }

  }

  void arm_execute_service_callback(
    const std::shared_ptr<omx_moveit_msgs::srv::ArmPlanExe::Request> request,
    std::shared_ptr<omx_moveit_msgs::srv::ArmPlanExe::Response> response
  ) {
    RCLCPP_INFO(LOGGER, "arm_execute_service_callback");
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.planning_time_ = request->planning_time;
    plan.start_state_ = request->start_state;
    plan.trajectory_ = request->trajectory;
    move_group_arm_.execute(plan);
    response->result = true;
  }

  void arm_action_sub_callback(const geometry_msgs::msg::TransformStamped msg) {
    RCLCPP_INFO(LOGGER, "arm_action_sub_callback");
    
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = msg.transform.translation.x;
    target_pose.position.y = msg.transform.translation.y;
    target_pose.position.z = msg.transform.translation.z;
    target_pose.orientation.x = msg.transform.rotation.x;
    target_pose.orientation.y = msg.transform.rotation.y;
    target_pose.orientation.z = msg.transform.rotation.z;
    target_pose.orientation.w = msg.transform.rotation.w;

    move_group_arm_.setPoseTarget(target_pose);
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_arm_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if(success) {
      move_group_arm_.execute(plan);
    } else {
      RCLCPP_ERROR(LOGGER, "Execute failed! arm_action_sub_callback");
    }

  }


  void arm_command_sub_callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(LOGGER, "arm_command_sub_callback %s", msg->data.c_str());

    move_group_arm_.setNamedTarget(msg->data);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_arm_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if(success) {
      move_group_arm_.execute(plan);
    } else {
      RCLCPP_ERROR(LOGGER, "Execute failed! arm_command_sub_callback");
    }
  }


  void gripper_command_sub_callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(LOGGER, "gripper_command_sub_callback %s", msg->data.c_str());

    move_group_gripper_.setNamedTarget(msg->data);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_gripper_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if(success) {
      move_group_gripper_.execute(plan);
    } else {
      RCLCPP_ERROR(LOGGER, "Execute failed! gripper_command_sub_callback");
    }
  }

};

}


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("robot_execute", node_options);

  robot_execute::RobotExecute RobotExecute(node);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
