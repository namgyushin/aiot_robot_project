#ifndef ROS2_CONTROL__OMX_CONTROLLER_HPP_
#define ROS2_CONTROL__OMX_CONTROLLER_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "control_msgs/action/gripper_command.hpp"
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_action/create_server.hpp"
#include "rclcpp_action/server_goal_handle.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "realtime_tools/realtime_server_goal_handle.h"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace omx_moveit_gripper_controller
{
class RobotGripperController : public controller_interface::ControllerInterface
{
public:
  CONTROLLER_INTERFACE_PUBLIC
  RobotGripperController();

  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period
  ) override;

  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state
  ) override;

  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state
  ) override;

  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state
  ) override;

  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state
  ) override;

  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state
  ) override;

  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state
  ) override;


  rclcpp_action::GoalResponse goal_received_callback(
    const rclcpp_action::GoalUUID & uuid, 
    std::shared_ptr<const control_msgs::action::GripperCommand::Goal> goal
  );

  rclcpp_action::CancelResponse goal_cancelled_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::GripperCommand>> goal_handle
  );

  void goal_accepted_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::GripperCommand>> goal_handle
  );

protected:
  rclcpp_action::Server<control_msgs::action::GripperCommand>::SharedPtr action_server_;

  std::vector<std::string> joint_names_;
  std::vector<std::string> command_interface_types_;
  std::vector<std::string> state_interface_types_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<control_msgs::msg::GripperCommand>> gripper_msg_external_point_ptr_;
  bool new_msg_ = false;
  rclcpp::Time start_time_;
  std::shared_ptr<control_msgs::msg::GripperCommand> gripper_msg_;

  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_position_command_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_effort_command_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_position_state_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_effort_state_interface_;

  std::unordered_map<
    std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> *>
    command_interface_map_ = {
      {"position", &joint_position_command_interface_},
      {"effort", &joint_effort_command_interface_}};

  std::unordered_map<
    std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> *>
    state_interface_map_ = {
      {"position", &joint_position_state_interface_},
      {"effort", &joint_effort_state_interface_}};

};

}  // namespace omx_moveit_gripper_controller

#endif  // ROS2_CONTROL__OMX_CONTROLLER_HPP_
