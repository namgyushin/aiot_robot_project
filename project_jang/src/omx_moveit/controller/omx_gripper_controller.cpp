
#include "omx_moveit/omx_gripper_controller.hpp"

#include <stddef.h>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <iostream>

#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

using config_type = controller_interface::interface_configuration_type;
using GripperCommand = control_msgs::action::GripperCommand;
using GoalHandleGripperCommand = rclcpp_action::ServerGoalHandle<GripperCommand>;

namespace omx_moveit_gripper_controller
{

auto logger_ = rclcpp::get_logger("omx_controller_RobotGripperController");

RobotGripperController::RobotGripperController() : controller_interface::ControllerInterface() {}


controller_interface::CallbackReturn RobotGripperController::on_init(){
  RCLCPP_INFO(logger_, "on_init");

    // should have error handling
  joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
  command_interface_types_ = auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
  state_interface_types_ = auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

  return CallbackReturn::SUCCESS;
}


controller_interface::InterfaceConfiguration RobotGripperController::command_interface_configuration() const {
  RCLCPP_INFO(logger_, "command_interface_configuration");
  controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

  conf.names.reserve(joint_names_.size() * command_interface_types_.size());
  for(const auto & joint_name : joint_names_){
    for(const auto & interface_type : command_interface_types_){
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }

  return conf;
}


controller_interface::InterfaceConfiguration RobotGripperController::state_interface_configuration() const {
  RCLCPP_INFO(logger_, "state_interface_configuration");
  controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

  conf.names.reserve(joint_names_.size() * state_interface_types_.size());
  for(const auto & joint_name : joint_names_){
    for(const auto & interface_type : state_interface_types_){
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }

  return conf;
}

controller_interface::CallbackReturn RobotGripperController::on_configure(const rclcpp_lifecycle::State &){
  RCLCPP_INFO(logger_, "on_configure");

  using namespace std::placeholders;
  action_server_ = rclcpp_action::create_server<GripperCommand>(
    get_node()->get_node_base_interface(), get_node()->get_node_clock_interface(),
    get_node()->get_node_logging_interface(), get_node()->get_node_waitables_interface(),
    "/gripper_controller/gripper_cmd",
    std::bind(&RobotGripperController::goal_received_callback, this, _1, _2),
    std::bind(&RobotGripperController::goal_cancelled_callback, this, _1),
    std::bind(&RobotGripperController::goal_accepted_callback, this, _1)
  );

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotGripperController::on_activate(const rclcpp_lifecycle::State &){
  RCLCPP_INFO(logger_, "RobotGripperController on_activate");

  joint_position_command_interface_.clear();
  joint_effort_command_interface_.clear();
  joint_position_state_interface_.clear();
  joint_effort_state_interface_.clear();

    // assign command interfaces
  for(auto & interface : command_interfaces_){
    command_interface_map_[interface.get_interface_name()]->push_back(interface);
  }

  // assign state interfaces
  for(auto & interface : state_interfaces_){
    state_interface_map_[interface.get_interface_name()]->push_back(interface);
  }

  return CallbackReturn::SUCCESS;
}


controller_interface::return_type RobotGripperController::update(const rclcpp::Time & time, const rclcpp::Duration & period){
  (void)time;
  (void)period;

  if(new_msg_){
    gripper_msg_ = *gripper_msg_external_point_ptr_.readFromRT();
    start_time_ = time;
    new_msg_ = false;
  }

  if(gripper_msg_ != nullptr){
    for(size_t i = 0; i < joint_position_command_interface_.size(); i++){
      joint_position_command_interface_[i].get().set_value(gripper_msg_->position);
    }

    gripper_msg_ = nullptr;
    return controller_interface::return_type::OK;
  }

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn RobotGripperController::on_deactivate(const rclcpp_lifecycle::State &){
  RCLCPP_INFO(logger_, "on_deactivate");
  release_interfaces();

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotGripperController::on_cleanup(const rclcpp_lifecycle::State &){
  RCLCPP_INFO(logger_, "on_cleanup");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotGripperController::on_error(const rclcpp_lifecycle::State &){
  RCLCPP_INFO(logger_, "on_error");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotGripperController::on_shutdown(const rclcpp_lifecycle::State &){
  RCLCPP_INFO(logger_, "on_shutdown");
  return CallbackReturn::SUCCESS;
}


rclcpp_action::GoalResponse RobotGripperController::goal_received_callback(
  const rclcpp_action::GoalUUID & uuid, 
  std::shared_ptr<const control_msgs::action::GripperCommand::Goal> goal
){
  (void) uuid;
  (void) goal;
  RCLCPP_INFO(logger_, "GoalResponse goal_received_callback");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RobotGripperController::goal_cancelled_callback(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::GripperCommand>> goal_handle
){
  (void) goal_handle;
  RCLCPP_INFO(logger_, "GoalResponse handle_cancel");
  return rclcpp_action::CancelResponse::ACCEPT;
}


void RobotGripperController::goal_accepted_callback(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::GripperCommand>> goal_handle
){
  RCLCPP_INFO(logger_, "GoalResponse handle_accepted");

  auto goal = goal_handle->get_goal();

  if(!goal){
    RCLCPP_ERROR(logger_, "Invalid goal received");
    return;
  }

  rclcpp::Clock clock;
  rclcpp::Time now = clock.now(); 
  rclcpp::Duration duration2(2, 0);
  
  control_msgs::msg::GripperCommand gripper_msg;

  gripper_msg = goal->command;
  auto gripper_msg_ptr = std::make_shared<control_msgs::msg::GripperCommand>(gripper_msg);

  new_msg_ = true;
  gripper_msg_external_point_ptr_.writeFromNonRT(gripper_msg_ptr);

  auto result = std::make_shared<control_msgs::action::GripperCommand::Result>();
  result->position = goal_handle->get_goal()->command.position;
  result->effort = goal_handle->get_goal()->command.max_effort;
  result->stalled = false;
  result->reached_goal = true;

  goal_handle->succeed(result);
}


}  // namespace omx_moveit_gripper_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(omx_moveit_gripper_controller::RobotGripperController, controller_interface::ControllerInterface)
