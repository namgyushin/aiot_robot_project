
#include "omx_moveit/omx_controller.hpp"

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
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFollowJointTrajectory = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

namespace omx_moveit
{

auto logger_ = rclcpp::get_logger("omx_controller");

RobotController::RobotController() : controller_interface::ControllerInterface() {}
controller_interface::CallbackReturn RobotController::on_init(){
  RCLCPP_INFO(logger_, "on_init");

  // should have error handling
  joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
  command_interface_types_ = auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
  state_interface_types_ = auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

  point_interp_.positions.assign(joint_names_.size(), 0);
  point_interp_.velocities.assign(joint_names_.size(), 0);

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration RobotController::command_interface_configuration() const {
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

controller_interface::InterfaceConfiguration RobotController::state_interface_configuration() const {
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



rclcpp_action::GoalResponse RobotController::goal_received_callback(
  const rclcpp_action::GoalUUID & uuid, 
  std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal
){
  (void) uuid;
  (void) goal;
  RCLCPP_INFO(logger_, "GoalResponse goal_received_callback");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RobotController::goal_cancelled_callback(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle
){
  (void) goal_handle;
  RCLCPP_INFO(logger_, "GoalResponse handle_cancel");
  return rclcpp_action::CancelResponse::ACCEPT;
}


void RobotController::goal_accepted_callback(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle
){
  RCLCPP_INFO(logger_, "GoalResponse handle_accepted");

    // 목표 Trajectory 가져오기
  auto goal = goal_handle->get_goal();
  if(!goal){
    RCLCPP_ERROR(logger_, "Invalid goal received");
    return;
  }
  rclcpp::Clock clock;
  rclcpp::Time now = clock.now(); 
  rclcpp::Duration duration2(2, 0);
  
  // trajectory_msgs::msg::JointTrajectory
  trajectory_msgs::msg::JointTrajectory trajectory_msg;

  // goal에서 trajectory 데이터 복사
  trajectory_msg = goal->trajectory;
  auto trajectory_msg_ptr = std::make_shared<trajectory_msgs::msg::JointTrajectory>(trajectory_msg);

  new_msg_ = true;
  traj_msg_external_point_ptr_.writeFromNonRT(trajectory_msg_ptr);

  // 목표 수행 완료로 상태 업데이트
  auto result = std::make_shared<FollowJointTrajectory::Result>();
  result->error_code = control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL;
  goal_handle->succeed(result);

  RCLCPP_INFO(logger_, "Trajectory execution succeeded.");
}


controller_interface::CallbackReturn RobotController::on_configure(const rclcpp_lifecycle::State &){
  RCLCPP_INFO(logger_, "on_configure");

  using namespace std::placeholders;
  action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
    get_node()->get_node_base_interface(), get_node()->get_node_clock_interface(),
    get_node()->get_node_logging_interface(), get_node()->get_node_waitables_interface(),
    std::string(get_node()->get_name()) + "/follow_joint_trajectory",
    std::bind(&RobotController::goal_received_callback, this, _1, _2),
    std::bind(&RobotController::goal_cancelled_callback, this, _1),
    std::bind(&RobotController::goal_accepted_callback, this, _1)
  );

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotController::on_activate(const rclcpp_lifecycle::State &){
  RCLCPP_INFO(logger_, "on_activate");

  // clear out vectors in case of restart
  joint_position_command_interface_.clear();
  joint_velocity_command_interface_.clear();
  joint_position_state_interface_.clear();
  joint_velocity_state_interface_.clear();

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


void interpolate_point(
  const trajectory_msgs::msg::JointTrajectoryPoint & point_1,
  const trajectory_msgs::msg::JointTrajectoryPoint & point_2,
  trajectory_msgs::msg::JointTrajectoryPoint & point_interp, double delta
){
  for(size_t i = 0; i < point_1.positions.size(); i++){
    point_interp.positions[i] = delta * point_2.positions[i] + (1.0 - delta) * point_1.positions[i];
  }

  for(size_t i = 0; i < point_1.velocities.size(); i++){
    point_interp.velocities[i] = delta * point_2.velocities[i] + (1.0 - delta) * point_1.velocities[i];
  }
}

void interpolate_trajectory_point(
  const trajectory_msgs::msg::JointTrajectory & traj_msg, 
  const rclcpp::Duration & cur_time,
  trajectory_msgs::msg::JointTrajectoryPoint & point_interp
){
  double traj_len = traj_msg.points.size();
  auto last_time = traj_msg.points[traj_len - 1].time_from_start;
  double total_time = last_time.sec + last_time.nanosec * 1E-9;

  size_t ind = cur_time.seconds() * (traj_len / total_time);
  ind = std::min(static_cast<double>(ind), traj_len - 2);
  double delta = cur_time.seconds() - ind * (total_time / traj_len);
  interpolate_point(traj_msg.points[ind], traj_msg.points[ind + 1], point_interp, delta);
}

controller_interface::return_type RobotController::update(const rclcpp::Time & time, const rclcpp::Duration & period){
  (void)period;

  if(new_msg_){
    trajectory_msg_ = *traj_msg_external_point_ptr_.readFromRT();
    start_time_ = time;
    new_msg_ = false;
  }

  if(trajectory_msg_ != nullptr){
    // interpolate_trajectory_point(*trajectory_msg_, time - start_time_, point_interp_);

    for(size_t i = 0; i < joint_position_command_interface_.size(); i++){
      joint_position_command_interface_[i].get().set_value(trajectory_msg_->points[0].positions[i]);
      // std::cerr << "trajectory_msg_.positions[i]" << trajectory_msg_->points[0].positions[i] << std::endl;
    }

    for(size_t i = 0; i < joint_velocity_command_interface_.size(); i++){
      joint_velocity_command_interface_[i].get().set_value(trajectory_msg_->points[0].velocities[i]);
      // std::cerr << "trajectory_msg_.velocities[i]" << trajectory_msg_->points[0].velocities[i] << std::endl;
    }

    if(trajectory_msg_->points.size() == 1){
      trajectory_msg_ = nullptr;
      return controller_interface::return_type::OK;
    }

    trajectory_msg_->points.erase(trajectory_msg_->points.begin());

    // std::cerr << "trajectory_msg_.points.size" << trajectory_msg_->points.size() << std::endl;

  }

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn RobotController::on_deactivate(const rclcpp_lifecycle::State &){
  RCLCPP_INFO(logger_, "on_deactivate");
  release_interfaces();

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotController::on_cleanup(const rclcpp_lifecycle::State &){
  RCLCPP_INFO(logger_, "on_cleanup");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotController::on_error(const rclcpp_lifecycle::State &){
  RCLCPP_INFO(logger_, "on_error");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotController::on_shutdown(const rclcpp_lifecycle::State &){
  RCLCPP_INFO(logger_, "on_shutdown");
  return CallbackReturn::SUCCESS;
}

}  // namespace omx_moveit

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(omx_moveit::RobotController, controller_interface::ControllerInterface)
