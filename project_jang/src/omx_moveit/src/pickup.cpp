#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "std_msgs/msg/string.hpp"
#include "omx_moveit_msgs/msg/pick_and_place_status.hpp"
#include "omx_moveit_msgs/srv/arm_plan.hpp"
#include "omx_moveit_msgs/srv/arm_plan_exe.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include <chrono>
#include <thread>

namespace pickup
{

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pickup");

class Pickup {

public:
  Pickup(const rclcpp::Node::SharedPtr& node) : node_(node) {
    RCLCPP_INFO(LOGGER, "Pickup");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pickup_sub_ = node_->create_subscription<geometry_msgs::msg::TransformStamped>(
      "/pickup",
      10,
      std::bind(&Pickup::pickup_sub_callback, this, std::placeholders::_1)
    );

    joint_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, 
      std::bind(&Pickup::joint_state_callback, this, std::placeholders::_1)
    );

    pickup_status_pub_ = node_->create_publisher<omx_moveit_msgs::msg::PickAndPlaceStatus>("/pickup/status", 10);
    arm_command_pub_ = node_->create_publisher<std_msgs::msg::String>("/arm/command", 10);
    gripper_command_pub_ = node_->create_publisher<std_msgs::msg::String>("/gripper/command", 10);
    arm_plan_service_client_ = node_->create_client<omx_moveit_msgs::srv::ArmPlan>("/arm/plan");
    arm_execute_service_client_ = node_->create_client<omx_moveit_msgs::srv::ArmPlanExe>("/arm/plan/execute");

    pickup_status_timer_ = node_->create_wall_timer(std::chrono::milliseconds(500), std::bind(&Pickup::send_pickup_status, this));
  }

  void initialize(){
    RCLCPP_INFO(LOGGER, "Pickup temp");
    std_msgs::msg::String temp;
    temp.data = "standby";
    arm_command_pub_->publish(temp);
    RCLCPP_INFO(LOGGER, "Pickup temp done");
  }


private:
  rclcpp::Node::SharedPtr node_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr pickup_sub_;

  rclcpp::Publisher<omx_moveit_msgs::msg::PickAndPlaceStatus>::SharedPtr pickup_status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr arm_command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gripper_command_pub_;
  rclcpp::Client<omx_moveit_msgs::srv::ArmPlan>::SharedPtr arm_plan_service_client_;
  rclcpp::Client<omx_moveit_msgs::srv::ArmPlanExe>::SharedPtr arm_execute_service_client_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::shared_ptr<geometry_msgs::msg::TransformStamped> msg_;
  std::shared_ptr<omx_moveit_msgs::srv::ArmPlanExe::Request> execute_msg_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<omx_moveit_msgs::srv::ArmPlanExe::Request>> execute_msg_external_point_ptr_;

  std::shared_ptr<trajectory_msgs::msg::JointTrajectory> goal_position_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<trajectory_msgs::msg::JointTrajectory>> goal_position_external_point_ptr_;

  rclcpp::TimerBase::SharedPtr pickup_status_timer_;

  rclcpp::Clock clock_;
  rclcpp::Time plan_time_ = clock_.now();

  bool is_initialize_ = false;
  bool is_arm_runing_ = false;
  bool is_arm_plan_runing_ = false;
  bool is_arm_plan_done_ = false;
  bool is_arm_plan_success_ = false;
  bool is_arm_execute_done_ = false;
  bool is_arm_execute_success_ = false;

  void reset(){
    is_arm_runing_ = false;
    is_arm_plan_runing_ = false;
    is_arm_plan_done_ = false;
    is_arm_plan_success_ = false;
    is_arm_execute_done_ = false;
    is_arm_execute_success_ = false;
  }

  void send_pickup_status(){
    omx_moveit_msgs::msg::PickAndPlaceStatus pick_and_place_status;
    bool done = false;
    bool success = is_arm_execute_success_;

    if(is_arm_plan_done_ && is_arm_execute_done_){
      done = true;
      is_arm_runing_ = false;
      is_arm_plan_runing_ = false;
      is_arm_plan_done_ = false;
      is_arm_plan_success_ = false;
      is_arm_execute_done_ = false;
      is_arm_execute_success_ = false;
    }

    if(msg_ != nullptr){
      pick_and_place_status.target_data = *msg_;
    }

    pick_and_place_status.done = done;
    pick_and_place_status.success = success;
    pickup_status_pub_->publish(pick_and_place_status);
  }


  void pickup_status_pub(bool done, bool success){
    omx_moveit_msgs::msg::PickAndPlaceStatus pick_and_place_status;
    pick_and_place_status.target_data = *msg_;
    pick_and_place_status.done = done;
    pick_and_place_status.success = success;
    pickup_status_pub_->publish(pick_and_place_status);
  }


  void pickup_sub_callback(const geometry_msgs::msg::TransformStamped msg) {
    if(is_arm_runing_ && is_arm_plan_runing_){
      // 현재 plan, execute 실행 중일 경우 리턴
      return;
    }

    is_arm_runing_ = true;
    msg_ = std::make_shared<geometry_msgs::msg::TransformStamped>(msg);

    if(!is_arm_plan_runing_ && !is_arm_plan_done_){
      // 현재 plan 호출이 진행중이 아닐경우 계획 arm_plan_service_client_ 실행
      is_arm_plan_runing_ = true;
      plan_time_ = clock_.now();
      arm_plan(msg);
    }

    if(is_arm_plan_done_ && !is_arm_plan_success_){
      rclcpp::Duration elapsed_time = clock_.now() - plan_time_;
      if(elapsed_time.seconds() > 5){
        reset();
      }
      // plan 이 실패 하였을 경우 리턴
      is_arm_runing_ = false;
      pickup_status_pub(true, false);
      return;
    }

    if(is_arm_plan_runing_){
      return;
    }

    if(!is_arm_execute_done_){
      gripper("open");
    }

    execute_msg_ = *execute_msg_external_point_ptr_.readFromRT();
    execute_msg_external_point_ptr_.reset();

    if(execute_msg_ == nullptr){
      pickup_status_pub(true, false);
      return;
    }

    arm_execute(execute_msg_);
  }


  void gripper(std::string pose) {
    std_msgs::msg::String msg;
    msg.data = pose;

    gripper_command_pub_->publish(msg);
  }



  void arm_plan(geometry_msgs::msg::TransformStamped transformStamped){

    auto request = std::make_shared<omx_moveit_msgs::srv::ArmPlan::Request>(); 
    request->target_data = transformStamped.transform;

    while (!arm_plan_service_client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(LOGGER, "arm_plan Interrupted while waiting for the service. Exiting.");
        is_arm_plan_runing_ = false;
        return;
      }
    }

    using ArmPlanResponseFuture = rclcpp::Client<omx_moveit_msgs::srv::ArmPlan>::SharedFuture;
    auto callback = [this](ArmPlanResponseFuture future) {
      auto result = future.get();

      is_arm_plan_runing_ = false;
      is_arm_plan_done_ = true;

      if(!result->result){
        is_arm_runing_ = false;
        is_arm_plan_success_ = false;
        return;
      }

      auto execute_msg = std::make_shared<omx_moveit_msgs::srv::ArmPlanExe::Request>(); 

      execute_msg->start_state = result->start_state;
      execute_msg->trajectory = result->trajectory;
      execute_msg->planning_time = result->planning_time;

      execute_msg_external_point_ptr_.writeFromNonRT(execute_msg);

      is_arm_plan_success_ = true;
    };

    is_arm_plan_runing_ = true;
    arm_plan_service_client_->async_send_request(request, callback);
  }


  void arm_execute(std::shared_ptr<omx_moveit_msgs::srv::ArmPlanExe_Request> request){

    goal_position_external_point_ptr_.writeFromNonRT(
      std::make_shared<trajectory_msgs::msg::JointTrajectory>(request->trajectory.joint_trajectory)
    );


    using ArmExecuteResponseFuture = rclcpp::Client<omx_moveit_msgs::srv::ArmPlanExe>::SharedFuture;
    auto callback = [this](ArmExecuteResponseFuture future) {
      auto result = future.get();

      is_arm_runing_ = false;
      is_arm_execute_done_ = true;

      if(!result->result){
        is_arm_execute_success_ = false;
        pickup_status_pub(true, false);
        return;
      }
      is_arm_execute_success_ = true;
    };
    
    while (!arm_execute_service_client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        is_arm_runing_ = false;
        is_arm_execute_done_ = true;
        is_arm_execute_success_ = false;
        return;
      }
    }

    arm_execute_service_client_->async_send_request(request, callback);
  }

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    goal_position_ = *goal_position_external_point_ptr_.readFromRT();

    if(goal_position_ == nullptr){
      return;
    }

    const auto& points = goal_position_->points;
    const auto& joint_names = goal_position_->joint_names;

    const auto& point = points.back();

    for (size_t idx = 0; idx < point.positions.size(); idx++) {

      auto it = std::find(joint_names.begin(), joint_names.end(), msg->name[idx]);
      if (it == joint_names.end()){
        return;
      }

      size_t index = std::distance(joint_names.begin(), it);
      
      if(msg->position[idx] != point.positions[index]){
        return;
      }
      
      RCLCPP_INFO(LOGGER, "Name: %s Point %.2f: / Last Name: %s Point %.2f:", msg->name[idx].c_str(),  msg->position[idx], joint_names[index].c_str(), point.positions[index]);
    }

    gripper("close");
    goal_position_external_point_ptr_.reset();
    pickup_status_pub(true, true);

  }


};

}


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("pickup", node_options);

  pickup::Pickup pickup(node);

  std::this_thread::sleep_for(std::chrono::seconds(3));
  pickup.initialize();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
