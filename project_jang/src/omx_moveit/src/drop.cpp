#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "std_msgs/msg/string.hpp"
#include "omx_moveit_msgs/msg/pick_and_place_status.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace drop
{

static const rclcpp::Logger LOGGER = rclcpp::get_logger("drop");

class Drop {

public:
  Drop(const rclcpp::Node::SharedPtr& node) : node_(node) {
    RCLCPP_INFO(LOGGER, "Drop");

    drop_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/drop",
      10,
      std::bind(&Drop::drop_sub_callback, this, std::placeholders::_1)
    );

    joint_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, 
      std::bind(&Drop::joint_state_callback, this, std::placeholders::_1)
    );


    drop_status_pub_ = node_->create_publisher<omx_moveit_msgs::msg::PickAndPlaceStatus>("/drop/status", 10);
    drop_status_timer_ = node_->create_wall_timer(std::chrono::milliseconds(500), std::bind(&Drop::send_drop_status, this));
    gripper_command_pub_ = node_->create_publisher<std_msgs::msg::String>("/gripper/command", 10);
    arm_command_pub_ = node_->create_publisher<std_msgs::msg::String>("/arm/command", 10);



  }

private:
  rclcpp::Node::SharedPtr node_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr drop_sub_;
  rclcpp::Publisher<omx_moveit_msgs::msg::PickAndPlaceStatus>::SharedPtr drop_status_pub_;

  rclcpp::TimerBase::SharedPtr drop_status_timer_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gripper_command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr arm_command_pub_;

  sensor_msgs::msg::JointState::SharedPtr prev_joint_state_;

  rclcpp::Clock clock_;
  rclcpp::Time exe_time_ = clock_.now();
  

  bool is_done_ = false;
  bool is_standby_ = true;
  bool is_turn_right_ = true;
  bool is_moving_ = true;
  int step = 0;


  void drop_sub_callback(const std_msgs::msg::String msg) {

    if(is_moving_){
      return;
    }

    std_msgs::msg::String temp;

    if(step == 0){
      temp.data = "standby";
      arm_command_pub_->publish(temp);
      step++;
      return;
    } 

    if(step == 1){
      if(is_standby_){
        return;
      }

      temp.data = "turn_right";
      arm_command_pub_->publish(temp);
      step++;
      return;
    } 
    
    if(step == 2){
      if(is_turn_right_){
        return;
      }
      temp.data = "open";
      gripper_command_pub_->publish(temp);
      step++;
      return;
    }

    if(step == 3){
      temp.data = "standby";
      arm_command_pub_->publish(temp);
      is_done_ = true;
    }
  }

  
  void send_drop_status(){
    omx_moveit_msgs::msg::PickAndPlaceStatus pick_and_place_status;
    geometry_msgs::msg::TransformStamped temp;
    bool done = true;
    bool success = true;

    pick_and_place_status.done = done;
    pick_and_place_status.success = success;
    pick_and_place_status.target_data = temp;

    if(is_done_){
      step = 0;
      is_standby_ = true;
      is_turn_right_ = true;
      is_done_ = false;
      drop_status_pub_->publish(pick_and_place_status);
    }
  }


  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (prev_joint_state_ == nullptr) {
      prev_joint_state_ = msg;
      return;
    }

    bool is_joint_change = is_joint_state_changed(*prev_joint_state_, *msg);
    is_moving_ = is_joint_change;

    rclcpp::Duration elapsed_time = clock_.now() - exe_time_;
    
    if(elapsed_time.seconds() < 0.5){
      return;
    }

    exe_time_ = clock_.now();

    if(is_done_){
      return;
    }

    prev_joint_state_ = msg;


    if(is_standby_ && step == 1){
      if(is_joint_change){
        return;
      }
      is_standby_ = false;
    }

    if(is_turn_right_ && step == 2){
      if(is_joint_change){
        return;
      }
      is_turn_right_ = false;
    }

    prev_joint_state_ = msg;
  }

  bool is_joint_state_changed(const sensor_msgs::msg::JointState &prev, const sensor_msgs::msg::JointState &current) {
    if (prev.position.size() != current.position.size()) {
      return true;
    }

    // 위치 값 비교
    const double threshold = 1e-6; // 작은 움직임은 노이즈로 간주
    for (size_t i = 0; i < prev.position.size(); ++i) {
      if (std::fabs(prev.position[i] - current.position[i]) > threshold) {
        return true;
      }
    }

    return false;
  }


};

}


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("drop", node_options);

  drop::Drop drop(node);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
