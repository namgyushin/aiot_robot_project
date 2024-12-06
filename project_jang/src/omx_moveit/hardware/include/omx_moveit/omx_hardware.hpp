#ifndef ROS2_CONTROL__OMX_HARDWARE_HPP_
#define ROS2_CONTROL__OMX_HARDWARE_HPP_

#include "unordered_map"
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <sstream>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"

using hardware_interface::return_type;

namespace omx_moveit
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

typedef struct _Joint{
  double position;
  double velocity;
  double current;
  double effort;
  double position_command;
  double velocity_command;
  double effort_command;
} Joint;

typedef struct _ItemValue
{
  std::string item_name;
  int32_t value;
} ItemValue;

class HARDWARE_INTERFACE_PUBLIC RobotSystem : public hardware_interface::SystemInterface
{
public:
  ~RobotSystem();

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

protected:
  DynamixelWorkbench *dxl_wb_;
  rclcpp::Logger logger_ = rclcpp::get_logger("omx_hardware");
  std::vector<Joint> joints_;

  std::vector<std::string> split(std::string str, char Delimiter) {
    std::istringstream iss(str);
    std::string buffer;

    std::vector<std::string> result;

    while (getline(iss, buffer, Delimiter)) {
      result.push_back(buffer);
    }

    return result;
  }

private:
  // ROS Parameters
  std::string port_name_;
  int64_t baud_rate_;
  std::string yaml_file_;
  std::string interface_;
  char delimiter = ':';

  std::map<std::string, const ControlItem*> control_items_;
  std::map<std::string, uint32_t> dynamixel_;
  std::map<uint8_t, uint8_t> dynamixel_order_;
  std::vector<std::pair<std::string, ItemValue>> dynamixel_info_;

  bool initWorkbench(const std::string port_name, const uint32_t baud_rate);
  bool initDynamixels(const std::string yaml_file);
  bool initControlItems(void);
  bool initSDKHandlers(void);

  /// The size of this vector is (standard_interfaces_.size() x nr_joints)
  std::vector<double> joint_position_command_;
  std::vector<double> joint_velocities_command_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocities_;
  std::vector<double> ft_states_;
  std::vector<double> ft_command_;

  std::unordered_map<std::string, std::vector<std::string>> joint_interfaces = {{"position", {}}, {"velocity", {}}};

};

using std::string;

// trim from left 
inline std::string& ltrim(std::string& s, const char* t = " \t\n\r\f\v")
{
	s.erase(0, s.find_first_not_of(t));
	return s;
}
// trim from right 
inline std::string& rtrim(std::string& s, const char* t = " \t\n\r\f\v")
{
	s.erase(s.find_last_not_of(t) + 1);
	return s;
}
// trim from left & right 
inline std::string& trim(std::string& s, const char* t = " \t\n\r\f\v")
{
	return ltrim(rtrim(s, t), t);
}

}  // namespace omx_moveit

#endif  // ROS2_CONTROL__OMX_CONTROLLER_HPP_
