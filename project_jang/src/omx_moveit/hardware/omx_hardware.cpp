#include "omx_moveit/omx_hardware.hpp"

namespace omx_moveit
{


RobotSystem::~RobotSystem(){
  RCLCPP_INFO(logger_, "~RobotSystem exit");
  for(auto const& dxl:dynamixel_){
    dxl_wb_->torqueOff((uint8_t)dxl.second);
  }
}

CallbackReturn RobotSystem::on_init(const hardware_interface::HardwareInfo & info){

  if(hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS){
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "omx_hardware on_init!");
  RCLCPP_INFO(logger_, "HardwareInfo name: %s", info.name.c_str());
  RCLCPP_INFO(logger_, "HardwareInfo type: %s", info.type.c_str());
  RCLCPP_INFO(logger_, "HardwareInfo usb_port: %s", info_.hardware_parameters["usb_port"].c_str());
  RCLCPP_INFO(logger_, "HardwareInfo baud_rate: %s", info_.hardware_parameters["baud_rate"].c_str());
  RCLCPP_INFO(logger_, "HardwareInfo yaml_file: %s", info_.hardware_parameters["yaml_file"].c_str());
  RCLCPP_INFO(logger_, "HardwareInfo interface: %s", info_.hardware_parameters["interface"].c_str());

  port_name_ = info_.hardware_parameters["usb_port"];
  baud_rate_ = stoi(info_.hardware_parameters["baud_rate"]);
  yaml_file_ = info_.hardware_parameters["yaml_file"];
  interface_ = info_.hardware_parameters["interface"];

  joint_position_.assign(5, 0);
  joint_velocities_.assign(5, 0);
  joint_position_command_.assign(5, 0);
  joint_velocities_command_.assign(5, 0);

  ft_states_.assign(5, 0);
  ft_command_.assign(5, 0);


  // 하드웨어 인터페이스 출력 (상태 인터페이스)
  RCLCPP_INFO(logger_, "  State Interfaces:");
  for(const auto & joint : info_.joints){
    for(const auto & state_interface : joint.state_interfaces){
      RCLCPP_INFO(logger_, "    %s / %s", state_interface.name.c_str(), joint.name.c_str());
    }
  }

  // 하드웨어 인터페이스 출력 (명령 인터페이스)
  RCLCPP_INFO(logger_, "  Command Interfaces:");
  for(const auto & joint : info_.joints){
    for(const auto & command_interface : joint.command_interfaces){
      RCLCPP_INFO(logger_, "    %s / %s", command_interface.name.c_str(), joint.name.c_str());
    }
  }

  for(const auto & joint : info_.joints){
    for(const auto & interface : joint.state_interfaces){
      joint_interfaces[interface.name].push_back(joint.name);
    }
  }

  if(!initWorkbench(port_name_, baud_rate_)){
    RCLCPP_ERROR(logger_, "Please check USB port name");
    return CallbackReturn::ERROR;
  }

  if(!initDynamixels(yaml_file_)){
    RCLCPP_ERROR(logger_, "Please check control table (http://emanual.robotis.com/#control-table)");
    return CallbackReturn::ERROR;
  }

  if(!initControlItems()){
    RCLCPP_ERROR(logger_, "Please check control items");
    return CallbackReturn::ERROR;
  }

  if(!initSDKHandlers()){
    RCLCPP_ERROR(logger_, "Failed to set Dynamixel SDK Handler");
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "omx_hardware on_init done!");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotSystem::on_configure(const rclcpp_lifecycle::State & /*previous_state*/){
  RCLCPP_INFO(logger_, "Successfully configured!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RobotSystem::export_state_interfaces(){
  RCLCPP_INFO(logger_, "export_state_interfaces");
  std::vector<hardware_interface::StateInterface> state_interfaces;

  int ind = 0;
  for(const auto & joint_name : joint_interfaces["position"]){
    state_interfaces.emplace_back(joint_name, "position", &joint_position_[ind++]);
  }

  ind = 0;
  for(const auto & joint_name : joint_interfaces["velocity"]){
    state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_[ind++]);
  }

  state_interfaces.emplace_back("tcp_fts_sensor", "force.x", &ft_states_[0]);
  state_interfaces.emplace_back("tcp_fts_sensor", "force.y", &ft_states_[1]);
  state_interfaces.emplace_back("tcp_fts_sensor", "force.z", &ft_states_[2]);
  state_interfaces.emplace_back("tcp_fts_sensor", "torque.x", &ft_states_[3]);
  state_interfaces.emplace_back("tcp_fts_sensor", "torque.y", &ft_states_[4]);
  state_interfaces.emplace_back("tcp_fts_sensor", "torque.z", &ft_states_[5]);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobotSystem::export_command_interfaces(){
  RCLCPP_INFO(logger_, "export_command_interfaces");
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  int ind = 0;
  for(const auto & joint_name : joint_interfaces["position"]){
    command_interfaces.emplace_back(joint_name, "position", &joint_position_command_[ind++]);
  }

  ind = 0;
  for(const auto & joint_name : joint_interfaces["velocity"]){
    command_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_command_[ind++]);
  }

  command_interfaces.emplace_back("tcp_fts_sensor", "force.x", &ft_command_[0]);
  command_interfaces.emplace_back("tcp_fts_sensor", "force.y", &ft_command_[1]);
  command_interfaces.emplace_back("tcp_fts_sensor", "force.z", &ft_command_[2]);
  command_interfaces.emplace_back("tcp_fts_sensor", "torque.x", &ft_command_[3]);
  command_interfaces.emplace_back("tcp_fts_sensor", "torque.y", &ft_command_[4]);
  command_interfaces.emplace_back("tcp_fts_sensor", "torque.z", &ft_command_[5]);

  return command_interfaces;
}

CallbackReturn RobotSystem::on_activate(const rclcpp_lifecycle::State & previous_state){
  (void)previous_state;
  RCLCPP_INFO(logger_, "System activated.");

  return CallbackReturn::SUCCESS;
}

CallbackReturn RobotSystem::on_deactivate(const rclcpp_lifecycle::State & previous_state){
  (void)previous_state;
  RCLCPP_INFO(logger_, "System deactivated.");

  return CallbackReturn::SUCCESS;
}

return_type RobotSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period){

  for(auto i = 0ul; i < joint_velocities_command_.size(); i++){
    if(std::isnan(joint_velocities_command_[i])){
      joint_velocities_command_[i] = 0;
    }
    joint_velocities_[i] = joint_velocities_command_[i];
    joint_position_[i] += joint_velocities_command_[i] * period.seconds();
  }

  for(auto i = 0ul; i < joint_position_command_.size(); i++){
    if(std::isnan(joint_position_command_[i])) {
      joint_position_command_[i] = 0;
    }
    joint_position_[i] = joint_position_command_[i];
  }

  return return_type::OK;
}

return_type RobotSystem::write(const rclcpp::Time &, const rclcpp::Duration &){

  // RCLCPP_INFO(logger_, "write");

  uint8_t id_array[dynamixel_.size()];
  int32_t dynamixel_position[dynamixel_.size()];
  uint8_t idx = 0;
  uint8_t id = 0;
  uint8_t order = 0;
  float position_command = 0;
  const char* log;

  for(auto const& dxl : dynamixel_){
    id = (uint8_t)dxl.second;
    order = dynamixel_order_[id];
    position_command = joint_position_command_[order];

    id_array[idx] = id;
    
    dynamixel_position[idx] = dxl_wb_->convertRadian2Value(id, position_command);

    if(strcmp(dxl.first.c_str(), "gripper") == 0){
      dynamixel_position[idx] = dxl_wb_->convertRadian2Value(id, position_command * 150.0);
    }
    idx ++;
  }

  uint8_t sync_write_handler = 0; // 0: position, 1: velocity, 2: effort
  dxl_wb_->syncWrite(sync_write_handler, id_array, dynamixel_.size(), dynamixel_position, 1, &log);

  return return_type::OK;
}

bool RobotSystem::initWorkbench(const std::string port_name, const uint32_t baud_rate){
  bool result = false;
  const char* log;
  dxl_wb_ = new DynamixelWorkbench;

  RCLCPP_INFO(logger_, "initWorkbench");

  result = dxl_wb_->init(port_name.c_str(), baud_rate, &log);
  if(result == false){
    RCLCPP_ERROR(logger_, "%s", log);
  }

  return result;
}

bool RobotSystem::initDynamixels(const std::string yaml_file){
  std::cout << "initDynamixels " << std::endl;

  std::ifstream openFile(yaml_file.data());
  if(openFile.is_open()){
    std::string line;
    std::string name;
    uint32_t order = 0;

    while(getline(openFile, line)){
      std::vector<std::string>items = split(line, delimiter);
      std::string item = items[0];
      // RCLCPP_INFO(logger_, "initDynamixels item:  %s val: %s", item.c_str(), items[1]);

      if(isspace(line.front()) == 0){
        name = trim(item);
        continue;
      }

      if(item.compare("ID")){
        dynamixel_[trim(name)] = stoi(items[1]);
        dynamixel_order_[stoi(items[1])] = order++;
      }

      ItemValue item_value = {trim(item), stoi(items[1])};
      std::pair<std::string, ItemValue> info(trim(name), item_value);

      dynamixel_info_.push_back(info);
    }
    openFile.close();
  }

  const char* log;
  bool result = false;
  uint8_t joint_size = 0;
  
  for(auto const& dxl : dynamixel_){
    uint16_t model_number = 0;
    result = dxl_wb_->ping((uint8_t)dxl.second, &model_number, &log);
    
    if(result == false){
      RCLCPP_ERROR(logger_, "%s", log);
      RCLCPP_ERROR(logger_, "Can't find Dynamixel ID '%d'", dxl.second);
      return result;
    }

    if (joint_size < (uint8_t)dxl.second){
      joint_size = (uint8_t)dxl.second;
    }

    joints_.resize(joint_size);
  }


  
  for(auto const& dxl : dynamixel_){
    dxl_wb_->torqueOff((uint8_t)dxl.second);

    for(auto const& info:dynamixel_info_){
      if(dxl.first == info.first){
        if(info.second.item_name != "ID" && info.second.item_name != "Baud_Rate"){
          bool result = dxl_wb_->itemWrite((uint8_t)dxl.second, info.second.item_name.c_str(), info.second.value, &log);
          if(result == false){
            RCLCPP_ERROR(logger_, "%s", log);
            RCLCPP_ERROR(logger_, "Failed to write value[%d] on items[%s] to Dynamixel[Name : %s, ID : %d]", info.second.value, info.second.item_name.c_str(), dxl.first.c_str(), dxl.second);
            return false;
          }
        }
      }
    }
  }

  // Torque On after setting up all servo
  for(auto const& dxl : dynamixel_){
    RCLCPP_INFO(logger_, "torqueOn Name : %s, ID : %d,", dxl.first.c_str(), dxl.second);
    dxl_wb_->torqueOn((uint8_t)dxl.second);
  }

  return true;
}

bool RobotSystem::initControlItems(void){
  RCLCPP_INFO(logger_, "initControlItems");
  auto it = dynamixel_.begin();

  const ControlItem *goal_position = dxl_wb_->getItemInfo((uint8_t)it->second, "Goal_Position");

  if(goal_position == NULL) {
    return false;
  }

  const ControlItem *goal_velocity = dxl_wb_->getItemInfo((uint8_t)it->second, "Goal_Velocity");
  
  if(goal_velocity == NULL){
    goal_velocity = dxl_wb_->getItemInfo((uint8_t)it->second, "Moving_Speed");
  }

  if(goal_velocity == NULL){
    return false;
  }

  const ControlItem *goal_current = dxl_wb_->getItemInfo((uint8_t)it->second, "Goal_Current");
  
  if(goal_current == NULL){
    goal_current = dxl_wb_->getItemInfo((uint8_t)it->second, "Present_Load");
  }

  if(goal_current == NULL){
    return false;
  }

  const ControlItem *present_position = dxl_wb_->getItemInfo((uint8_t)it->second, "Present_Position");
  
  if(present_position == NULL){
    return false;
  }

  const ControlItem *present_velocity = dxl_wb_->getItemInfo((uint8_t)it->second, "Present_Velocity");
  
  if(present_velocity == NULL){
    present_velocity = dxl_wb_->getItemInfo((uint8_t)it->second, "Present_Speed");
  }

  if(present_velocity == NULL){
    return false;
  }

  const ControlItem *present_current = dxl_wb_->getItemInfo((uint8_t)it->second, "Present_Current");
  
  if(present_current == NULL){
    present_current = dxl_wb_->getItemInfo((uint8_t)it->second, "Present_Load");
  }

  if(present_current == NULL){
    return false;
  }

  control_items_["Goal_Position"] = goal_position;
  control_items_["Goal_Velocity"] = goal_velocity;
  control_items_["Goal_Current"] = goal_current;

  control_items_["Present_Position"] = present_position;
  control_items_["Present_Velocity"] = present_velocity;
  control_items_["Present_Current"] = present_current;

  return true;
}

bool RobotSystem::initSDKHandlers(void){
  RCLCPP_INFO(logger_, "initSDKHandlers");
  bool result = false;
  const char* log = NULL;

  result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_Position"]->address, control_items_["Goal_Position"]->data_length, &log);

  if(result == false){
    RCLCPP_ERROR(logger_, "%s", log);
    return result;
  }

  RCLCPP_INFO(logger_, "%s", log);

  result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_Velocity"]->address, control_items_["Goal_Velocity"]->data_length, &log);

  if(result == false){
    RCLCPP_ERROR(logger_, "%s", log);
    return result;
  }

  RCLCPP_INFO(logger_, "%s", log);

  result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_Current"]->address, control_items_["Goal_Current"]->data_length, &log);
  
  if(result == false){
    RCLCPP_ERROR(logger_, "%s", log);
    return result;
  }

  RCLCPP_INFO(logger_, "%s", log);

  if(dxl_wb_->getProtocolVersion() == 2.0f){
    uint16_t start_address = std::min(control_items_["Present_Position"]->address, control_items_["Present_Current"]->address);

    /* As some models have an empty space between Present_Velocity and Present Current, read_length is modified as below.*/
    uint16_t read_length = control_items_["Present_Position"]->data_length + control_items_["Present_Velocity"]->data_length + control_items_["Present_Current"]->data_length;

    result = dxl_wb_->addSyncReadHandler(
      start_address,
      read_length,
      &log
    );

    if(result == false){
      RCLCPP_ERROR(logger_, "%s", log);
      return result;
    }
  }

  return result;
}


}  // namespace omx_moveit


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(omx_moveit::RobotSystem, hardware_interface::SystemInterface)