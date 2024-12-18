cmake_minimum_required(VERSION 3.22)
project(omx_moveit)

find_package(ament_cmake REQUIRED)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

set(HW_IF_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  hardware_interface
  dynamixel_workbench_toolbox
  yaml-cpp
)
set(REF_GEN_INCLUDE_DEPENDS
  kdl_parser
  rclcpp
  trajectory_msgs
)
set(CONTROLLER_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  controller_interface
  realtime_tools
  trajectory_msgs
  rsl
)

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
foreach(Dependency IN ITEMS ${HW_IF_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${REF_GEN_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
find_package(std_msgs REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(omx_moveit_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(
  omx_moveit
  SHARED
  hardware/omx_hardware.cpp
  controller/omx_arm_controller.cpp
  controller/omx_gripper_controller.cpp
)


add_executable(pick_and_place src/pick_and_place.cpp)
ament_target_dependencies(
  pick_and_place PUBLIC
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  omx_moveit_msgs
  moveit_task_constructor_core
  realtime_tools
)

add_executable(drop src/drop.cpp)
ament_target_dependencies(
  drop PUBLIC
  rclcpp
  std_msgs
  geometry_msgs
  omx_moveit_msgs
)

add_executable(pickup src/pickup.cpp)
ament_target_dependencies(
  pickup PUBLIC
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  omx_moveit_msgs
  moveit_task_constructor_core
  realtime_tools
  sensor_msgs
  trajectory_msgs
)

add_executable(robot_execute src/robot_execute.cpp)
ament_target_dependencies(
  robot_execute PUBLIC
  rclcpp
  std_msgs
  moveit_task_constructor_core
  moveit_ros_planning_interface
  omx_moveit_msgs
)



install(TARGETS 
  pick_and_place
  drop
  pickup
  robot_execute
  DESTINATION lib/${PROJECT_NAME}
)


target_compile_features(omx_moveit PUBLIC cxx_std_17)
target_include_directories(omx_moveit PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/controller/include>
$<INSTALL_INTERFACE:include/omx_moveit>
)
ament_target_dependencies(
  omx_moveit PUBLIC
  ${HW_IF_INCLUDE_DEPENDS}
  ${CONTROLLER_INCLUDE_DEPENDS}
)


# install python modules
ament_python_install_package(${PROJECT_NAME})

# install python executables
install(PROGRAMS
  scripts/object_detection.py
  scripts/object_detection_aruco.py
  DESTINATION lib/${PROJECT_NAME}
)




install(DIRECTORY launch DESTINATION share/${PROJECT_NAME} PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY scripts DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

pluginlib_export_plugin_description_file(hardware_interface omx_moveit.xml)
pluginlib_export_plugin_description_file(controller_interface omx_moveit.xml)

install(
  DIRECTORY hardware/include/
  DESTINATION include/omx_moveit
)

install(
  DIRECTORY controller/include/
  DESTINATION include/omx_moveit
)

install(TARGETS omx_moveit
  EXPORT export_omx_moveit
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## EXPORTS
ament_export_targets(export_omx_moveit HAS_LIBRARY_TARGET)
ament_export_dependencies(${HW_IF_INCLUDE_DEPENDS} ${REF_GEN_INCLUDE_DEPENDS} ${CONTROLLER_INCLUDE_DEPENDS})
ament_package()