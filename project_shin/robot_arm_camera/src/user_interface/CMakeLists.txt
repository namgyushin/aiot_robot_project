cmake_minimum_required(VERSION 3.8)
project(user_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)  # action_msgs 패키지 의존성 추가
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# Generate messages, services, and actions
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/UserInt.msg"
  "srv/AddAndOdd.srv"
  "action/Fibonacci.action"
  "msg/ArithmeticArgument.msg"
  "srv/ArithmeticOperator.srv"
  "action/ArithmeticChecker.action"
  "action/Bool.action"  # Bool.action 추가
  DEPENDENCIES 
    builtin_interfaces
    action_msgs  # action_msgs 의존성 추가
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
