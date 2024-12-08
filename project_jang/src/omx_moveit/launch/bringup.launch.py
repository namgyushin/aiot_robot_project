import os
from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg, add_debuggable_node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
  moveit_config = MoveItConfigsBuilder("open_manipulator_x", package_name="omx_moveit").to_moveit_configs()
  launch_package_path = moveit_config.package_path

  # Parameters
  usb_port = LaunchConfiguration('usb_port', default='/dev/ttyUSB0')
  baud_rate = LaunchConfiguration('baud_rate', default='1000000')
  yaml_file = LaunchConfiguration(
    'yaml_file', 
    default=PathJoinSubstitution(
      [FindPackageShare("omx_moveit"), "config", "hardware.yaml"]
    )
  )
  interface = LaunchConfiguration('interface', default='position')

  robot_description_content = Command([
    PathJoinSubstitution([FindExecutable(name="xacro")]),
    " ",
    PathJoinSubstitution(
      [FindPackageShare("omx_moveit"), "config", "open_manipulator_x.urdf.xacro"]
    ),
    " ",
    "usb_port:=",
    usb_port,
    " ",
    "baud_rate:=",
    baud_rate,
    " ",
    "yaml_file:=",
    yaml_file,
    " ",
    "interface:=",
    interface,
  ])

  kinematics_yaml = os.path.join(get_package_share_directory('omx_moveit'), 'config', 'kinematics.yaml')
  print(kinematics_yaml)
  # Example for loading the parameters

  robot_description = {"robot_description": robot_description_content}

  ld = LaunchDescription()
  ld.add_action(DeclareBooleanLaunchArg("use_sim_time", default_value=True))

  ld.add_action(
    Node(
      package="robot_state_publisher",
      executable="robot_state_publisher",
      name="robot_state_publisher",
      # output="screen",
      # respawn=True,
      parameters=[robot_description],
    )
  )

  ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=True))

  ld.add_action(
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
        str(launch_package_path / "launch/moveit_rviz.launch.py")
      ),
      condition=IfCondition(LaunchConfiguration("use_rviz")),
    )
  )

  ld.add_action(
    Node(
      package="controller_manager",
      executable="ros2_control_node",
      parameters=[
        robot_description,
        str(launch_package_path / "config/ros2_controllers.yaml"),
      ],
      remappings=[
        ("~/robot_description", "/robot_description"),
      ],
      output="both",
    )
  )

  ld.add_action(
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
        str(launch_package_path / "launch/spawn_controllers.launch.py")
      ),
    )
  )




  # ld.add_action(
  #   IncludeLaunchDescription(
  #     PythonLaunchDescriptionSource(
  #       str(launch_package_path / "launch/move_group.launch.py")
  #     ),
  #   )
  # )


  ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
  ld.add_action(
    DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True)
  )
  ld.add_action(
    DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True)
  )
  # load non-default MoveGroup capabilities (space separated)
  ld.add_action(
    DeclareLaunchArgument(
      "capabilities",
      default_value=moveit_config.move_group_capabilities["capabilities"],
    )
  )
  # inhibit these default MoveGroup capabilities (space separated)
  ld.add_action(
    DeclareLaunchArgument(
      "disable_capabilities",
      default_value=moveit_config.move_group_capabilities["disable_capabilities"],
    )
  )

  # do not copy dynamics information from /joint_states to internal robot monitoring
  # default to false, because almost nothing in move_group relies on this information
  ld.add_action(DeclareBooleanLaunchArg("monitor_dynamics", default_value=False))

  should_publish = LaunchConfiguration("publish_monitored_planning_scene")

  move_group_moveit_config = (
    MoveItConfigsBuilder("open_manipulator_x", package_name="omx_moveit")
    .robot_description(file_path="config/open_manipulator_x.urdf.xacro")
    .robot_description_kinematics(file_path=kinematics_yaml)
    .trajectory_execution(file_path="config/moveit_controllers.yaml")
    .planning_scene_monitor(
      publish_robot_description=True, publish_robot_description_semantic=True
    )
    .planning_pipelines(
      pipelines=["ompl"],
      default_planning_pipeline="ompl"
    )
    .to_moveit_configs()
  )
  
  move_group_configuration = {
    "publish_robot_description_semantic": True,
    "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
    # Note: Wrapping the following values is necessary so that the parameter value can be the empty string
    "capabilities": ParameterValue(
      LaunchConfiguration("capabilities"), value_type=str
    ),
    "disable_capabilities": ParameterValue(
      LaunchConfiguration("disable_capabilities"), value_type=str
    ),
    # Publish the planning scene of the physical robot so that rviz plugin can know actual robot
    "publish_planning_scene": should_publish,
    "publish_geometry_updates": should_publish,
    "publish_state_updates": should_publish,
    "publish_transforms_updates": should_publish,
    "monitor_dynamics": False,
    "use_sim_time": False,
    "pipeline": "ompl"
  }


  move_group_params = [
    move_group_moveit_config.to_dict(),
    move_group_configuration,
  ]

  add_debuggable_node(
    ld,
    package="moveit_ros_move_group",
    executable="move_group",
    output="screen",
    parameters=move_group_params,
    extra_debug_args=["--debug"],
    additional_env={"DISPLAY": os.environ["DISPLAY"]},
  )

  #     # Static TF
  # ld.add_action(
  #   Node(
  #     package="tf2_ros",
  #     executable="static_transform_publisher",
  #     name="static_transform_publisher",
  #     output="log",
  #     arguments=["0.0", "0.0", "0.0", "0.0", "world", "link1"],
  #   )
  # )


  return ld