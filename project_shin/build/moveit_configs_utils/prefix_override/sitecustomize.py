import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/user/aiot_robot_project/project_shin/install/moveit_configs_utils'
