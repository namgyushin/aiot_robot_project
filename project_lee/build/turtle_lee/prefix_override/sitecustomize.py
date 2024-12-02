import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/lmc/aiot_robot_project/project_lee/install/turtle_lee'
