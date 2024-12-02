import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/test/aiot_robot_project/project_byun/install/turtle_byun'
