import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/cobaltboy/ROS2/Bharat-Forge-Inter-IIT-24/install/robot_gazebo'
