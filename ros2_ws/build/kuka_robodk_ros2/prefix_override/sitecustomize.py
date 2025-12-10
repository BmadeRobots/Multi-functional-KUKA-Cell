import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/bmade-robotics/ros2_ws/install/kuka_robodk_ros2'
