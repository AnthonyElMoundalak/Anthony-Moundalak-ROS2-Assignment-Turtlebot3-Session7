import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/anthonyubuntu/Desktop/inmind/session7/Anthony-Moundalak-ROS2-Assignment-Turtlebot3-Session7/ros2_ws/install/turtlebot_navigation'
