import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/javier/puzzlebot_ws/src/install/robot_tf2_broadcaster'