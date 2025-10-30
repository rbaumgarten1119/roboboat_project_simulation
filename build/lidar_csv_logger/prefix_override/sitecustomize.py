import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/phoenixautonomy/ros2_ws/install/lidar_csv_logger'
