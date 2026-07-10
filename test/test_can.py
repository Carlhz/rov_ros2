import os, sys
os.environ['ROS_DOMAIN_ID'] = '42'

# RK3588 ROS2 Python path
sys.path.insert(0, '/opt/ros/lib/python3.8/site-packages')
sys.path.insert(0, '/opt/ros/rov_ros2_ws')

from motor_controller import build_ctrl_200, build_ctrl_201, rpm_to_cmd

from motor_controller import build_ctrl_200, build_ctrl_201, rpm_to_cmd

# Test with sample values
g = [1200, 1200, 1200, 1200, 0, 1200, 1200, 0]
print("Testing build_ctrl_200...")
frame200 = build_ctrl_200(g)
print("200 OK", len(frame200))
print("Testing build_ctrl_201...")
frame201 = build_ctrl_201(g)
print("201 OK", len(frame201))
print("All tests passed!")
