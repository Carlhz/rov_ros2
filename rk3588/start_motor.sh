#!/bin/bash
source /opt/ros/setup.bash
export ROS_DOMAIN_ID=42
export PYTHONUNBUFFERED=1
exec python3 -u /opt/ros/rov_ros2_ws/motor_controller.py
