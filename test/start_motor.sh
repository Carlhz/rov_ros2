#!/bin/bash
source /opt/ros/setup.bash
export ROS_DOMAIN_ID=42
cd /opt/ros/rov_ros2_ws
pkill -f motor_controller.py
sleep 2
nohup python3 -u motor_controller.py > /tmp/motor_controller.log 2>&1 &
echo "motor_controller started, PID=$!"
