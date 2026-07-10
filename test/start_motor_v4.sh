#!/bin/bash
pkill -9 -f motor_controller
cd /opt/ros/rov_ros2_ws
source /opt/ros/setup.bash
export ROS_DOMAIN_ID=42
python3 -u motor_controller.py > /tmp/motor_controller.log 2>&1 &
echo $!
