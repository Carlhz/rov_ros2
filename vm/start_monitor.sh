#!/bin/bash
# VM 桌面一键启动 — ROV 综合监控

# 清除 RK3588 交叉编译 sysroot 污染，确保加载本地 x86_64 ROS2 库
# （仅影响当前脚本进程，不影响交叉编译 shell）
unset PYTHONPATH
unset LD_LIBRARY_PATH

cd /home/carl/rov_ros2_ws
source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
python3 monitor/integrated_monitor.py
