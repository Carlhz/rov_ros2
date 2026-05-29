#!/bin/bash
# 快速启动脚本 - Ubuntu VM (上位机)
# 一键启动上位机监控

set -e

WS_DIR="$(cd "$(dirname "$0")/.." && pwd)"

echo "=========================================="
echo "ROV INS 上位机监控 - 快速启动"
echo "=========================================="

# Source ROS2
echo "[1/2] 加载 ROS2 环境..."
source /opt/ros/foxy/setup.bash 2>/dev/null || source /opt/ros/setup.bash
source ${WS_DIR}/install/setup.bash

echo "✓ ROS_DISTRO: $ROS_DISTRO"
echo "✓ ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo ""

# 检查话题
echo "[2/2] 检查 ROS2 话题..."
echo "可用话题:"
ros2 topic list 2>/dev/null || echo "  (暂无可用话题)"
echo ""

# 启动监控
echo "=========================================="
echo "启动 INS 监控节点..."
echo "=========================================="
echo ""
echo "可用命令:"
echo "  start  - 启动 INS 数据输出"
echo "  stop   - 停止 INS 数据输出"
echo "  lat X  - 设置纬度"
echo "  lon X  - 设置经度"
echo "  status - 显示状态"
echo "  quit   - 退出"
echo ""

ros2 launch rov_topside topside.launch.py
