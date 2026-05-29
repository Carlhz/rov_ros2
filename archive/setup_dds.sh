#!/bin/bash
# DDS 多机通信配置脚本
# 在 Ubuntu VM (上位机) 上运行

set -e

echo "=========================================="
echo "ROS2 Foxy DDS 多机通信配置"
echo "=========================================="

# 检查参数
if [ $# -lt 1 ]; then
    echo "用法: $0 <RK3588_IP> [ROS_DOMAIN_ID]"
    echo "示例: $0 172.16.28.82 42"
    exit 1
fi

RK3588_IP=$1
ROS_DOMAIN_ID=${2:-42}

echo "RK3588 IP: ${RK3588_IP}"
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
echo ""

# 添加到 .bashrc
echo "[1/3] 配置环境变量..."

BASHRC_ADDITION="
# ROV ROS2 DDS 配置
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
export ROS_LOCALHOST_ONLY=0
"

if ! grep -q "ROV ROS2 DDS 配置" ~/.bashrc; then
    echo "${BASHRC_ADDITION}" >> ~/.bashrc
    echo "✓ 已添加到 ~/.bashrc"
else
    echo "✓ 配置已存在"
fi

# 立即生效
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
export ROS_LOCALHOST_ONLY=0

# 配置防火墙
echo ""
echo "[2/3] 配置防火墙..."
if command -v ufw &> /dev/null; then
    # DDS 默认端口范围
    sudo ufw allow 7400:7500/udp comment 'ROS2 DDS'
    sudo ufw allow 7400:7500/tcp comment 'ROS2 DDS'
    echo "✓ UFW 防火墙已配置"
else
    echo "⚠ UFW 未安装，跳过防火墙配置"
fi

# 测试连接
echo ""
echo "[3/3] 测试与 RK3588 的连接..."
if ping -c 1 -W 2 ${RK3588_IP} > /dev/null 2>&1; then
    echo "✓ 网络连通: ${RK3588_IP}"
else
    echo "⚠ 无法 ping 通 ${RK3588_IP}"
    echo "  请检查网络连接"
fi

echo ""
echo "=========================================="
echo "DDS 配置完成！"
echo "=========================================="
echo ""
echo "环境变量:"
echo "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "  ROS_LOCALHOST_ONLY=0"
echo ""
echo "使用方法:"
echo "  1. 重新打开终端，或运行: source ~/.bashrc"
echo "  2. 启动上位机监控: ros2 launch rov_topside topside.launch.py"
echo ""
echo "查看话题列表:"
echo "  ros2 topic list"
echo ""
echo "查看 INS 数据:"
echo "  ros2 topic echo /ins/data"
echo ""
echo "发送启动命令:"
echo "  ros2 topic pub /ins/command rov_ins_interfaces/msg/InsCommand '{command: \"start\"}'"
echo ""
