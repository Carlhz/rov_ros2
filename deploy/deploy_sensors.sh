#!/bin/bash
# 传感器驱动部署脚本 - 从 Windows 工作站部署到 RK3588
# 用法：在 Git Bash 中运行 bash deploy_sensors.sh

set -e

RK3588_IP="172.16.28.82"
RK3588_USER="root"
RK3588_DEST="/opt/ros/rov_ros2_ws/"

echo "=== 部署传感器驱动到 RK3588 ==="
echo "目标: ${RK3588_USER}@${RK3588_IP}:${RK3588_DEST}"
echo ""

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

FILES=(
    "sensors/depth_sensor_driver.py"
    "sensors/altimeter_driver.py"
    "rk3588/start_sensors.sh"
)

for f in "${FILES[@]}"; do
    src="${PROJECT_DIR}/${f}"
    if [ -f "$src" ]; then
        echo "上传: ${f}"
        scp "$src" "${RK3588_USER}@${RK3588_IP}:${RK3588_DEST}"
    else
        echo "跳过: ${f} (文件不存在)"
    fi
done

echo ""
echo "设置可执行权限..."
ssh "${RK3588_USER}@${RK3588_IP}" "chmod +x ${RK3588_DEST}start_sensors.sh ${RK3588_DEST}depth_sensor_driver.py ${RK3588_DEST}altimeter_driver.py"

echo ""
echo "=== 部署完成 ==="
echo ""
echo "在 RK3588 上启动传感器:"
echo "  ssh ${RK3588_USER}@${RK3588_IP}"
echo "  cd ${RK3588_DEST}"
echo "  ./start_sensors.sh bg"
echo ""
echo "在 VM 上查看传感器数据:"
echo "  cd ~/rov_ros2_ws/"
echo "  python3 vm/sensor_monitor.py"
