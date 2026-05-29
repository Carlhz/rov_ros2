#!/bin/bash
# RK3588 传感器驱动启动脚本
# 部署路径：/opt/ros/rov_ros2_ws/start_sensors.sh
#
# 用法：
#   ./start_sensors.sh          # 前台启动所有驱动
#   ./start_sensors.sh bg       # 后台启动

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROS_DOMAIN_ID=42
export ROS_DOMAIN_ID

echo "=== ROV 传感器驱动启动 ==="
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo ""

# 检查串口是否存在
check_port() {
    if [ -e "$1" ]; then
        echo "  [OK] $1"
        return 0
    else
        echo "  [--] $1 不存在，跳过"
        return 1
    fi
}

echo "检查串口:"
check_port /dev/ttyS3 || true
check_port /dev/ttyS5 || true
echo ""

run_driver() {
    local name=$1
    local script=$2
    echo "启动 ${name}..."
    if [ "$1" = "bg" ]; then
        python3 "${SCRIPT_DIR}/${script}" &
        echo "  PID: $!"
    else
        python3 "${SCRIPT_DIR}/${script}"
    fi
}

case "${1:-fg}" in
    bg)
        echo "后台模式"
        run_driver "D30深温计" "depth_sensor_driver.py"
        run_driver "SF高度计" "altimeter_driver.py"
        echo ""
        echo "所有驱动已在后台启动。查看日志: tail -f /tmp/rov_sensors*.log"
        wait
        ;;
    fg)
        echo "前台模式 — 在一个终端运行所有传感器"
        echo "提示: 使用 'bg' 参数在后台运行"
        echo "按 Ctrl+C 停止"
        echo ""
        python3 "${SCRIPT_DIR}/depth_sensor_driver.py" &
        PID1=$!
        python3 "${SCRIPT_DIR}/altimeter_driver.py" &
        PID2=$!
        trap "kill $PID1 $PID2 2>/dev/null; exit" INT TERM
        wait
        ;;
    *)
        echo "用法: $0 [fg|bg]"
        ;;
esac
