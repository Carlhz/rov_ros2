#!/bin/bash
# RK3588 传感器驱动启动/停止脚本
# 用法:
#   ./start_sensors.sh bg      后台运行
#   ./start_sensors.sh fg      前台运行 (Ctrl+C 停止)
#   ./start_sensors.sh stop    停止所有传感器驱动
#   ./start_sensors.sh status  查看运行状态

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CMD="${1:-fg}"

source /opt/ros/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

# ===== stop =====
if [ "$CMD" = "stop" ]; then
    echo "=== 停止传感器驱动 ==="
    DEPTH_PIDS=$(pgrep -f "depth_sensor_driver" 2>/dev/null || true)
    ALTI_PIDS=$(pgrep -f "altimeter_driver" 2>/dev/null || true)
    ALL="$DEPTH_PIDS $ALTI_PIDS"
    if [ -z "$(echo $ALL)" ]; then
        echo "  没有运行中的驱动"
    else
        for pid in $ALL; do
            kill "$pid" 2>/dev/null && echo "  已停止 PID=$pid"
        done
        sleep 1
        # 确认死透
        for pid in $ALL; do
            kill -0 "$pid" 2>/dev/null && kill -9 "$pid" && echo "  强制停止 PID=$pid" || true
        done
    fi
    echo "=== 完成 ==="
    exit 0
fi

# ===== status =====
if [ "$CMD" = "status" ]; then
    echo "=== 传感器驱动状态 ==="
    echo "环境: ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
    for port in /dev/ttyS3 /dev/ttyS5; do
        [ -e "$port" ] && echo "  [OK] $port" || echo "  [--] $port 不存在"
    done
    echo "---"
    echo "进程:"
    ps aux | grep -E "depth_sensor_driver|altimeter_driver" | grep -v grep || echo "  无运行中进程"
    echo "---"
    echo "话题:"
    ros2 topic list 2>/dev/null | grep -i rov || echo "  无 /rov/* 话题"
    exit 0
fi

# ===== start (fg/bg) =====
echo "=== ROV 传感器驱动启动 (${CMD}) ==="
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"

echo "检查串口:"
for port in /dev/ttyS3 /dev/ttyS5; do
    [ -e "$port" ] && echo "  [OK] $port" || echo "  [--] $port 不存在"
done

if [ "$CMD" = "bg" ]; then
    python3 "${SCRIPT_DIR}/depth_sensor_driver.py" > /tmp/depth.log 2>&1 &
    echo "  D30深温计 PID=$!"
    python3 "${SCRIPT_DIR}/altimeter_driver.py" > /tmp/alti.log 2>&1 &
    echo "  SF高度计  PID=$!"
    echo "所有驱动后台运行中"
    echo "日志: /tmp/depth.log  /tmp/alti.log"
    echo "停止: ./start_sensors.sh stop"
    wait
else
    echo "前台模式 (Ctrl+C 停止)"
    python3 "${SCRIPT_DIR}/depth_sensor_driver.py" &
    P1=$!
    python3 "${SCRIPT_DIR}/altimeter_driver.py" &
    P2=$!
    trap "kill $P1 $P2 2>/dev/null" INT TERM
    wait
fi
