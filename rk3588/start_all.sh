#!/bin/bash
# ═══════════════════════════════════════════════════════
#  ROV 传感器一键启动脚本 (RK3588)
#  启动: CAN配置 + INS自动驱动 + ttyS5总线中枢(D30深温计+PWM灯板) + SF高度计
# ═══════════════════════════════════════════════════════
#
# 用法:
#   ./start_all.sh             前台运行 (Ctrl+C 退出)
#   ./start_all.sh bg           后台运行
#   ./start_all.sh stop         停止全部
#   ./start_all.sh status       查看状态
#   ./start_all.sh logs         查看日志
#
# 物理接线:
#   eth0   → 交换机 (192.168.0.99/24，INS通信)
#   can0   → MCP2515 SPI-CAN @ 500kbps (螺旋桨电机)
#   ttyS3  → SF高度计  (RS485, 9600/8N1)
#   ttyS5  → D30深温计 + HCX-8406 PWM水下灯板 (RS485, 19200/8N1)
#   eth0   → DVL H1000 192.168.0.11 (TCP 10000/10001, PD6 ASCII)
# ═══════════════════════════════════════════════════════

# set -e 已移除：避免后台进程退出时静默中止脚本
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CMD="${1:-fg}"
source /opt/ros/setup.bash 2>/dev/null || source /opt/ros/foxy/setup.bash 2>/dev/null || true
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY=0

# ─── stop ─────────────────────────────────────
if [ "$CMD" = "stop" ]; then
    echo "=== 停止传感器驱动 (INS保持运行) ==="
    # 使用 pgrep 查找所有相关进程，兼容 busybox
    for proc in ttyS5_modbus_hub altimeter_driver motor_controller dvl_driver; do
        PIDS=$(pgrep -f "$proc" 2>/dev/null)
        if [ -n "$PIDS" ]; then
            for pid in $PIDS; do
                kill "$pid" 2>/dev/null && echo "  已停止 $proc PID=$pid"
            done
        fi
    done
    sleep 1
    # 强制停止残留进程
    for proc in ttyS5_modbus_hub altimeter_driver motor_controller dvl_driver; do
        PIDS=$(pgrep -f "$proc" 2>/dev/null)
        if [ -n "$PIDS" ]; then
            for pid in $PIDS; do
                kill -9 "$pid" 2>/dev/null && echo "  强制停止 $proc PID=$pid" || true
            done
        fi
    done
    # 也杀掉 TL3588 SDK 冲突进程
    for tlname in rov_3588_node rov_light_rs485_node depth_sensor_driver_node start_tronlong_3588; do
        PIDS=$(pgrep -f "$tlname" 2>/dev/null)
        if [ -n "$PIDS" ]; then
            echo "  停止 TL3588 $tlname PID=$PIDS"
            kill -9 $PIDS 2>/dev/null || true
        fi
    done
    echo "=== 完成 (INS驱动未停止) ==="
    exit 0
fi

# ─── status ───────────────────────────────────
if [ "$CMD" = "status" ]; then
    echo "╔════════════════════════════════════════════╗"
    echo "║     ROV 传感器状态                         ║"
    echo "╚════════════════════════════════════════════╝"
    echo ""
    echo "环境: ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
    echo ""
    echo "── 网络 ──"
    ip addr show eth0 2>/dev/null | grep "inet " | sed 's/^/  /'
    ping -c1 -W1 192.168.0.7 2>/dev/null && echo "  [OK] INS 192.168.0.7 可达" || echo "  [××] INS 192.168.0.7 不可达"
    ping -c1 -W1 192.168.0.5 2>/dev/null && echo "  [OK] 声纳 192.168.0.5 可达" || echo "  [--] 声纳 192.168.0.5 不可达"
    ping -c1 -W1 192.168.0.6 2>/dev/null && echo "  [OK] DVL 192.168.0.6 可达" || echo "  [--] DVL 192.168.0.6 不可达"
    echo ""
    echo "── 串口 ──"
    [ -e /dev/ttyS3 ] && echo "  [OK] /dev/ttyS3 (SF高度计)" || echo "  [--] /dev/ttyS3 不存在"
    [ -e /dev/ttyS5 ] && echo "  [OK] /dev/ttyS5 (D30深温计 + PWM灯板)" || echo "  [--] /dev/ttyS5 不存在"
    echo ""
    echo "── CAN ──"
    "${SCRIPT_DIR}/setup_can.sh" status 2>/dev/null || ip link show can0 2>/dev/null | head -1
    echo ""
    echo "── 进程 ──"
    ps aux 2>/dev/null | grep -E "ins_driver_auto|ttyS5_modbus_hub|altimeter_driver|motor_controller|dvl_driver" | grep -v grep | while read line; do
        echo "  [RUN] $line"
    done || echo "  无运行中进程"
    echo ""
    echo "── ROS2 话题 ──"
    ros2 topic list 2>/dev/null | grep -E "^/ins/|^/rov/" | sed 's/^/  /' || echo "  无 /ins/* 或 /rov/* 话题"
    echo ""
    exit 0
fi

# ─── logs ─────────────────────────────────────
if [ "$CMD" = "logs" ]; then
    echo "=== 传感器日志 ==="
    for f in /tmp/ins_driver.log /tmp/ttyS5_modbus_hub.log /tmp/altimeter.log /tmp/dvl_driver.log; do
        echo ""
        echo "── $(basename $f) ──"
        tail -10 "$f" 2>/dev/null || echo "  (日志文件不存在)"
    done
    exit 0
fi

# ─── 启动前检查 ───────────────────────────────
echo "╔════════════════════════════════════════════╗"
echo "║     ROV 传感器一键启动                     ║"
echo "╚════════════════════════════════════════════╝"
echo ""

# 检查 IP
if ! ip addr show eth0 | grep -q "192.168.0.99"; then
    echo "[警告] eth0 没有 192.168.0.99 地址，尝试添加..."
    ip addr add 192.168.0.99/24 dev eth0 2>/dev/null && echo "[OK] 已添加" || echo "[失败] 请手动执行: sudo ./setup_ip.sh"
fi

# 检查串口
for p in /dev/ttyS3 /dev/ttyS5; do
    [ -e "$p" ] && echo "[OK] $p" || echo "[警告] $p 不存在"
done

# ─── 杀掉 TL3588 SDK 冲突进程 ─────────────────
# TL3588 SDK 的 start_tronlong_3588.sh 会启动以下节点，与我们的 Python 驱动冲突：
#   rov_3588_node         → 占用 ttyS3 读高度计，与 altimeter_driver.py 冲突
#   rov_light_rs485_node  → 占用 ttyS5，与 ttyS5_modbus_hub.py 冲突
#   depth_sensor_driver_node → C++ 深度计节点，与 Python 驱动冲突
echo "[>>] 清理 TL3588 SDK 冲突进程..."
for tlname in rov_3588_node rov_light_rs485_node depth_sensor_driver_node start_tronlong_3588; do
    PIDS=$(pgrep -f "$tlname" 2>/dev/null)
    if [ -n "$PIDS" ]; then
        echo "     杀掉 $tlname (PID: $PIDS)"
        kill -9 $PIDS 2>/dev/null
    fi
done
sleep 1

# ─── 配置 CAN ────────────────────────────────
echo "[>>] 配置 can0 (MCP2515 @ 500kbps)..."
"${SCRIPT_DIR}/setup_can.sh"
echo ""

echo "启动驱动(ROS_DOMAIN_ID=${ROS_DOMAIN_ID})..."
echo ""

# ─── 启动电机控制器 ───────────────────────────
if pgrep -f "motor_controller" > /dev/null 2>&1; then
    echo "[--] 电机控制器已在运行，跳过"
else
    echo "[>>] 启动 ROV 电机控制器 (订阅 /rov/cmd_vel)..."
    nohup python3 -u "${SCRIPT_DIR}/motor_controller.py" >> /tmp/motor_controller.log 2>&1 &
    echo "     PID=$!"
fi

# ─── 启动 INS ─────────────────────────────────
if pgrep -f "ins_driver_auto" > /dev/null 2>&1; then
    echo "[--] INS驱动已在运行，跳过"
else
    echo "[>>] 启动 INS 自动驱动..."
    python3 "${SCRIPT_DIR}/ins_driver_auto.py" > /tmp/ins_driver.log 2>&1 &
    echo "     PID=$!"
fi

# ─── 启动 ttyS5 总线中枢（D30 深温计 + HCX-8406 PWM 水下灯） ───
# 先杀掉 TL3588 SDK 自带的 C++ 深度计节点（会占用串口且不是有效ROS2节点）
pkill -f "depth_sensor_driver_node" 2>/dev/null
sleep 0.5
if pgrep -f "ttyS5_modbus_hub.py" > /dev/null 2>&1; then
    echo "[--] ttyS5 Modbus 中枢已在运行，跳过"
else
    echo "[>>] 启动 ttyS5 Modbus 中枢 (/dev/ttyS5)..."
    export TTY_S5_PORT=/dev/ttyS5
    python3 "${SCRIPT_DIR}/sensors/ttyS5_modbus_hub.py" > /tmp/ttyS5_modbus_hub.log 2>&1 &
    echo "     PID=$!"
fi

# ─── 启动高度计 ───────────────────────────────
if pgrep -f "altimeter_driver" > /dev/null 2>&1; then
    echo "[--] SF高度计已在运行，跳过"
else
    echo "[>>] 启动 SF 高度计 (/dev/ttyS3)..."
    export ALTI_PORT=/dev/ttyS3
    python3 "${SCRIPT_DIR}/sensors/altimeter_driver.py" > /tmp/altimeter.log 2>&1 &
    echo "     PID=$!"
fi

# ─── 启动 DVL ─────────────────────────────────
if pgrep -f "dvl_driver" > /dev/null 2>&1; then
    echo "[--] DVL驱动已在运行，跳过"
else
    echo "[>>] 启动 H1000 DVL 驱动 (192.168.0.11:10000/10001 PD6)..."
    if ping -c1 -W1 192.168.0.11 > /dev/null 2>&1; then
        python3 "${SCRIPT_DIR}/dvl_driver.py" --ip 192.168.0.11 > /tmp/dvl_driver.log 2>&1 &
        echo "     PID=$!"
    else
        echo "     [警告] DVL 192.168.0.11 不可达，跳过启动"
    fi
fi

echo ""
echo "╔════════════════════════════════════════════╗"
echo "║  INS 协议 (INS400使用说明 V6):            ║"
echo "║  上电→监控→输入位置→启动→粗对准→精对准→导航║"
echo "║  默认位置: 深圳(22.73N 113.54E)          ║"
echo "║  如需修改: ins_driver_auto.py --lat --lon  ║"
echo "║  保持静止！预计 3-10 分钟完成对准          ║"
echo "╚════════════════════════════════════════════╝"
echo ""
echo "日志文件:"
echo "  电机:   /tmp/motor_controller.log"
echo "  INS:    /tmp/ins_driver.log"
echo "  ttyS5:  /tmp/ttyS5_modbus_hub.log"
echo "  高度计: /tmp/altimeter.log"
echo "  DVL:    /tmp/dvl_driver.log"
echo ""
echo "停止: $0 stop    |  状态: $0 status"
echo "日志: $0 logs     |  监控: (VM端) integrated_monitor.py"
echo ""
echo "螺旋桨电机:"
echo "  /usr/can_motor_v1.0 init    上电初始化"
echo "  /usr/can_motor_v1.0 status  读取反馈"
echo "  /usr/can_motor_v1.0 stop    全部停止"
echo ""

# ─── 前台模式等待 ─────────────────────────────
if [ "$CMD" = "bg" ]; then
    echo "所有驱动已在后台启动"
else
    echo "前台模式运行中 (Ctrl+C 退出)"
    echo "============================================"
    # 前台模式：实时显示各日志
    tail -f /tmp/motor_controller.log /tmp/ins_driver.log /tmp/ttyS5_modbus_hub.log /tmp/altimeter.log 2>/dev/null
fi
