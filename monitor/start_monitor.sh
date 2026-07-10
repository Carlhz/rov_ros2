#!/bin/bash
# ============================================================
# 声纳监控一键启动 (VM Ubuntu 上位机)
# ============================================================
# 用途: 在 VM 上订阅 RK3588 的声纳话题并显示数据
# 前提: RK3588 已启动 sonar_omni_driver
# ============================================================

MONITOR_SCRIPT="$HOME/rov_ros2_ws/monitor/sonar_monitor.py"
QUICK_SCRIPT="$HOME/rov_ros2_ws/monitor/sonar_quick_view.py"

# ---- DDS 配置 (必须与 RK3588 声纳驱动一致) ----
ROS_DOMAIN="0"      # 声纳固定 domain 0（INS 用 domain 42）
ROS_LOCAL="${ROS_LOCALHOST_ONLY:-0}"

case "${1:-monitor}" in
    monitor|full)
        echo "=== 启动全向声纳监控仪表板 ==="
        cd ~/rov_ros2_ws
        source /opt/ros/foxy/setup.bash
        source install/local_setup.bash 2>/dev/null || true
        export ROS_DOMAIN_ID="$ROS_DOMAIN"
        export ROS_LOCALHOST_ONLY="$ROS_LOCAL"
        echo "  DOMAIN=$ROS_DOMAIN  LOCALHOST=$ROS_LOCAL"
        exec python3 "$MONITOR_SCRIPT"
        ;;
    quick)
        echo "=== 快速查看 original 话题 ==="
        echo "  DOMAIN=$ROS_DOMAIN  LOCALHOST=$ROS_LOCAL"
        cd ~/rov_ros2_ws
        source /opt/ros/foxy/setup.bash
        source install/local_setup.bash 2>/dev/null || true
        export ROS_DOMAIN_ID="$ROS_DOMAIN"
        export ROS_LOCALHOST_ONLY="$ROS_LOCAL"
        exec python3 "$QUICK_SCRIPT" --topic original
        ;;
    quick-rigidity)
        echo "=== 快速查看 rigidity 话题 ==="
        echo "  DOMAIN=$ROS_DOMAIN  LOCALHOST=$ROS_LOCAL"
        cd ~/rov_ros2_ws
        source /opt/ros/foxy/setup.bash
        source install/local_setup.bash 2>/dev/null || true
        export ROS_DOMAIN_ID="$ROS_DOMAIN"
        export ROS_LOCALHOST_ONLY="$ROS_LOCAL"
        exec python3 "$QUICK_SCRIPT" --topic rigidity
        ;;
    quick-boundary)
        echo "=== 快速查看 boundary 话题 ==="
        echo "  DOMAIN=$ROS_DOMAIN  LOCALHOST=$ROS_LOCAL"
        cd ~/rov_ros2_ws
        source /opt/ros/foxy/setup.bash
        source install/local_setup.bash 2>/dev/null || true
        export ROS_DOMAIN_ID="$ROS_DOMAIN"
        export ROS_LOCALHOST_ONLY="$ROS_LOCAL"
        exec python3 "$QUICK_SCRIPT" --topic boundary
        ;;
    list|topics)
        echo "=== 列出声纳话题 (DOMAIN=$ROS_DOMAIN) ==="
        cd ~/rov_ros2_ws
        source /opt/ros/foxy/setup.bash
        source install/local_setup.bash 2>/dev/null || true
        export ROS_DOMAIN_ID="$ROS_DOMAIN"
        export ROS_LOCALHOST_ONLY="$ROS_LOCAL"
        echo ""
        echo "话题列表:"
        ros2 topic list 2>/dev/null | grep -i sonar || echo "  (未发现声纳话题——检查: 1)驱动是否运行 2)ROS_DOMAIN_ID 是否匹配)"
        echo ""
        echo "话题详情:"
        ros2 topic info /sonar/omni/original 2>/dev/null || echo "  /sonar/omni/original — 无数据"
        echo ""
        ;;
    echo|raw)
        echo "=== 原始数据 echo (DOMAIN=$ROS_DOMAIN) ==="
        cd ~/rov_ros2_ws
        source /opt/ros/foxy/setup.bash
        source install/local_setup.bash 2>/dev/null || true
        export ROS_DOMAIN_ID="$ROS_DOMAIN"
        export ROS_LOCALHOST_ONLY="$ROS_LOCAL"
        exec ros2 topic echo /sonar/omni/boundary --no-arr
        ;;
    *)
        echo "用法: $0 {monitor|quick|quick-rigidity|quick-boundary|list|echo}"
        echo ""
        echo "  monitor          - 终端仪表板 (彩色,实时)"
        echo "  quick            - 快速查看 original 话题 (文本行)"
        echo "  quick-rigidity   - 快速查看 rigidity 话题"
        echo "  quick-boundary   - 快速查看 boundary 话题"
        echo "  list             - 列出声纳话题/状态"
        echo "  echo             - ros2 topic echo (raw)"
        exit 1
        ;;
esac
