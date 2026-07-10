#!/bin/bash
# ═══════════════════════════════════════════════════════
#  CAN 自动配置脚本 (RK3588)
#  MCP2515 SPI-CAN, can0 @ 500kbps
# ═══════════════════════════════════════════════════════
#
# 用法:
#   ./setup_can.sh              配置并启用 can0
#   ./setup_can.sh status        查看 CAN 状态
#   ./setup_can.sh force         强制重配（即使已 UP）
# ═══════════════════════════════════════════════════════

CMD="${1:-up}"

if [ "$CMD" = "status" ]; then
    echo "=== CAN 状态 ==="
    ip link show can0 2>/dev/null || echo "can0 不存在"
    ip -details link show can0 2>/dev/null | grep -E "can0:|can state|bitrate|CAN" || true
    echo ""
    echo "=== CAN 模块 ==="
    lsmod | grep -E "^can" 2>/dev/null || echo "  can 模块未加载"
    exit 0
fi

# ── 检查 can0 是否已就绪 ─────────────────────
STATE=$(ip link show can0 2>/dev/null | grep -oP 'state \K\w+' || echo "MISSING")
BITRATE=$(ip -details link show can0 2>/dev/null | grep -oP 'bitrate \K\d+' || echo "0")

if [ "$STATE" = "UP" ] && [ "$BITRATE" = "500000" ] && [ "$CMD" != "force" ]; then
    echo "[OK] can0 已配置 @ 500kbps (跳过)"
    exit 0
fi

if [ "$CMD" = "force" ] || [ "$STATE" != "UP" ]; then
    echo "[>>] 配置 can0 @ 500kbps..."
    
    # 1. 先 down（必须，否则修改参数失败）
    ip link set can0 down 2>/dev/null && echo "  [OK] can0 down" || echo "  [--] can0 已 down 或不存在"
    
    # 2. 设置波特率并 up
    if ip link set can0 up type can bitrate 500000 2>/dev/null; then
        echo "  [OK] can0 up type can bitrate 500000"
    else
        echo "  [FAIL] 无法启��� can0"
        echo "  检查: lsmod | grep mcp251x"
        echo "  检查: dmesg | grep -i can"
        exit 1
    fi
    
    # 3. 验证
    sleep 0.2
    NEW_STATE=$(ip link show can0 2>/dev/null | grep -oP 'state \K\w+' || echo "?")
    if [ "$NEW_STATE" = "UP" ]; then
        echo "[OK] 验证: can0 state=$NEW_STATE, bitrate=500000"
    else
        echo "[WARN] can0 state=$NEW_STATE (预期 UP)"
    fi
fi

echo ""
echo "螺旋桨控制: /usr/can_motor_v1.0"
echo "  初始化: /usr/can_motor_v1.0 init"
echo "  前进:   /usr/can_motor_v1.0 move 500"
echo "  上浮:   /usr/can_motor_v1.0 up 500"
echo "  停止:   /usr/can_motor_v1.0 stop"
