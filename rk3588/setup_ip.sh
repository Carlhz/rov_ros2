#!/bin/bash
# RK3588 eth0 辅助 IP 配置 — 添加 192.168.0.99/24 用于 INS 通信
# 一次性执行：sudo ./setup_ip.sh
# 立即生效 + 持久化（netplan 或 rc.local）

set -e

INS_SUBNET="192.168.0.99/24"
INTERFACE="eth0"

echo "=== RK3588 eth0 辅助 IP 配置 ==="
echo "接口: ${INTERFACE}"
echo "目标: ${INS_SUBNET}"
echo ""

# 1. 立即生效（临时）
echo ">>> 1. 添加 IP（立即生效）..."
if ip addr show ${INTERFACE} | grep -q "192.168.0.99"; then
    echo "    192.168.0.99 已存在，跳过"
else
    ip addr add ${INS_SUBNET} dev ${INTERFACE}
    echo "    已添加 ${INS_SUBNET} 到 ${INTERFACE}"
fi

# 2. 持久化 — 优先用 netplan
NETPLAN_FILE="/etc/netplan/01-netcfg.yaml"
if [ -f "${NETPLAN_FILE}" ]; then
    echo ""
    echo ">>> 2. 持久化（netplan）..."
    if grep -q "192.168.0.99" "${NETPLAN_FILE}"; then
        echo "    192.168.0.99 已在 netplan 中，跳过"
    else
        # 在当前 eth0 配置中添加辅助 IP
        cp "${NETPLAN_FILE}" "${NETPLAN_FILE}.bak.$(date +%Y%m%d)"
        cat > "${NETPLAN_FILE}" << 'NETPLAN_EOF'
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      dhcp4: yes
      addresses:
        - 192.168.0.99/24
      optional: true
NETPLAN_EOF
        echo "    已更新 ${NETPLAN_FILE}"
        echo "    备份: ${NETPLAN_FILE}.bak.$(date +%Y%m%d)"
        echo ""
        echo "    执行: netplan apply"
        netplan apply 2>/dev/null || echo "    (请手动执行: sudo netplan apply)"
    fi
else
    # 用 rc.local 作为备选
    echo ""
    echo ">>> 2. 持久化（rc.local）..."
    RC_FILE="/etc/rc.local"
    CMD="ip addr add ${INS_SUBNET} dev ${INTERFACE}"
    if [ -f "${RC_FILE}" ] && grep -q "192.168.0.99" "${RC_FILE}"; then
        echo "    已在 rc.local 中，跳过"
    else
        if [ ! -f "${RC_FILE}" ]; then
            echo "#!/bin/sh -e" > "${RC_FILE}"
            chmod +x "${RC_FILE}"
        fi
        echo "${CMD}" >> "${RC_FILE}"
        echo "    已追加到 ${RC_FILE}"
    fi
fi

# 3. 验证
echo ""
echo "=== 验证 ==="
ip addr show ${INTERFACE} | grep -E "inet " | head -5
echo ""
echo "=== 完成 ==="
echo "INS 通信 IP 192.168.0.99/24 已配置"
