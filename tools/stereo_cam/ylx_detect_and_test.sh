#!/bin/bash
# ============================================================
#  YLX 双目摄像头 + 陀螺仪 一键检测脚本
#  适用: Sunplus 1bcf:0b15  YLX CAMERA (USB3.0 UVC)
#  在 VM 桌面终端里运行: bash ylx_detect_and_test.sh
# ============================================================

set -e
echo ""
echo "=============================================="
echo "  YLX 双目摄像头 + 陀螺仪 快速验证"
echo "=============================================="
echo ""

# ---- 1. 确认设备 ----
echo "[1] USB 设备列表:"
lsusb
echo ""

echo "[2] 确认 YLX CAMERA (1bcf:0b15):"
if lsusb | grep -i "1bcf:0b15"; then
    echo "  ✓ YLX CAMERA 已识别"
else
    echo "  ✗ 未找到 YLX CAMERA，请检查 USB 连接"
    exit 1
fi
echo ""

# ---- 2. 检查 /dev/video 节点 ----
echo "[3] Video 设备节点:"
ls -la /dev/video* 2>/dev/null || echo "  无 /dev/video* 设备"
echo ""

# ---- 3. 安装必要工具 ----
echo "[4] 安装/确认 v4l-utils..."
if ! which v4l2-ctl > /dev/null 2>&1; then
    sudo apt-get install -y v4l-utils
fi
echo "  v4l2-ctl: $(which v4l2-ctl)"
echo ""

# ---- 4. 列出所有摄像头 ----
echo "[5] 所有视频设备:"
v4l2-ctl --list-devices 2>/dev/null || echo "  (无可用设备)"
echo ""

# ---- 5. 详细查询每个设备 ----
for VDEV in $(ls /dev/video* 2>/dev/null); do
    echo "[设备 $VDEV]"
    v4l2-ctl -d $VDEV --all 2>/dev/null | grep -E "Driver|Card|Bus|Format|Width|Height|Pixel" | head -10
    echo "  支持格式:"
    v4l2-ctl -d $VDEV --list-formats 2>/dev/null | head -8
    echo ""
done

# ---- 6. USB 描述符 — 寻找 Extension Unit ----
echo "[6] USB 描述符 Extension Unit (陀螺仪通道):"
echo "  需要 root，扫描 bDescriptorSubtype=0x06 (Extension Unit)..."
sudo lsusb -v -d 1bcf:0b15 2>/dev/null | grep -A 20 "bDescriptorSubtype.*6\|Extension Unit\|bUnitID\|bNumControls\|wIndex\|bGUID" | head -60 || \
    echo "  (需要 root 或设备未响应)"
echo ""

# ---- 7. 获取完整描述符 ----
echo "[7] 完整 UVC 描述符 (陀螺仪 Extension Unit 查找):"
sudo lsusb -v -d 1bcf:0b15 2>/dev/null | grep -E "bDescriptor|bUnit|bNumControls|bNrInPins|bSourceID|iExtension|GUID" | head -40 || \
    echo "  (无法获取，确认摄像头已接好)"
echo ""

# ---- 8. 检查是否有多个接口 (陀螺仪可能在独立接口) ----
echo "[8] UVC 接口列表:"
sudo lsusb -v -d 1bcf:0b15 2>/dev/null | grep -E "bInterfaceClass|bInterfaceSubClass|bInterfaceProtocol|bInterfaceNumber|iInterface" | head -30 || true
echo ""

# ---- 9. 查 sysfs 获取 XU 信息 ----
echo "[9] sysfs UVC 控制节点:"
find /sys/class/video4linux/ -name "control_mapping" 2>/dev/null | head -10
find /sys/bus/usb/devices/ -name "*1bcf*0b15*" 2>/dev/null | head -5 || true
ls /sys/class/video4linux/ 2>/dev/null
echo ""

# ---- 10. 安装 python 依赖 ----
echo "[10] 安装 Python 依赖..."
pip3 install pyusb opencv-python 2>/dev/null | tail -3 || true
python3 -c "import cv2; print('  opencv:', cv2.__version__)" 2>/dev/null || echo "  opencv: not available"
python3 -c "import usb; print('  pyusb: OK')" 2>/dev/null || echo "  pyusb: not installed"
echo ""

echo "=============================================="
echo "  检测完成！请把以上输出截图给上位机。"
echo "  下一步根据 Extension Unit 信息解析陀螺仪。"
echo "=============================================="
