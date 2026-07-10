#!/bin/bash
# Step 1: Detect and inspect the stereo UVC camera
# Run this first after plugging in the camera
# Usage: bash step1_detect.sh

echo "============================================"
echo "  UVC Stereo Camera Detection"
echo "============================================"
echo ""

echo "[1] USB devices:"
lsusb
echo ""

echo "[2] Video devices:"
ls -la /dev/video* 2>/dev/null || echo "  No /dev/video* found"
echo ""

echo "[3] Check uvcvideo kernel module:"
lsmod | grep uvc || echo "  uvcvideo not loaded"
echo ""

echo "[4] Install tools if needed:"
which v4l2-ctl > /dev/null 2>&1 || sudo apt-get install -y v4l-utils
which uvcdynctrl > /dev/null 2>&1 || sudo apt-get install -y uvcdynctrl 2>/dev/null || true
echo ""

echo "[5] List all video capture devices:"
v4l2-ctl --list-devices 2>/dev/null || echo "  v4l2-ctl not available"
echo ""

echo "[6] Device capabilities (video0):"
v4l2-ctl -d /dev/video0 --all 2>/dev/null | head -40 || echo "  /dev/video0 not available"
echo ""

echo "[7] Supported formats (video0):"
v4l2-ctl -d /dev/video0 --list-formats-ext 2>/dev/null || echo "  Cannot query formats"
echo ""

echo "[8] Check for Extension Units (IMU/Gyro):"
echo "  Scanning USB descriptor for Extension Units..."
# Try to find XU via uvcvideo sysfs
find /sys/class/video4linux/ -name "*.xu*" 2>/dev/null | head -20 || echo "  No XU found via sysfs"
# Try uvcdynctrl
uvcdynctrl -l 2>/dev/null || echo "  uvcdynctrl not available (install: sudo apt install uvcdynctrl)"
echo ""

echo "[9] USB descriptor dump (first UVC device):"
# Find first UVC device bus/dev numbers
UVCBUS=$(lsusb | grep -i "camera\|video\|stereo\|uvc" | head -1 | awk '{print $2}')
UVCDEV=$(lsusb | grep -i "camera\|video\|stereo\|uvc" | head -1 | awk '{print $4}' | tr -d ':')
if [ -n "$UVCBUS" ]; then
    echo "  Found camera: Bus $UVCBUS Dev $UVCDEV"
    lsusb -v -d $(lsusb | grep -i "camera\|video\|stereo\|uvc" | head -1 | awk '{print $6}') 2>/dev/null \
        | grep -A5 "Extension\|Gyro\|IMU\|XU\|bGUID\|GUID" | head -60
else
    echo "  No camera keyword match — listing all USB devices:"
    lsusb -v 2>/dev/null | grep -B2 -A5 "Extension Unit" | head -80
fi
echo ""

echo "============================================"
echo "  Detection complete. Share output above."
echo "============================================"
