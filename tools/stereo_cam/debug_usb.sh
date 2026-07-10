#!/bin/bash
OUT="/mnt/hgfs/CarlWS/rov_ros2/tools/stereo_cam/debug_result.txt"

{
echo "=== debugfs ==="
mount | grep debugfs
sudo ls /sys/kernel/debug/usb/ 2>&1
sudo ls /sys/kernel/debug/usb/uvcvideo/ 2>&1

echo ""
echo "=== uvcvideo trace ==="
cat /sys/module/uvcvideo/parameters/trace 2>&1
# Try to enable all trace
sudo bash -c 'echo 0xffff > /sys/module/uvcvideo/parameters/trace' 2>&1

# Trigger some activity - capture one frame
echo ""
echo "=== Trigger camera activity ==="
v4l2-ctl -d /dev/video0 --set-fmt-video width=640,height=480,pixelformat=MJPG 2>&1
v4l2-ctl -d /dev/video0 --stream-mmap --stream-count=5 --stream-to=/dev/null 2>&1

echo ""
echo "=== dmesg ==="
dmesg | tail -30

# Reset trace
sudo bash -c 'echo 0 > /sys/module/uvcvideo/parameters/trace' 2>&1
} > "$OUT" 2>&1

echo "Done"
