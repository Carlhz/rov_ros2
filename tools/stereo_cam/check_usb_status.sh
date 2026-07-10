#!/bin/bash
echo "=== lsusb ==="
lsusb
echo ""
echo "=== /dev/video* ==="
ls -la /dev/video* 2>&1
echo ""
echo "=== uvcvideo status ==="
lsmod | grep uvc 2>&1
echo ""
echo "=== USB devices ==="
ls /sys/bus/usb/devices/ 2>&1
echo ""
echo "=== 3-2 status ==="
ls -la /sys/bus/usb/devices/3-2/ 2>&1
echo ""
echo "=== 3-2:1.0 driver ==="
readlink /sys/bus/usb/devices/3-2:1.0/driver 2>&1
echo "EXIT:0"
