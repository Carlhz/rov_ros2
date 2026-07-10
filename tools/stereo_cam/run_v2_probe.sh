#!/bin/bash
# 部署并运行 V2 陀螺仪探测 (Raw V4L2)
REMOTE_DIR="/home/carl/stereo_cam_tools"
SCRIPT="ylx_gyro_probe_v2.py"

echo "=== 检查 v4l2-ctl ==="
which v4l2-ctl || sudo apt-get install -y v4l-utils

echo ""
echo "=== 检查摄像头 ==="
v4l2-ctl -d /dev/video0 --list-formats 2>&1 || echo "No /dev/video0"

echo ""
echo "=== 运行探测 ==="
cd $REMOTE_DIR
python3 $SCRIPT
