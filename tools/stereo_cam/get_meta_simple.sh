#!/bin/bash
# 简单的元数据捕获: 先启动视频流，再读取元数据
OUT="/mnt/hgfs/CarlWS/rov_ros2/tools/stereo_cam/meta_simple.txt"

{
echo "========================================"
echo "  元数据捕获 (video + meta 同时运行)"
echo "========================================"

# 1. 启动视频流 (后台)
echo ""
echo "[1] 启动 MJPEG 视频流..."
v4l2-ctl -d /dev/video0 --set-fmt-video width=640,height=480,pixelformat=MJPG 2>&1
v4l2-ctl -d /dev/video0 --stream-mmap --stream-count=30 --stream-to=/tmp/vid_stream.bin &
VID_PID=$!
sleep 1
echo "  视频流 PID: $VID_PID"

# 2. 尝试 Python 元数据读取 (现在视频流应该激活了)
echo ""
echo "[2] 读取 metadata..."
/usr/bin/python3 /mnt/hgfs/CarlWS/rov_ros2/tools/stereo_cam/read_uvc_meta.py 2>&1

# 3. 等待视频流结束
wait $VID_PID 2>/dev/null
echo ""
echo "[3] 视频流完成"
ls -la /tmp/vid_stream.bin 2>&1

echo ""
echo "=== DONE ==="
} > "$OUT" 2>&1

echo "Done. See $OUT"
