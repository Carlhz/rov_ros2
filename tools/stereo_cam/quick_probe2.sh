#!/bin/bash
# Quick probe 2: capture event5/hidraw while video is streaming
OUT="/mnt/hgfs/CarlWS/rov_ros2/tools/stereo_cam/probe_result2.txt"

{
echo "========================================"
echo "  PROBE 2: 在视频流运行时读取 IMU 数据"
echo "========================================"

# 1. List event5 capabilities
echo ""
echo "=== event5 capabilities ==="
evtest --info /dev/input/event5 2>&1 || echo "evtest not available"

# 2. Start MJPEG streaming from /dev/video0 in background
echo ""
echo "=== Starting video stream on /dev/video0 ==="
v4l2-ctl -d /dev/video0 --set-fmt-video width=640,height=480,pixelformat=MJPG 2>&1
v4l2-ctl -d /dev/video0 --stream-mmap --stream-count=20 --stream-to=/tmp/test_stream.bin &
STREAM_PID=$!
sleep 1

# 3. While streaming, read event5
echo ""
echo "=== Reading /dev/input/event5 while streaming ==="
timeout 2 dd if=/dev/input/event5 bs=24 count=50 2>/dev/null | xxd | head -60

# 4. Try again with hidraw
echo ""
echo "=== Reading /dev/hidraw0 while streaming ==="
timeout 2 dd if=/dev/hidraw0 bs=16 count=50 2>/dev/null | xxd | head -60

# 5. Try /dev/video1 metadata while streaming
echo ""
echo "=== Reading /dev/video1 while streaming ==="
v4l2-ctl -d /dev/video1 --set-fmt-video pixelformat=UVCH 2>&1
v4l2-ctl -d /dev/video1 --stream-mmap --stream-count=5 --stream-to=/tmp/metadata_test.bin 2>&1 &
META_PID=$!
sleep 2
kill $META_PID 2>/dev/null
if [ -f /tmp/metadata_test.bin ]; then
    xxd /tmp/metadata_test.bin | head -40
    echo "Metadata file size: $(stat -c%s /tmp/metadata_test.bin) bytes"
fi

# Wait for stream to finish
wait $STREAM_PID 2>/dev/null

echo ""
echo "=== Stream output ==="
ls -la /tmp/test_stream.bin 2>&1

echo ""
echo "=== DONE ==="
} > "$OUT" 2>&1

echo "Probe done. See $OUT"
