#!/bin/bash
# Quick probe script to run inside VM via shared folder
OUT="/mnt/hgfs/CarlWS/rov_ros2/tools/stereo_cam/probe_result.txt"

{
echo "=== INPUT DEVICES ==="
ls -la /dev/input/by-id/ 2>&1
ls -la /dev/input/by-path/ 2>&1

echo ""
echo "=== INPUT DEVICE INFO ==="
for dev in /dev/input/event*; do
  echo "--- $dev ---"
  udevadm info --query=all --name=$dev 2>&1 | head -5
done

echo ""
echo "=== HIDRAW ==="
ls -la /dev/hidraw* 2>&1

echo ""
echo "=== EVTEST CAPTURE (2 sec) ==="
# Read raw binary from event5 for 2 seconds
timeout 2 dd if=/dev/input/event5 bs=24 count=50 2>/dev/null | xxd | head -60

echo ""
echo "=== HIDRAW CAPTURE (2 sec) ==="
timeout 2 dd if=/dev/hidraw0 bs=16 count=50 2>/dev/null | xxd | head -60

echo ""
echo "=== DONE ==="
} > "$OUT" 2>&1

echo "Probe done. See $OUT"
