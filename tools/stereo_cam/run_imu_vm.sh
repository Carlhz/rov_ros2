#!/bin/bash
# Run on VM via shared folder or vmrun
# This script installs deps and runs the IMU reader

set -e

echo "=== YLX IMU Linux Runner ==="
echo "Checking camera..."
lsusb | grep -i "1bcf\|0b15" || echo "WARNING: Camera not found!"

echo ""
echo "Checking /dev/video..."
ls /dev/video* 2>/dev/null || echo "No /dev/video devices"

echo ""
echo "Installing dependencies..."
# Install libusb if needed
dpkg -l | grep -q libusb-1.0-0-dev || sudo apt-get install -y libusb-1.0-0-dev

# Install pyusb
pip3 show pyusb >/dev/null 2>&1 || sudo pip3 install pyusb

echo ""
echo "Verifying pyusb..."
python3 -c "import usb.core; d=usb.core.find(idVendor=0x1BCF,idProduct=0x0B15); print(f'Device found: {d}')"

echo ""
echo "Running IMU reader..."
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
sudo python3 "$SCRIPT_DIR/ylx_imu_linux.py"
