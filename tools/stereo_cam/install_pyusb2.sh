#!/bin/bash
# Install pyusb via apt (no network needed if cached) or pip
echo "=== Trying apt ===" 
sudo apt-get install -y python3-usb 2>&1 | tail -3
echo "---"
echo "=== Checking pip ==="
pip3 install pyusb -i https://pypi.tuna.tsinghua.edu.cn/simple 2>&1 | tail -5
echo "---"
echo "=== Verify ==="
python3 -c "import usb.core; print('PYUSB_OK')" 2>&1
echo "DONE" > /tmp/install_done.txt
