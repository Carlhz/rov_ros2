#!/bin/bash
# Quick install via apt
echo "=== apt install python3-usb ===" 
sudo apt-get install -y python3-usb 2>&1 | tail -5
echo "EXIT:$?" > /tmp/apt_result.txt
