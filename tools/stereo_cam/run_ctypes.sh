#!/bin/bash
# Run with sudo password piped in
echo "159357" | /usr/bin/sudo -S /usr/bin/python3 /home/carl/imu_ctypes_probe.py > /tmp/imu_ctypes_out.txt 2>&1
echo "EXIT:$?" >> /tmp/imu_ctypes_out.txt
