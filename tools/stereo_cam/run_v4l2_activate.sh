#!/bin/bash
echo "159357" | sudo -S python3 /home/carl/imu_v4l2_activate.py > /tmp/imu_v4l2_out.txt 2>&1
echo "EXIT:$?"
