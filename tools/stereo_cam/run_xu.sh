#!/bin/bash
echo "159357" | /usr/bin/sudo -S /usr/bin/python3 /home/carl/imu_xu_activate.py > /tmp/imu_xu_out.txt 2>&1
echo "EXIT:$?" >> /tmp/imu_xu_out.txt
