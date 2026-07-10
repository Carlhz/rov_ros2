#!/bin/bash
echo "159357" | /usr/bin/sudo -S /usr/bin/python3 /home/carl/imu_usb_ctrl.py > /tmp/imu_ctrl_out.txt 2>&1
echo "EXIT:$?" >> /tmp/imu_ctrl_out.txt
