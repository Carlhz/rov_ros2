#!/bin/bash
echo "159357" | /usr/bin/sudo -S /usr/bin/python3 /home/carl/imu_unbind_read.py > /tmp/imu_unbind_out.txt 2>&1
echo "EXIT:$?" >> /tmp/imu_unbind_out.txt
