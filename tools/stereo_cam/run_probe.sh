#!/bin/bash
# Wrapper to run probe and save output
/usr/bin/python3 /home/carl/imu_probe_quick.py > /tmp/imu_out.txt 2>&1
echo "EXIT:$?" >> /tmp/imu_out.txt
