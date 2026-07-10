#!/bin/bash
echo "159357" | sudo -S python3 /home/carl/imu_scan_all_eps.py > /tmp/imu_scan_out.txt 2>&1
echo "EXIT:$?"
