#!/bin/bash
echo "159357" | sudo -S python3 /home/carl/imu_multi_strategy.py > /tmp/imu_multi_out.txt 2>&1
echo "EXIT:$?"
