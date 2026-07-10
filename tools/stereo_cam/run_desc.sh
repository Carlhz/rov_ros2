#!/bin/bash
echo "159357" | /usr/bin/sudo -S /usr/bin/python3 /home/carl/dump_descriptors.py > /tmp/desc_out.txt 2>&1
echo "EXIT:$?" >> /tmp/desc_out.txt
