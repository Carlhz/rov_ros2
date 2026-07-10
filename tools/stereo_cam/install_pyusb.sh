#!/bin/bash
pip3 install pyusb 2>&1 | tail -5
echo "INSTALL_EXIT:$?" > /tmp/install_out.txt
