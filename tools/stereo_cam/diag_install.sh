#!/bin/bash
echo "=== pip list ===" > /tmp/py_check.txt
pip3 list 2>/dev/null | grep -i usb >> /tmp/py_check.txt 2>&1
echo "---" >> /tmp/py_check.txt
echo "=== apt python3-usb ===" >> /tmp/py_check.txt
dpkg -l python3-usb 2>/dev/null >> /tmp/py_check.txt 2>&1
echo "---" >> /tmp/py_check.txt
echo "=== python import test ===" >> /tmp/py_check.txt
python3 -c "import usb; print('usb OK')" >> /tmp/py_check.txt 2>&1
python3 -c "import usb.core; print('usb.core OK')" >> /tmp/py_check.txt 2>&1
echo "---" >> /tmp/py_check.txt
echo "=== pip install (direct) ===" >> /tmp/py_check.txt
pip3 install --user pyusb 2>>/tmp/py_check.txt
echo "EXIT:$?" >> /tmp/py_check.txt
python3 -c "import usb.core; print('FINAL OK')" >> /tmp/py_check.txt 2>&1
