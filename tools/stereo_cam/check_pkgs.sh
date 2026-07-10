#!/bin/bash
dpkg -l *usb* > /tmp/usb_pkgs.txt 2>&1
dpkg -l *libusb* >> /tmp/usb_pkgs.txt 2>&1
python3 -c "import usb" >> /tmp/usb_pkgs.txt 2>&1 && echo "PYUSB_OK" >> /tmp/usb_pkgs.txt || echo "PYUSB_MISSING" >> /tmp/usb_pkgs.txt
echo "DONE" >> /tmp/usb_pkgs.txt
