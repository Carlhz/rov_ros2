#!/bin/bash
echo "=== lsusb -v ===" > /tmp/desc_out.txt
echo "159357" | /usr/bin/sudo -S /usr/bin/lsusb -v -d 1BCF:0B15 >> /tmp/desc_out.txt 2>&1
echo "---" >> /tmp/desc_out.txt
echo "=== lsusb (no root) ===" >> /tmp/desc_out.txt
/usr/bin/lsusb -v -d 1BCF:0B15 >> /tmp/desc_out.txt 2>&1
echo "---" >> /tmp/desc_out.txt
echo "=== sysfs ===" >> /tmp/desc_out.txt
ls -la /sys/bus/usb/devices/3-*/ 2>/dev/null >> /tmp/desc_out.txt
for d in /sys/bus/usb/devices/3-*; do
    vid=$(cat $d/idVendor 2>/dev/null)
    pid=$(cat $d/idProduct 2>/dev/null)
    if [ "$vid" = "1bcf" ] && [ "$pid" = "0b15" ]; then
        echo "=== Device $d ===" >> /tmp/desc_out.txt
        cat $d/descriptors 2>/dev/null | xxd >> /tmp/desc_out.txt 2>&1
        echo "---" >> /tmp/desc_out.txt
        # Show all sub-devices
        for sub in $d/$d:*; do
            if [ -d "$sub" ]; then
                echo "=== Sub: $(basename $sub) ===" >> /tmp/desc_out.txt
                cat $sub/descriptors 2>/dev/null | xxd >> /tmp/desc_out.txt 2>&1
            fi
        done
    fi
done
echo "EXIT:$?" >> /tmp/desc_out.txt
