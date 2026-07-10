#!/usr/bin/env python3
"""Check YLX camera USB speed on VM and compare with Windows"""
import paramiko, sys, time

host = '172.16.30.0'

c = paramiko.SSHClient()
c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect(host, username='carl', password='159357', timeout=15)

# 1. Check USB bus speed via sysfs
cmds = [
    "echo '=== USB Bus Speed (sysfs) ==='",
    """for d in /sys/bus/usb/devices/[0-9]-*; do
    if [ -f "$d/idVendor" ] && [ -f "$d/idProduct" ]; then
        vid=$(cat "$d/idVendor")
        pid=$(cat "$d/idProduct")
        if [ "$vid" = "1bcf" ] && [ "$pid" = "0b15" ]; then
            echo "Device: $d"
            echo "  Vendor: $vid  Product: $pid"
            [ -f "$d/speed" ] && echo "  Speed: $(cat $d/speed)"
            [ -f "$d/version" ] && echo "  USB version: $(cat $d/version)"
            [ -f "$d/bMaxPower" ] && echo "  Max Power: $(cat $d/bMaxPower)"
            [ -f "$d/bConfigurationValue" ] && echo "  Config: $(cat $d/bConfigurationValue)"
            [ -f "$d/bNumInterfaces" ] && echo "  Interfaces: $(cat $d/bNumInterfaces)"
            # Check bus number
            bus=$(basename $(dirname $(readlink -f $d)))
            echo "  Bus: $bus"
        fi
    fi
done""",
    "",
    "echo '=== lsusb -t (tree with speed) ==='",
    "lsusb -t 2>/dev/null || echo 'lsusb -t not supported'",
    "",
    "echo '=== Full USB descriptor dump ==='",
    "echo '159357' | sudo -S lsusb -v -d 1bcf:0b15 2>/dev/null | head -80",
    "",
    "echo '=== dmesg USB speed info ==='",
    "dmesg | grep -i '1bcf\|high.speed\|full.speed\|super.speed\|low.speed' | tail -20",
]

for cmd in cmds:
    _, so, se = c.exec_command(cmd, timeout=15)
    out = so.read().decode()
    err = se.read().decode()
    print(out, end='')
    if err:
        print(f'[stderr]: {err[:200]}', end='')

c.close()
