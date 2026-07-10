#!/usr/bin/env python3
"""Full USB descriptor and speed comparison"""
import paramiko, time

host = '172.16.30.0'

c = paramiko.SSHClient()
c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect(host, username='carl', password='159357', timeout=15)

# Full descriptor dump focused on XU#4
_, so, se = c.exec_command(
    "echo '159357' | sudo -S lsusb -v -d 1bcf:0b15 2>/dev/null", timeout=20)
full = so.read().decode()
err = se.read().decode()

# Find XU#4 section
for i, line in enumerate(full.split('\n')):
    if 'Extension' in line or '63610682' in line.lower() or 'xu' in line.lower():
        # Print context around match
        lines = full.split('\n')
        start = max(0, i-2)
        end = min(len(lines), i+20)
        for j in range(start, end):
            marker = '>>>' if j == i else '   '
            print(f'{marker} {lines[j]}')
        print('---')

# Also check if camera supports USB3 descriptor (BOS descriptor)
_, so, _ = c.exec_command(
    "echo '159357' | sudo -S lsusb -v -d 1bcf:0b15 2>/dev/null | grep -A5 'Binary Object Store\\|SuperSpeed\\|bcdUSB'", timeout=10)
print('\n=== BOS/SuperSpeed check ===')
print(so.read().decode())

# Compare: is the camera on a USB3 physical port?
_, so, _ = c.exec_command(
    "ls /sys/bus/usb/devices/3-2/power/ 2>/dev/null; "
    "cat /sys/bus/usb/devices/3-2/rx_lanes /sys/bus/usb/devices/3-2/tx_lanes 2>/dev/null; "
    "echo '---'; "
    "ls -la /sys/bus/usb/devices/3-2/ 2>/dev/null | grep -i 'lane\\|speed'", timeout=10)
print('\n=== Physical port details ===')
print(so.read().decode())

c.close()
