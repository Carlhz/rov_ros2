#!/usr/bin/env python3
"""Deploy and run IMU bypass script on Linux VM"""
import paramiko
import time

VM_HOST = "172.16.30.0"
VM_USER = "carl"
VM_PASS = "159357"
SCRIPT = "imu_linux_bypass.py"

import os
local_path = os.path.join(os.path.dirname(__file__), SCRIPT)

print(f"Connecting to {VM_HOST}...")
ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect(VM_HOST, username=VM_USER, password=VM_PASS, timeout=10)

# Upload
sftp = ssh.open_sftp()
sftp.putfo(open(local_path, 'rb'), f'/home/{VM_USER}/{SCRIPT}')
sftp.close()
print(f"Uploaded {SCRIPT}")

# Check pyusb
stdin, stdout, stderr = ssh.exec_command("python3 -c 'import usb.core; print(\"OK\")' 2>&1", timeout=10)
out = stdout.read().decode().strip()
if "OK" not in out:
    print("Installing pyusb...")
    stdin, stdout, stderr = ssh.exec_command("pip3 install pyusb 2>&1", timeout=30)
    print(stdout.read().decode())

# Check camera
stdin, stdout, stderr = ssh.exec_command("lsusb -d 1BCF:0B15", timeout=5)
print(f"Camera: {stdout.read().decode().strip()}")

# Check kernel driver binding
stdin, stdout, stderr = ssh.exec_command(
    "for d in /sys/bus/usb/devices/*/driver; do "
    "devname=$(dirname $d); vid=$(cat $devname/idVendor 2>/dev/null); pid=$(cat $devname/idProduct 2>/dev/null); "
    'if [ "$vid" = "1bcf" ] && [ "$pid" = "0b15" ]; then '
    'echo "Driver: $(basename $(readlink $d)) for $(basename $devname)"; '
    "fi; done", timeout=5)
print(f"Driver: {stdout.read().decode().strip() or 'no driver found'}")

# Run the script with 15s timeout
print("\n=== Running IMU bypass probe ===")
channel = ssh.get_transport().open_session()
channel.exec_command(f"cd /home/{VM_USER} && sudo python3 {SCRIPT}")
channel.settimeout(20)

output = b""
start = time.time()
while time.time() - start < 18:
    if channel.recv_ready():
        chunk = channel.recv(8192)
        if chunk:
            output += chunk
            print(chunk.decode(errors='replace'), end='', flush=True)
    if channel.exit_status_ready():
        break
    time.sleep(0.05)

# Drain remaining
time.sleep(0.5)
while channel.recv_ready():
    chunk = channel.recv(4096)
    if chunk:
        output += chunk
        print(chunk.decode(errors='replace'), end='', flush=True)

rc = channel.recv_exit_status()
print(f"\nExit: {rc}")

ssh.close()
