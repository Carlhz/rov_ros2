#!/usr/bin/env python3
"""Deploy and run IMU reader - no pip needed, pure ctypes/libusb"""
import paramiko
import sys
import time

VM_IP = "172.16.30.0"
VM_USER = "carl"
VM_PASS = "159357"
LOCAL = r"D:\Carl_WorkStation\rov_ros2\tools\stereo_cam\ylx_imu_linux.py"
REMOTE = "/home/carl/ylx_imu_linux.py"

c = paramiko.SSHClient()
c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
print(f"Connecting to {VM_USER}@{VM_IP}...")
c.connect(VM_IP, username=VM_USER, password=VM_PASS, timeout=15)
print("Connected!")

# Upload script
print("\n--- Upload ---")
sftp = c.open_sftp()
sftp.put(LOCAL, REMOTE)
sftp.chmod(REMOTE, 0o755)
sftp.close()
print("Uploaded!")

# Check libusb
print("\n--- Check libusb ---")
_, so, _ = c.exec_command("ldconfig -p | grep libusb-1.0")
print(so.read().decode().strip())

# Run the reader
print("\n" + "=" * 60)
full_cmd = f"echo '{VM_PASS}' | sudo -S python3 {REMOTE} 2>&1"
print("Running IMU reader... (will read for ~15 sec)")
print("=" * 60)

channel = c.get_transport().open_session()
channel.get_pty()
channel.exec_command(f"/bin/bash -c '{full_cmd}'")

start = time.time()
while time.time() - start < 20:
    if channel.recv_ready():
        data = channel.recv(4096)
        if not data:
            break
        sys.stdout.write(data.decode("utf-8", errors="replace"))
        sys.stdout.flush()
    if channel.exit_status_ready():
        break
    time.sleep(0.1)

# Read remaining
try:
    while channel.recv_ready():
        d = channel.recv(4096)
        if d:
            sys.stdout.write(d.decode("utf-8", errors="replace"))
except:
    pass

exit_code = channel.recv_exit_status()
print(f"\nExit code: {exit_code}")
c.close()
