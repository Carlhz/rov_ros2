#!/usr/bin/env python3
"""Deploy stereo camera tools to VM"""
import paramiko, os

HOST = "172.16.30.0"
USER = "carl"
PASS = "159357"
REMOTE = "/home/carl/stereo_cam_tools"
LOCAL  = r"D:\Carl_WorkStation\rov_ros2\tools\stereo_cam"

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
print(f"Connecting to VM {HOST} ...")
ssh.connect(HOST, username=USER, password=PASS, timeout=15)

stdin, stdout, stderr = ssh.exec_command(f"mkdir -p {REMOTE}")
stdout.read()

sftp = ssh.open_sftp()
files = [f for f in os.listdir(LOCAL) if f.endswith(('.py', '.sh'))]
for fname in sorted(files):
    local_path  = os.path.join(LOCAL, fname)
    remote_path = f"{REMOTE}/{fname}"
    sftp.put(local_path, remote_path)
    print(f"  Uploaded: {fname}")
sftp.close()

# chmod
ssh.exec_command(f"chmod +x {REMOTE}/*.sh {REMOTE}/*.py")

# Install v4l-utils
print("\nChecking v4l-utils ...")
stdin, stdout, stderr = ssh.exec_command("which v4l2-ctl || sudo apt-get install -y v4l-utils 2>&1")
out = stdout.read().decode().strip()
print("  " + (out[:100] if out else "already installed"))

# Check opencv
print("Checking opencv ...")
stdin, stdout, stderr = ssh.exec_command("python3 -c 'import cv2; print(cv2.__version__)' 2>&1")
out = stdout.read().decode().strip()
if out:
    print(f"  opencv: {out}")
else:
    print("  opencv not found, installing ...")
    stdin, stdout, stderr = ssh.exec_command("pip3 install opencv-python 2>&1 | tail -3")
    print("  " + stdout.read().decode().strip())

# Check pyusb
print("Checking pyusb ...")
stdin, stdout, stderr = ssh.exec_command("python3 -c 'import usb.core; print(\"pyusb OK\")' 2>&1")
out = stdout.read().decode().strip()
if "OK" not in out:
    print("  installing pyusb ...")
    stdin, stdout, stderr = ssh.exec_command("pip3 install pyusb 2>&1 | tail -2")
    print("  " + stdout.read().decode().strip())
else:
    print("  pyusb OK")

ssh.close()

print(f"\nDeployed to VM: {REMOTE}/")
print("  Next steps on VM:")
print(f"  cd {REMOTE}")
print("  bash step1_detect.sh        # plug in camera first!")
print("  python3 step2_stereo_viewer.py")
print("  sudo python3 step3_imu_probe.py")
