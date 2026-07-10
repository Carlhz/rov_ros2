#!/bin/bash
# Deploy stereo camera tools to VM
# Run from Windows: bash deploy_to_vm.sh

VM_HOST="172.16.30.0"
VM_USER="carl"
VM_PASS="159357"
REMOTE_DIR="~/stereo_cam_tools"

echo "Deploying stereo camera tools to VM $VM_HOST ..."

# Use python/paramiko based deploy (same pattern as sonar deploy)
python3 - << 'EOF'
import paramiko, os, sys

HOST = "172.16.30.0"
USER = "carl"
PASS = "159357"
REMOTE = "/home/carl/stereo_cam_tools"
LOCAL  = r"D:\Carl_WorkStation\rov_ros2\tools\stereo_cam"

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect(HOST, username=USER, password=PASS, timeout=15)

stdin, stdout, stderr = ssh.exec_command(f"mkdir -p {REMOTE}")
stdout.read()

sftp = ssh.open_sftp()
files = [f for f in os.listdir(LOCAL) if f.endswith(('.py', '.sh'))]
for fname in files:
    local_path  = os.path.join(LOCAL, fname)
    remote_path = f"{REMOTE}/{fname}"
    sftp.put(local_path, remote_path)
    print(f"  Uploaded: {fname}")

sftp.close()

# Install dependencies
print("\nInstalling dependencies on VM...")
deps = "sudo apt-get install -y v4l-utils python3-pip && pip3 install opencv-python pyusb"
stdin, stdout, stderr = ssh.exec_command(deps, timeout=120)
out = stdout.read().decode()
err = stderr.read().decode()
if "error" in out.lower() or err:
    print("Install output:", out[-500:])
    if err: print("ERR:", err[:300])
else:
    print("  Dependencies OK")

# Make scripts executable
ssh.exec_command(f"chmod +x {REMOTE}/*.sh")
stdout.read()

print(f"\nDone. Files in VM: {REMOTE}/")
print("  step1_detect.sh       - run first (detect camera)")
print("  step2_stereo_viewer.py - camera image test")
print("  step3_imu_probe.py    - find IMU XU controls")
print("  step4_stereo_imu.py   - combined viewer")
ssh.close()
EOF
