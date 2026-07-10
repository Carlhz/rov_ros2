#!/usr/bin/env python3
"""Deploy and run ylx_imu_linux.py on VM via paramiko"""

import paramiko
import sys
import os

VM_IP = "172.16.30.0"
VM_USER = "carl"
VM_PASS = "159357"

LOCAL_SCRIPT = os.path.join(os.path.dirname(__file__), "ylx_imu_linux.py")
REMOTE_PATH = "/home/carl/ylx_imu_linux.py"


def run_ssh(client, cmd, timeout=30):
    """Run command and return stdout"""
    print(f"  $ {cmd}")
    stdin, stdout, stderr = client.exec_command(cmd, timeout=timeout)
    out = stdout.read().decode()
    err = stderr.read().decode()
    if err:
        print(f"    stderr: {err[:500]}")
    return out, err


def main():
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    print(f"Connecting to {VM_USER}@{VM_IP}...")
    try:
        client.connect(VM_IP, username=VM_USER, password=VM_PASS, timeout=10)
        print("Connected!")
    except Exception as e:
        print(f"Connection failed: {e}")
        sys.exit(1)

    # Step 1: Check camera
    print("\n--- Step 1: Check camera ---")
    out, _ = run_ssh(client, "lsusb | grep -i '1bcf\|0b15'")
    if not out.strip():
        print("  WARNING: YLX camera (1BCF:0B15) NOT found in lsusb!")
        out2, _ = run_ssh(client, "lsusb")
        print(f"  All USB devices:\n{out2[:1000]}")
    else:
        print(f"  Found: {out.strip()}")

    # Step 2: Check /dev/video
    print("\n--- Step 2: Check /dev/video* ---")
    run_ssh(client, "ls -la /dev/video* 2>/dev/null | head -10")

    # Step 3: Install pyusb
    print("\n--- Step 3: Install pyusb & libusb ---")
    run_ssh(client, "sudo apt-get install -y libusb-1.0-0-dev 2>/dev/null", timeout=60)
    run_ssh(client, "pip3 install pyusb --quiet 2>&1 | tail -3", timeout=30)

    # Step 4: Check pyusb
    print("\n--- Step 4: Verify pyusb ---")
    out, err = run_ssh(client, "python3 -c 'import usb.core; print(\"pyusb OK\")'")
    if "pyusb OK" not in out:
        print("  pyusb NOT working, trying sudo install...")
        run_ssh(client, "sudo pip3 install pyusb --quiet 2>&1 | tail -3", timeout=30)

    # Step 5: Copy script
    print("\n--- Step 5: Copy script ---")
    sftp = client.open_sftp()
    sftp.put(LOCAL_SCRIPT, REMOTE_PATH)
    sftp.chmod(REMOTE_PATH, 0o755)
    sftp.close()
    print(f"  Copied to {REMOTE_PATH}")

    # Step 6: Run with sudo (needed for libusb access)
    print("\n--- Step 6: Run IMU reader ---")
    print("=" * 60)
    stdin, stdout, stderr = client.exec_command(
        f"sudo python3 {REMOTE_PATH}",
        timeout=30
    )
    
    # Read output in real-time
    import select
    import time
    
    # Need to read both stdout and stderr
    stdout.channel.settimeout(10)
    stderr.channel.settimeout(10)
    
    start = time.time()
    while time.time() - start < 15:
        if stdout.channel.recv_ready():
            data = stdout.channel.recv(4096).decode()
            if data:
                print(data, end="", flush=True)
        if stderr.channel.recv_ready():
            data = stderr.channel.recv(4096).decode()
            if data:
                print(data, end="", flush=True)
        if stdout.channel.exit_status_ready():
            break
        time.sleep(0.1)
    
    # Get remaining output
    try:
        rest = stdout.read().decode()
        if rest:
            print(rest)
    except:
        pass
    try:
        rest = stderr.read().decode()
        if rest:
            print(rest)
    except:
        pass

    print("=" * 60)
    print("\nDone!")
    client.close()


if __name__ == "__main__":
    main()
