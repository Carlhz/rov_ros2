#!/usr/bin/env python3
"""Check VM matplotlib/display environment for 3D sonar view."""

import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("172.16.30.0", username="carl", password="159357", timeout=10)

def run(cmd, timeout=15):
    stdin, stdout, stderr = ssh.exec_command(cmd, timeout=timeout)
    return stdout.read().decode("utf-8", errors="replace") + stderr.read().decode("utf-8", errors="replace")

print("=== 1. matplotlib 是否安装 ===")
print(run("python3 -c 'import matplotlib; print(matplotlib.__version__)' 2>&1"))

print("=== 2. numpy ===")
print(run("python3 -c 'import numpy; print(numpy.__version__)' 2>&1"))

print("=== 3. DISPLAY 环境变量 ===")
print(run("echo DISPLAY=$DISPLAY"))

print("=== 4. Tkinter (matplotlib backend) ===")
print(run("python3 -c 'import tkinter; print(tkinter.TkVersion)' 2>&1"))

print("=== 5. 显示相关包 ===")
print(run("dpkg -l | grep -E 'tk|tkinter|python3-tk' 2>&1 | head -10"))

ssh.close()
