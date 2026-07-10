#!/usr/bin/env python3
"""远程执行命令到RK3588"""
import paramiko
import sys

HOST = "172.16.28.82"
USER = "root"
PASS = "tronlong"

def run_cmd(cmd, timeout=10):
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(HOST, username=USER, password=PASS, timeout=5)
    stdin, stdout, stderr = client.exec_command(cmd, timeout=timeout)
    out = stdout.read().decode('utf-8', errors='replace')
    err = stderr.read().decode('utf-8', errors='replace')
    client.close()
    return out, err

if __name__ == "__main__":
    cmd = " ".join(sys.argv[1:])
    if not cmd:
        cmd = "ps aux | head -20"
    out, err = run_cmd(cmd)
    if out:
        print(out, end="")
    if err:
        print(err, end="", file=sys.stderr)
