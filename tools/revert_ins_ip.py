"""Revert INS IP back to 192.168.0.7 on RK3588"""
import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("172.16.28.82", username="root", password="159357", timeout=10)

def run(cmd, timeout=15):
    stdin, stdout, stderr = ssh.exec_command(cmd, timeout=timeout)
    return stdout.read().decode("utf-8", errors="replace") + stderr.read().decode("utf-8", errors="replace")

installed = "/opt/ros/rov_ros2_ws/install"

# Revert INS files on RK3588 (0.5 -> 0.7), only INS-related files
targets = [
    f"{installed}/rov_ins_driver/lib/python3*/site-packages/ins_driver_full.py",
    f"{installed}/rov_ins_driver/lib/rov_ins_driver/ins_driver_node.py",
    f"{installed}/rov_ins_driver/share/rov_ins_driver/config/ins_driver.yaml",
]

for pat in targets:
    # Find actual file
    files = run(f"ls {pat} 2>/dev/null").strip().split("\n")
    for f in files:
        if not f: continue
        # Check if it contains 192.168.0.5
        content = run(f"grep -l '192.168.0.5' {f} 2>/dev/null").strip()
        if content:
            print(f"Reverting: {f}")
            run(f"sed -i 's/192\\.168\\.0\\.5/192.168.0.7/g' {f}")
            new = run(f"grep '192.168.0' {f} 2>/dev/null").strip()
            print(f"  -> {new}")
        else:
            print(f"Skip (no 0.5): {f}")

# Also check launch file
launch_files = run(f"grep -rl '192.168.0.5' {installed}/ 2>/dev/null").strip().split("\n")
print("\n=== Remaining 192.168.0.5 on RK3588 (should be sonar only) ===")
for lf in launch_files:
    if lf:
        is_ins = "ins" in lf.lower()
        if is_ins:
            print(f"  REVERT INS: {lf}")
            run(f"sed -i 's/192\\.168\\.0\\.5/192.168.0.7/g' {lf}")
        else:
            print(f"  OK (sonar): {lf}")

ssh.close()
print("\nDone.")
