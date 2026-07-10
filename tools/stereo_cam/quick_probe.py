import paramiko, sys, time

c = paramiko.SSHClient()
c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect("172.16.30.0", username="carl", password="159357", timeout=15)

# Upload
s = c.open_sftp()
s.put(r"D:\Carl_WorkStation\rov_ros2\tools\stereo_cam\ylx_xu_probe.py", "/home/carl/ylx_xu_probe.py")
s.close()

# Kill old v4l2-ctl
c.exec_command("echo '159357' | sudo -S pkill -9 v4l2-ctl 2>/dev/null")
time.sleep(0.5)

# Run XU probe
channel = c.get_transport().open_session()
channel.get_pty()
channel.exec_command('/bin/bash -c "echo 159357 | sudo -S python3 /home/carl/ylx_xu_probe.py 2>&1"')

start = time.time()
while time.time() - start < 20:
    if channel.recv_ready():
        d = channel.recv(4096)
        if not d:
            break
        sys.stdout.write(d.decode("utf-8", errors="replace"))
        sys.stdout.flush()
    if channel.exit_status_ready():
        break
    time.sleep(0.05)
try:
    while channel.recv_ready():
        d = channel.recv(4096)
        if d:
            sys.stdout.write(d.decode("utf-8", errors="replace"))
except:
    pass

print("\nDone!")
c.close()
