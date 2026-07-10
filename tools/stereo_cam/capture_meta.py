import paramiko, time, sys

c = paramiko.SSHClient()
c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect("172.16.30.0", username="carl", password="159357", timeout=15)

# Kill old
c.exec_command("echo '159357' | sudo -S pkill -9 v4l2-ctl 2>/dev/null")
time.sleep(1)

# Start video stream in background
c.exec_command("echo '159357' | sudo -S nohup v4l2-ctl -d /dev/video0 --set-fmt-video=width=640,height=480,pixelformat=MJPG --stream-mmap 1>/dev/null 2>/dev/null &")
time.sleep(3)

# Capture metadata frames
print("=== Capturing metadata ===")
_, so, se = c.exec_command("echo '159357' | sudo -S timeout 5 v4l2-ctl -d /dev/video1 --stream-mmap --stream-count=3 --stream-to=/tmp/meta.bin 2>&1", timeout=15)
out = so.read().decode()
err = se.read().decode()
if out.strip():
    print(out)
if err.strip():
    print("ERR:", err[:300])

# Analyze captured metadata
print("\n=== Metadata file ===")
_, so, _ = c.exec_command("ls -la /tmp/meta.bin 2>/dev/null; echo '---'; xxd /tmp/meta.bin 2>/dev/null | head -40")
print(so.read().decode())

# Kill background stream
c.exec_command("echo '159357' | sudo -S pkill -9 v4l2-ctl 2>/dev/null")
c.close()
