#!/usr/bin/env python3
"""Capture USB: start capture FIRST, THEN open YLX software."""
import subprocess, os, time, ctypes

USBPcap_exe = r"C:\Program Files\USBPcap\USBPcapCMD.exe"
OUTPUT_FILE = r"D:\Carl_WorkStation\rov_ros2\tools\stereo_cam\ylx_capture3.pcapng"

device = r"\\.\USBPcap1"

print("=" * 60)
print("YLX USB Capture - Round 3")
print("=" * 60)
print()
print(">>> 操作步骤 <<<")
print("1. 先关闭所有 YLX 摄像头软件")
print("2. 5 秒后自动开始抓包")
print("3. 看到 'CAPTURING' 后 → 打开 YLX 摄像头软件")
print("4. 启动视频预览，确认陀螺仪数据可见")
print("5. 移动摄像头，30 秒后自动停止")
print()

for i in range(5, 0, -1):
    print(f"  {i}...")
    time.sleep(1)

if os.path.exists(OUTPUT_FILE):
    os.remove(OUTPUT_FILE)

print("\n>>> CAPTURING STARTED <<<")
print(">>> NOW open YLX camera software! <<<\n")

cmd = [USBPcap_exe, "-d", device, "-o", OUTPUT_FILE, "-A", "-s", "128"]
proc = subprocess.Popen(cmd, cwd=r"C:\Program Files\USBPcap",
                        stdout=subprocess.PIPE, stderr=subprocess.PIPE)

for remaining in range(30, 0, -5):
    if proc.poll() is not None:
        print(f"  Process exited early: {proc.returncode}")
        break
    print(f"  ... {remaining}s ...")
    time.sleep(5)

print("\nStopping capture...")
if proc.poll() is None:
    proc.terminate()
    try:
        proc.wait(timeout=5)
    except:
        proc.kill()

if os.path.exists(OUTPUT_FILE):
    size_kb = os.path.getsize(OUTPUT_FILE) / 1024
    print(f"\nSUCCESS! {OUTPUT_FILE} ({size_kb:.1f} KB)")
else:
    print("\nFAILED - no file created")
