#!/usr/bin/env python3
"""Re-capture USB traffic while plugging in the YLX camera to catch XU activation commands."""
import subprocess, os, sys, time, ctypes

USBPcap_exe = r"C:\Program Files\USBPcap\USBPcapCMD.exe"
OUTPUT_FILE = r"D:\Carl_WorkStation\rov_ros2\tools\stereo_cam\ylx_capture2.pcapng"

def find_devices():
    """Find available USBPcap devices."""
    devices = []
    for i in range(1, 10):
        device = r"\\.\USBPcap" + str(i)
        try:
            handle = ctypes.windll.kernel32.CreateFileW(
                device, 0x80000000, 0x00000001 | 0x00000002, None, 3, 0, None)
            if handle not in (-1, 0):
                ctypes.windll.kernel32.CloseHandle(handle)
                devices.append(i)
        except:
            continue
    return devices

def capture():
    print("=" * 60)
    print("YLX USB Capture - Round 2 (with device plug-in)")
    print("=" * 60)

    devices = find_devices()
    if not devices:
        print("ERROR: No USBPcap devices!")
        return
    device = r"\\.\USBPcap" + str(devices[0])
    print(f"Device: {device}")

    # Remove old file
    if os.path.exists(OUTPUT_FILE):
        os.remove(OUTPUT_FILE)

    print()
    print("=" * 60)
    print(">>> INSTRUCTIONS <<<")
    print("=" * 60)
    print("Step 1: Countdown starts in 3 seconds")
    print("Step 2: Capture will start")
    print("Step 3: THEN unplug the YLX camera USB cable")
    print("Step 4: Wait 5 seconds, then plug it back IN")
    print("Step 5: Wait for software to detect it")
    print("Step 6: Open YLX camera software, start preview")
    print("Step 7: Move camera to trigger IMU data")
    print("Capture will stop after 30 seconds")
    print()

    for i in range(3, 0, -1):
        print(f"  Starting in {i}...")
        time.sleep(1)

    print("\n>>> CAPTURING (unplug camera NOW) <<<")

    cmd = [USBPcap_exe, "-d", device, "-o", OUTPUT_FILE, "-A", "-s", "128"]
    proc = subprocess.Popen(cmd, cwd=r"C:\Program Files\USBPcap",
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    duration = 30
    for remaining in range(duration, 0, -5):
        if proc.poll() is not None:
            print(f"WARN: process exited early (code={proc.returncode})")
            break
        print(f"  ... {remaining}s ...")
        time.sleep(5)

    print("\nStopping capture...")
    if proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()

    if os.path.exists(OUTPUT_FILE):
        size_kb = os.path.getsize(OUTPUT_FILE) / 1024
        print(f"\nSUCCESS! File: {OUTPUT_FILE} ({size_kb:.1f} KB)")
        print(f"\nAnalyze: python analyze_pcap_v2.py \"{OUTPUT_FILE}\"")
    else:
        print("\nERROR: No capture file created!")

if __name__ == "__main__":
    capture()
