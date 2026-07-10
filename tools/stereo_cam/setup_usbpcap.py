#!/usr/bin/env python3
"""Setup USBPcap, bind to YLX camera's USB root hub, and start capture."""

import subprocess
import sys
import os
import time
import re

USBPcap_dir = r"C:\Program Files\USBPcap"
USBPcap_exe = os.path.join(USBPcap_dir, "USBPcapCMD.exe")
CAPTURE_OUTPUT = r"D:\Carl_WorkStation\rov_ros2\tools\stereo_cam\ylx_capture.pcapng"

def run_usbpcap_input(inputs):
    """Run USBPcapCMD with piped input and capture output."""
    proc = subprocess.Popen(
        [USBPcap_exe],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        cwd=USBPcap_dir,
        text=False  # binary mode for control
    )
    # USBPcapCMD expects text input
    input_str = "\n".join(inputs) + "\n"
    stdout, stderr = proc.communicate(input=input_str.encode('ascii', errors='ignore'), timeout=30)
    return stdout.decode('ascii', errors='ignore'), stderr.decode('ascii', errors='ignore')

def scan_devices():
    """Run USBPcapCMD with no input to list devices."""
    print("=" * 60)
    print("Scanning USB devices via USBPcapCMD...")
    print("=" * 60)
    
    proc = subprocess.Popen(
        [USBPcap_exe],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        cwd=USBPcap_dir,
    )
    # Send 'q' to quit immediately after listing
    try:
        stdout, stderr = proc.communicate(input=b"q\n", timeout=10)
    except subprocess.TimeoutExpired:
        proc.kill()
        stdout, stderr = proc.communicate()
    
    output = stdout.decode('ascii', errors='ignore')
    print(output[:3000])
    
    # Find root hub numbers
    # Output format: \\.\USBPcap1, \\.\USBPcap2, etc.
    hubs = re.findall(r'\\\\\.\\\\USBPcap(\d+)', output)
    return [int(h) for h in hubs]

def find_ylx_hub():
    """Try to find which USBPcap hub has the YLX camera."""
    print("\n" + "=" * 60)
    print("Finding YLX camera (VID 1BCF PID 0B15) on USB hubs...")
    print("=" * 60)
    
    # USBPcapCMD can filter by VID/PID
    # Format: USBPcapCMD.exe -d \\.\USBPcapN
    # But first, let's just try all hubs
    
    hubs = scan_devices()
    if not hubs:
        print("WARNING: No USBPcap devices found. Driver may need reboot to activate.")
        print("Please reboot your computer after USBPcap installation, then run this script again.")
        return None
    
    print(f"Found USBPcap hubs: {hubs}")
    
    # Since we can't easily determine which hub has YLX, 
    # the safest approach is to capture from ALL hubs and then filter
    # But for now, let's just try hub 1 (most common for single host controller)
    
    # Try to determine by checking USB tree
    # Use Windows API to find YLX device path
    import winreg
    
    # YLX camera video device
    vid_pid = "VID_1BCF&PID_0B15"
    
    print(f"\nFor now, the safest approach for capture:")
    print(f"  Hub 1 (\\\\.\\USBPcap1) is typically the primary USB 3.0 controller")
    print(f"  YLX camera is on a USB 3.0 root hub")
    print(f"\nRecommended: capture from ALL available hubs")
    
    return hubs

def capture_from_hub(hub_num, duration_seconds=30):
    """Start USB capture from a specific hub."""
    device = f"\\\\.\\USBPcap{hub_num}"
    output_file = CAPTURE_OUTPUT.replace(".pcapng", f"_hub{hub_num}.pcapng")
    
    print(f"\n{'=' * 60}")
    print(f"Starting capture from {device} for {duration_seconds}s...")
    print(f"Output: {output_file}")
    print(f"{'=' * 60}")
    print(f"\n>>> NOW: Open YLX camera software and start video preview <<<")
    print(f">>> Make sure gyroscope data is visible <<<")
    print(f">>> Move the camera around to generate IMU data <<<")
    
    # USBPcapCMD -d <device> -o <outputfile>
    # It runs until Ctrl+C, so use timeout via subprocess
    cmd = [USBPcap_exe, "-d", device, "-o", output_file]
    
    try:
        proc = subprocess.Popen(
            cmd,
            cwd=USBPcap_dir,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        
        print(f"\nCapturing... (will auto-stop after {duration_seconds}s)")
        print("Press Ctrl+C to stop early")
        
        try:
            proc.wait(timeout=duration_seconds)
        except subprocess.TimeoutExpired:
            # Send Ctrl+C equivalent - USBPcapCMD uses SIGINT or Ctrl+C
            print(f"\n{duration_seconds}s completed. Stopping capture...")
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
        
        stdout, stderr = proc.communicate(timeout=5)
        
        if os.path.exists(output_file):
            size_kb = os.path.getsize(output_file) / 1024
            print(f"\nCapture complete! File: {output_file} ({size_kb:.1f} KB)")
            return output_file
        else:
            print(f"\nWARNING: Output file not created: {output_file}")
            print(f"stdout: {stdout.decode('ascii', errors='ignore')[-500:]}")
            print(f"stderr: {stderr.decode('ascii', errors='ignore')[-500:]}")
            return None
            
    except Exception as e:
        print(f"Capture error: {e}")
        return None

def main():
    print("=" * 60)
    print("YLX USB Capture Setup")
    print("=" * 60)
    
    if not os.path.exists(USBPcap_exe):
        print(f"ERROR: USBPcapCMD.exe not found at {USBPcap_exe}")
        print("Please install USBPcap first.")
        sys.exit(1)
    
    hubs = find_ylx_hub()
    if hubs is None:
        return
    
    # For now, try hub 1
    # User should have already opened camera software by this point based on instructions
    capture_from_hub(1, duration_seconds=15)

if __name__ == "__main__":
    main()
