#!/usr/bin/env python3
"""Find the correct USBPcap device for the YLX camera and capture"""
import subprocess, time, os, sys

usbpcap_exe = r"C:\Program Files\USBPcap\USBPcapCMD.exe"
output_dir = r"D:\Carl_WorkStation\rov_ros2\tools\stereo_cam"

# Try each USBPcap device briefly and check if it produces any output
for idx in range(10):
    pcap_file = os.path.join(output_dir, f"test_usbpcap{idx}.pcap")
    
    print(f"\n=== Testing \\\\.\\USBPcap{idx} ===")
    cmd = [
        usbpcap_exe,
        "-d", f"\\\\.\\USBPcap{idx}",
        "-o", pcap_file,
        "-s", "1024",
        "-A",
        "--inject-descriptors"
    ]
    
    try:
        proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, 
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        time.sleep(2)  # Capture for 2 seconds
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except:
            proc.kill()
        
        time.sleep(0.5)
        
        if os.path.exists(pcap_file):
            size = os.path.getsize(pcap_file)
            if size > 24:  # More than just pcap header
                # Read first few packets to see device addresses
                with open(pcap_file, 'rb') as f:
                    data = f.read()
                
                # Quick scan for device addresses
                # USB pcap packets have device address at offset 19-21
                devices_found = set()
                offset = 24
                while offset + 40 < len(data):
                    incl_len = int.from_bytes(data[offset+8:offset+12], 'little')
                    if incl_len > 0 and offset + 16 + incl_len <= len(data):
                        pkt = data[offset+16:offset+16+incl_len]
                        if len(pkt) >= 21:
                            dev = int.from_bytes(pkt[17:19], 'little')
                            devices_found.add(dev)
                    offset += 16 + incl_len
                
                print(f"  CAPTURED: {size} bytes, devices: {devices_found}")
                
                # Look for device 10 (which is the camera address from registry)
                if 10 in devices_found:
                    print(f"  *** FOUND CAMERA (dev 10) on USBPcap{idx}! ***")
                    # Don't delete this one
                else:
                    os.remove(pcap_file)
            else:
                print(f"  Empty or header-only ({size} bytes)")
                os.remove(pcap_file)
        else:
            print(f"  No output file")
    except Exception as e:
        print(f"  Error: {e}")
