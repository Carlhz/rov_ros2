"""Test USBPcapCMD interactive mode to see available root hubs"""
import subprocess, time, sys

usbpcap = r'C:\Program Files\USBPcap\USBPcapCMD.exe'

# Run USBPcapCMD and pipe input
print("Starting USBPcapCMD interactive...")
p = subprocess.Popen(
    [usbpcap],
    stdin=subprocess.PIPE,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
    text=True
)

# Wait for the menu
time.sleep(2)

# Press Enter to select first option (list devices?)
try:
    p.stdin.write('\n')
    p.stdin.flush()
    time.sleep(2)
    p.stdin.write('q\n')
    p.stdin.flush()
except:
    pass

try:
    stdout, stderr = p.communicate(timeout=5)
    print("=== STDOUT ===")
    print(stdout)
    if stderr:
        print("=== STDERR ===")
        print(stderr)
except subprocess.TimeoutExpired:
    p.kill()
    stdout, stderr = p.communicate()
    print("=== STDOUT (timeout) ===")
    print(stdout)
    if stderr:
        print("=== STDERR ===")
        print(stderr)
