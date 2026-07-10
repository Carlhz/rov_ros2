"""Reinstall USBPcap for all USB root hubs and verify"""
import subprocess, sys

usbpcap = r'C:\Program Files\USBPcap\USBPcapCMD.exe'

print("=== Removing all USBPcap adapters ===")
for i in range(5):
    rc = subprocess.call([usbpcap, '-d', f'\\\\.\\USBPcap{i}', '-R'],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print(f'  USBPcap{i} remove: rc={rc}')

print("\n=== Installing USBPcap for root hubs ===")
for i in range(3):
    rc = subprocess.call([usbpcap, '-d', f'\\\\.\\USBPcap{i}', '-I'],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print(f'  USBPcap{i} install: rc={rc}')

print("\n=== Verifying via CreateFile ===")
import ctypes
from ctypes import wintypes
kernel32 = ctypes.windll.kernel32
GENERIC_READ = 0x80000000
FILE_SHARE_READ = 1
FILE_SHARE_WRITE = 2
OPEN_EXISTING = 3

for i in range(5):
    path = f'\\\\.\\USBPcap{i}'
    h = kernel32.CreateFileW(path, GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE,
                              None, OPEN_EXISTING, 0x80, None)
    if h and int(h) != -1:
        print(f'  USBPcap{i}: OK (handle={int(h)})')
        kernel32.CloseHandle(h)
    else:
        err = kernel32.GetLastError()
        print(f'  USBPcap{i}: FAIL (err={err})')
