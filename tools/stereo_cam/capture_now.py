"""Quick USBPcap capture from connected camera."""
import subprocess, time, os, struct, sys

OUTPUT = r'D:\Carl_WorkStation\rov_ros2\tools\stereo_cam\ylx_imu_stream.pcap'
USPCAP = r'C:\Program Files\USBPcap\USBPcapCMD.exe'

if os.path.exists(OUTPUT):
    os.remove(OUTPUT)

print('Starting capture on \\\\.\\USBPcap1...')
proc = subprocess.Popen(
    [USPCAP, '-d', r'\\.\USBPcap1', '-o', OUTPUT, '-s', '4096', '-A'],
    stdout=subprocess.PIPE, stderr=subprocess.PIPE)
print(f'USBPcapCMD PID: {proc.pid}')

time.sleep(1)
if proc.poll() is not None:
    out, err = proc.communicate()
    print(f'USBPcapCMD FAILED! stdout={out.decode(errors="replace")} stderr={err.decode(errors="replace")}')
    sys.exit(1)

# Open camera and stream 10s
import cv2
for idx in range(5):
    cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
    if cap.isOpened():
        print(f'Camera opened on index {idx}')
        for i in range(300):
            ret, _ = cap.read()
            if i == 0:
                print(f'  First frame OK' if ret else f'  First frame FAILED')
        cap.release()
        print('  Released')
        break
else:
    print('Camera not found on indices 0-4!')

time.sleep(2)
proc.terminate()
try:
    proc.wait(timeout=5)
except:
    proc.kill()

if not os.path.exists(OUTPUT):
    print('No capture file!')
    sys.exit(1)

size = os.path.getsize(OUTPUT)
print(f'Capture: {size} bytes ({size/1024:.1f} KB)')

# Quick analysis
with open(OUTPUT, 'rb') as f:
    data = f.read()

offset = 24
pkt_num, ctrl, intr = 0, 0, 0
while offset + 16 <= len(data):
    incl_len = struct.unpack_from('<I', data, offset+8)[0]
    offset += 16
    if offset + incl_len > len(data):
        break
    pkt = data[offset:offset+incl_len]
    offset += incl_len
    pkt_num += 1
    if len(pkt) < 28:
        continue
    hdr_len = struct.unpack('<H', pkt[0:2])[0]
    if hdr_len < 28:
        continue
    xfer = pkt[22] & 0x03
    if xfer == 2: ctrl += 1
    elif xfer == 1: intr += 1

print(f'Packets: {pkt_num}, CTRL: {ctrl}, INTR: {intr}')
if intr > 0:
    print('*** INTERRUPT DATA FOUND! ***')
else:
    print('No interrupt packets in stream capture')
