#!/usr/bin/env python3
"""Debug USBPcap header structure"""
import struct

filepath = r"D:\Carl_WorkStation\rov_ros2\tools\stereo_cam\ylx_capture.pcapng"
with open(filepath, "rb") as f:
    data = f.read()

pos = 24
for frame_num in range(5):
    if pos + 16 > len(data):
        break
    incl_len = struct.unpack_from("<I", data, pos+8)[0]
    
    frame_start = pos + 16
    frame_data = data[frame_start : frame_start + min(incl_len, 80)]
    
    print(f"=== Frame {frame_num}: offset={pos}, incl_len={incl_len} ===")
    
    for i in range(0, min(40, len(frame_data)), 16):
        hexbytes = " ".join(f"{frame_data[j]:02X}" for j in range(i, min(i+16, len(frame_data))))
        print(f"  {i:02X}: {hexbytes}")
    
    hdr_len = struct.unpack_from("<H", frame_data, 0)[0]
    print(f"  header_len={hdr_len}")
    
    if hdr_len >= 27:
        irp_id = struct.unpack_from("<Q", frame_data, 2)[0]
        status = struct.unpack_from("<I", frame_data, 10)[0]
        function = struct.unpack_from("<H", frame_data, 14)[0]
        info = frame_data[16]
        bus = struct.unpack_from("<H", frame_data, 17)[0]
        device = struct.unpack_from("<H", frame_data, 19)[0]
        ep = frame_data[21]
        transfer = frame_data[22]
        
        print(f"  irp_id=0x{irp_id:016X}")
        print(f"  status=0x{status:08X}")
        print(f"  function=0x{function:04X}")
        print(f"  info=0x{info:02X}  bus={bus}  device={device}")
        print(f"  endpoint=0x{ep:02X}, transfer_type={transfer}")
        
        transfer_names = {0: "ISOCHRONOUS", 1: "INTERRUPT", 2: "CONTROL", 3: "BULK"}
        print(f"  => {transfer_names.get(transfer, 'UNKNOWN')} on EP 0x{ep:02X}")
        
        if ep == 0x82 and transfer == 3:
            payload = frame_data[hdr_len:]
            print(f"  *** FOUND INTERRUPT EP 0x82 ***")
            print(f"  payload ({len(payload)} bytes): {' '.join(f'{b:02X}' for b in payload[:16])}")
    print()
    pos += 16 + incl_len
