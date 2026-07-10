#!/usr/bin/env python3
"""Fixed analysis of USBPcap IMU data (transfer_type=1 = INTERRUPT)"""

import struct, sys
from collections import Counter

filepath = sys.argv[1] if len(sys.argv) > 1 else r"D:\Carl_WorkStation\rov_ros2\tools\stereo_cam\ylx_capture.pcapng"

with open(filepath, "rb") as f:
    data = f.read()

pos = 24
all_payloads = []  # (irp_id, payload_bytes)
control_xfers = []
iso_xfers = []

while pos < len(data) - 16:
    incl_len = struct.unpack_from("<I", data, pos+8)[0]
    frame_start = pos + 16
    if frame_start + incl_len > len(data):
        break

    frame_data = data[frame_start : frame_start + incl_len]
    if len(frame_data) < 27:
        pos += 16 + max(incl_len, 0)
        continue

    hdr_len = struct.unpack_from("<H", frame_data, 0)[0]
    if hdr_len < 27:
        pos += 16 + incl_len
        continue

    irp_id = struct.unpack_from("<Q", frame_data, 2)[0]
    function = struct.unpack_from("<H", frame_data, 14)[0]
    info = frame_data[16]
    bus = struct.unpack_from("<H", frame_data, 17)[0]
    device = struct.unpack_from("<H", frame_data, 19)[0]
    ep = frame_data[21]
    transfer_type = frame_data[22]

    payload = frame_data[hdr_len:] if incl_len > hdr_len else b""

    # function 0x0009 = URB_FUNCTION_ISOCH_TRANSFER (isochronous)
    # function 0x0000 = URB_FUNCTION_BULK_OR_INTERRUPT_TRANSFER
    # transfer_type: 0=ISOCH, 1=INTERRUPT, 2=CONTROL, 3=BULK

    if transfer_type == 1:  # INTERRUPT
        all_payloads.append((irp_id, info, payload))
    elif transfer_type == 2:  # CONTROL
        if payload:
            control_xfers.append(payload)
    elif transfer_type == 0:  # ISOCHRONOUS
        iso_xfers.append((info, payload))

    pos += 16 + incl_len

print(f"Interrupt packets: {len(all_payloads)}")
print(f"Control packets with data: {len(control_xfers)}")
print(f"Isochronous packets: {len(iso_xfers)}")

if not all_payloads:
    print("No interrupt packets found!")
    sys.exit(0)

# Separate data-bearing (info=1) from empty (info=0)
data_packets = [(irp, pl) for irp, info, pl in all_payloads if info == 1 and len(pl) >= 8]
empty_packets = [(irp, pl) for irp, info, pl in all_payloads if info == 0]

print(f"With data (info=1): {len(data_packets)}")
print(f"Empty (info=0): {len(empty_packets)}")

# Show first 30 data packets
print("\n=== First 30 Data Packets ===")
for i, (irp, pl) in enumerate(data_packets[:30]):
    hex_str = " ".join(f"{b:02X}" for b in pl[:8])
    
    # Try parsing as: frame_number(u16) + X(u16) + Y(u16) + Z(u16)
    fn = struct.unpack_from("<H", pl, 0)[0]
    x_raw = struct.unpack_from("<H", pl, 2)[0]
    y_raw = struct.unpack_from("<H", pl, 4)[0]
    z_raw = struct.unpack_from("<H", pl, 6)[0]
    
    # High 12 bits
    xs = (x_raw >> 4) & 0xFFF
    if xs >= 0x800: xs -= 0x1000
    ys = (y_raw >> 4) & 0xFFF
    if ys >= 0x800: ys -= 0x1000
    zs = (z_raw >> 4) & 0xFFF
    if zs >= 0x800: zs -= 0x1000
    
    print(f"  #{i:3d}: {hex_str}   fn={fn:5d}  X={xs:+5d}  Y={ys:+5d}  Z={zs:+5d}")

# Count unique payloads
counts = Counter(bytes(pl[:8]) for _, pl in data_packets)
print(f"\n=== Unique payloads: {len(counts)} ===")
for pbytes, cnt in counts.most_common(10):
    hex_str = " ".join(f"{b:02X}" for b in pbytes)
    print(f"  {hex_str}: {cnt}x")

# Check if IMU values vary
all_x, all_y, all_z = [], [], []
for _, pl in data_packets:
    x_raw = struct.unpack_from("<H", pl, 2)[0]
    y_raw = struct.unpack_from("<H", pl, 4)[0]
    z_raw = struct.unpack_from("<H", pl, 6)[0]
    xs = (x_raw >> 4) & 0xFFF
    if xs >= 0x800: xs -= 0x1000
    ys = (y_raw >> 4) & 0xFFF
    if ys >= 0x800: ys -= 0x1000
    zs = (z_raw >> 4) & 0xFFF
    if zs >= 0x800: zs -= 0x1000
    all_x.append(xs)
    all_y.append(ys)
    all_z.append(zs)

if all_x:
    print(f"\n=== IMU Value Ranges (signed 12-bit) ===")
    print(f"  X: {min(all_x)} ~ {max(all_x)}  (unique: {len(set(all_x))})")
    print(f"  Y: {min(all_y)} ~ {max(all_y)}  (unique: {len(set(all_y))})")
    print(f"  Z: {min(all_z)} ~ {max(all_z)}  (unique: {len(set(all_z))})")

print(f"\n=== Sample Rate Analysis ===")
# Time between first and last data packet
# We need timestamps from pcap
pos = 24
data_timestamps = []
while pos < len(data) - 16:
    ts_sec = struct.unpack_from("<I", data, pos)[0]
    ts_usec = struct.unpack_from("<I", data, pos+4)[0]
    incl_len = struct.unpack_from("<I", data, pos+8)[0]
    frame_start = pos + 16
    if frame_start + incl_len > len(data):
        break
    frame_data = data[frame_start : frame_start + incl_len]
    if len(frame_data) >= 27:
        hdr_len = struct.unpack_from("<H", frame_data, 0)[0]
        info = frame_data[16]
        transfer_type = frame_data[22] if hdr_len >= 23 else 0
        if transfer_type == 1 and info == 1:  # INTERRUPT with data
            data_timestamps.append(ts_sec + ts_usec / 1000000.0)
    pos += 16 + incl_len

if len(data_timestamps) >= 2:
    duration = data_timestamps[-1] - data_timestamps[0]
    print(f"  Duration: {duration:.3f}s")
    print(f"  Data packets: {len(data_timestamps)}")
    print(f"  Rate: {len(data_timestamps)/duration:.1f} Hz" if duration > 0 else "  (all same timestamp)")
