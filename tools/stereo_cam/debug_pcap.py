"""Debug pcapng file structure"""
import struct

filepath = r'D:\Carl_WorkStation\rov_ros2\tools\stereo_cam\ylx_capture.pcapng'
with open(filepath, 'rb') as f:
    data = f.read()

print(f'File size: {len(data)} bytes')

# First 256 bytes hex dump
print('\n=== First 256 bytes ===')
for i in range(0, min(256, len(data)), 16):
    hex_bytes = ' '.join(f'{data[j]:02X}' for j in range(i, min(i+16, len(data))))
    ascii_repr = ''.join(chr(data[j]) if 32 <= data[j] < 127 else '.' for j in range(i, min(i+16, len(data))))
    print(f'{i:04X}: {hex_bytes:<48s}  {ascii_repr}')

print('\n=== Block analysis ===')
pos = 0
block_num = 0
while pos < len(data) and block_num < 20:
    if pos + 8 > len(data):
        break
    block_type = struct.unpack_from('<I', data, pos)[0]
    block_len = struct.unpack_from('<I', data, pos + 4)[0]
    
    type_names = {
        0x0A0D0D0A: 'SHB',
        0x00000001: 'IDB',
        0x00000006: 'EPB',
        0x00000003: 'SPB',
        0x00000004: 'NRB',
        0x00000005: 'ISB',
    }
    tname = type_names.get(block_type, f'UNKNOWN')
    print(f'Offset {pos}: type={tname} (0x{block_type:08X}), length={block_len}')
    
    if block_len <= 0 or block_len > len(data) - pos:
        print(f'  Invalid block_len, trying next 4 bytes')
        pos += 4
        continue
    
    if block_type == 0x0A0D0D0A:  # SHB
        bom = struct.unpack_from('<I', data, pos + 8)[0]
        ver_major = struct.unpack_from('<H', data, pos + 12)[0]
        ver_minor = struct.unpack_from('<H', data, pos + 14)[0]
        sec_len = struct.unpack_from('<q', data, pos + 16)[0]
        print(f'  BOM=0x{bom:08X}, v{ver_major}.{ver_minor}, section_len={sec_len}')
    
    elif block_type == 0x00000001:  # IDB
        # Interface Description Block
        link_type = struct.unpack_from('<H', data, pos + 8)[0]
        snap_len = struct.unpack_from('<I', data, pos + 12)[0]
        link_names = {220: 'USB_LINKTYPE', 1: 'ETHERNET', 0: 'NULL'}
        print(f'  LinkType={link_type} ({link_names.get(link_type, "?")}), SnapLen={snap_len}')
    
    elif block_type == 0x00000006:  # EPB
        if block_len < 28:
            print(f'  Too small for EPB, skipping')
            pos += block_len
            pos = (pos + 3) & ~3
            continue
        
        interface_id = struct.unpack_from('<I', data, pos + 8)[0]
        timestamp_hi = struct.unpack_from('<I', data, pos + 12)[0]
        timestamp_lo = struct.unpack_from('<I', data, pos + 16)[0]
        cap_len = struct.unpack_from('<I', data, pos + 20)[0]
        orig_len = struct.unpack_from('<I', data, pos + 24)[0]
        
        pkt_offset = pos + 28
        print(f'  iface={interface_id}, cap_len={cap_len}, orig_len={orig_len}')
        
        if pkt_offset + cap_len <= len(data):
            pkt_data = data[pkt_offset:pkt_offset + min(cap_len, 80)]
            hex_str = ' '.join(f'{b:02X}' for b in pkt_data[:48])
            print(f'  Data: {hex_str}')
            
            # Try to interpret as USB header
            if cap_len >= 2:
                usb_hdr_len = struct.unpack_from('<H', pkt_data, 0)[0]
                usb_hdr_type = struct.unpack_from('<H', pkt_data, 2)[0]
                print(f'  USB HdrLen={usb_hdr_len}, Type={usb_hdr_type}')
                
                if usb_hdr_type == 0 and usb_hdr_len >= 27:
                    irp_id = struct.unpack_from('<Q', pkt_data, 4)[0]
                    status = struct.unpack_from('<I', pkt_data, 12)[0]
                    urb_func = struct.unpack_from('<H', pkt_data, 16)[0]
                    info = pkt_data[18]
                    bus = struct.unpack_from('<H', pkt_data, 19)[0]
                    dev = struct.unpack_from('<H', pkt_data, 21)[0]
                    ep = pkt_data[23]
                    transfer = pkt_data[24]
                    dlen = struct.unpack_from('<I', pkt_data, 25)[0]
                    
                    transfer_names = {0: 'ISOCH', 1: 'INTERRUPT', 2: 'CONTROL', 3: 'BULK'}
                    urb_names = {0x0008: 'CONTROL_TRANSFER', 0x0009: 'BULK_OR_INTERRUPT_TRANSFER', 0x001A: 'ISOCH_TRANSFER'}
                    
                    print(f'  IRP={irp_id}, URB={urb_names.get(urb_func, hex(urb_func))}, Status={status}')
                    print(f'  Bus={bus} Dev={dev} EP=0x{ep:02X} Transfer={transfer_names.get(transfer, str(transfer))} DataLen={dlen}')
                    
                    if urb_func == 0x0008 and dlen >= 8:
                        ctrl_offset = usb_hdr_len
                        bmRT = pkt_data[ctrl_offset]
                        bReq = pkt_data[ctrl_offset + 1]
                        wVal = struct.unpack_from('<H', pkt_data, ctrl_offset + 2)[0]
                        wIdx = struct.unpack_from('<H', pkt_data, ctrl_offset + 4)[0]
                        wLen = struct.unpack_from('<H', pkt_data, ctrl_offset + 6)[0]
                        cdata = pkt_data[ctrl_offset + 8 : ctrl_offset + 8 + wLen]
                        
                        print(f'  SETUP: bmRT=0x{bmRT:02X} bReq=0x{bReq:02X} wVal=0x{wVal:04X} wIdx=0x{wIdx:04X} wLen={wLen}')
                        if cdata:
                            hex_cdata = ' '.join(f'{b:02X}' for b in cdata[:64])
                            print(f'  Payload[{len(cdata)}]: {hex_cdata}')
    
    block_num += 1
    pos += block_len
    # Align to 4 bytes
    pos = (pos + 3) & ~3
