#!/usr/bin/env python3
"""Transfer sysroot tarball to new VM via SCP with progress reporting."""
import paramiko
import os
import sys
import time
import socket

VM_HOST = '172.16.31.177'
VM_USER = 'carl'
VM_PASS = '159357'
LOCAL_FILE = r'D:\Carl_WorkStation\TL3588_V2.4\4-software\Ubuntu\LinuxSDK\rk3588-ubuntu20.04-sysroot-v1.1.tar.gz'
REMOTE_FILE = '/home/carl/rk3588-ubuntu20.04-sysroot-v1.1.tar.gz'

def progress_callback(transferred, total):
    pct = (transferred / total) * 100
    mb_done = transferred / (1024*1024)
    mb_total = total / (1024*1024)
    # Print progress every 5%
    if int(pct) % 5 == 0:
        sys.stdout.write(f'\rTransfer: {mb_done:.0f}/{mb_total:.0f} MB ({pct:.1f}%)')
        sys.stdout.flush()

def main():
    file_size = os.path.getsize(LOCAL_FILE)
    print(f'Local file: {LOCAL_FILE}')
    print(f'Size: {file_size / (1024*1024*1024):.2f} GB')
    
    # Connect SSH
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(VM_HOST, username=VM_USER, password=VM_PASS, timeout=30)
    print(f'Connected to {VM_HOST}')
    
    # Check available space
    stdin, stdout, stderr = ssh.exec_command('df -h / | tail -1')
    print(f'Disk space: {stdout.read().decode().strip()}')
    
    # Create RK3588 directory
    ssh.exec_command('mkdir -p /home/carl/RK3588')
    
    # Transfer file using SFTP with progress
    print(f'\nTransferring to {REMOTE_FILE}...')
    sftp = ssh.open_sftp()
    
    start_time = time.time()
    sftp.put(LOCAL_FILE, REMOTE_FILE, callback=progress_callback)
    elapsed = time.time() - start_time
    print(f'\n\nTransfer complete! Time: {elapsed:.1f}s ({file_size/elapsed/1024/1024:.1f} MB/s)')
    
    # Verify file size
    remote_stat = sftp.stat(REMOTE_FILE)
    print(f'Remote file size: {remote_stat.st_size} bytes')
    print(f'Local file size:  {file_size} bytes')
    if remote_stat.st_size == file_size:
        print('Size match: OK')
    else:
        print('WARNING: Size mismatch!')
    
    sftp.close()
    
    # Verify checksum (md5)
    print('\nVerifying md5sum...')
    import hashlib
    local_md5 = hashlib.md5()
    with open(LOCAL_FILE, 'rb') as f:
        while chunk := f.read(8192*1024):
            local_md5.update(chunk)
    local_md5_hex = local_md5.hexdigest()
    
    stdin, stdout, stderr = ssh.exec_command(f'md5sum {REMOTE_FILE}')
    remote_md5_hex = stdout.read().decode().split()[0]
    
    print(f'Local MD5:  {local_md5_hex}')
    print(f'Remote MD5: {remote_md5_hex}')
    if local_md5_hex == remote_md5_hex:
        print('MD5 match: OK')
    else:
        print('WARNING: MD5 mismatch!')
    
    ssh.close()
    print('\nDone!')

if __name__ == '__main__':
    main()
