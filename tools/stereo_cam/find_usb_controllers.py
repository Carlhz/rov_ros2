"""Find available USBPcap devices and try each one."""
import subprocess, os

# Method 1: Try opening USBPcap device files
for i in range(1, 10):
    device = f'\\\\.\\USBPcap{i}'
    proc = subprocess.Popen(
        [r'C:\Program Files\USBPcap\USBPcapCMD.exe', '-d', device, '--help'],
        stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    _, err = proc.communicate(timeout=3)
    err_text = err.decode(errors='replace')
    if 'cannot' in err_text.lower() or 'fail' in err_text.lower():
        continue
    print(f'USBPcap{i}: AVAILABLE')

# Method 2: List USB host controllers
print('\n=== USB Host Controllers ===')
result = subprocess.run(['powershell', '-Command', 
    'Get-PnpDevice -Class USB | Where-Object {$_.FriendlyName -like "*Host*" -or $_.FriendlyName -like "*Root*" -or $_.FriendlyName -like "*Controller*"} | Format-List FriendlyName,InstanceId,Status'],
    capture_output=True, text=True)
print(result.stdout[:2000])

# Method 3: Try to list all USB connected devices (for reference)
print('=== YLX Camera USB Hub Info ===')
result = subprocess.run(['powershell', '-Command',
    "Get-PnpDevice | Where-Object {$_.InstanceId -like '*1BCF*'} | Select-Object -ExpandProperty InstanceId"],
    capture_output=True, text=True)
for line in result.stdout.strip().split('\n'):
    line = line.strip()
    if line:
        print(f'  {line}')

# Method 4: Check which USB root hub the YLX is on
print('\n=== USB Tree (pnputil) ===')
result = subprocess.run(['powershell', '-Command',
    'Get-PnpDevice -Class USB | Where-Object {$_.FriendlyName -like "*Root Hub*"} | Format-List FriendlyName,InstanceId'],
    capture_output=True, text=True)
print(result.stdout[:1000])
