#!/usr/bin/env python3
"""通过串口发送文件到 RK3588"""

import serial
import time
import base64

def send_file_to_rk3588():
    # 读取文件并编码
    with open('D:\\Carl_WorkStation\\rov_ros2\\rk3588_ins_controller.py', 'rb') as f:
        data = f.read()
    
    # 转为 base64
    encoded = base64.b64encode(data).decode('ascii')
    
    # 打开串口
    s = serial.Serial('COM5', 115200, timeout=5)
    time.sleep(1)
    
    # 清空缓冲区
    s.read_all()
    
    # 创建目标文件
    s.write(b'cat > /opt/ros/rov_ros2_ws/rk3588_ins_controller.py << \'PYEOF\'\n')
    time.sleep(0.5)
    
    # 直接发送原始内容（不是 base64）
    with open('D:\\Carl_WorkStation\\rov_ros2\\rk3588_ins_controller.py', 'rb') as f:
        content = f.read()
        # 分段发送，每段 512 字节
        for i in range(0, len(content), 512):
            chunk = content[i:i+512]
            s.write(chunk)
            time.sleep(0.2)
    
    s.write(b'\nPYEOF\n')
    time.sleep(1)
    
    # 检查结果
    s.write(b'head -3 /opt/ros/rov_ros2_ws/rk3588_ins_controller.py\n')
    time.sleep(1)
    result = s.read_all().decode('utf-8', errors='ignore')
    print(result)
    
    s.close()
    print("文件传输完成")

if __name__ == '__main__':
    send_file_to_rk3588()
