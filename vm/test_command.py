import serial
import time

s = serial.Serial('COM5', 115200, timeout=5)
time.sleep(1)

# 测试发送 connect 命令
cmd = b'export ROS_DOMAIN_ID=42 && ros2 topic pub /ins/command std_msgs/String "{data: \'{\\"action\\": \\"connect\\"}\'}" --once\n'
s.write(cmd)
time.sleep(3)

# 查看控制器日志
s.write(b'cat /tmp/ins_controller.log | tail -20\n')
time.sleep(1)

output = s.read_all().decode('utf-8', errors='ignore')
print(output)

s.close()
