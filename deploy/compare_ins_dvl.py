#!/usr/bin/env python3
import paramiko, json, time
rk = {'hostname': '172.16.28.82', 'username': 'root', 'password': '159357'}
ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect(**rk, timeout=10)

# 看 DVL 和 INS 最新日志
print('=== DVL 最新 :BE 数据 ===')
stdin, stdout, stderr = ssh.exec_command('tail -10 /tmp/dvl_driver.log')
print(stdout.read().decode())

print('\n=== INS 最新数据 ===')
stdin, stdout, stderr = ssh.exec_command('tail -3 /tmp/ins_driver.log')
print(stdout.read().decode())

# 直接比较 ROS 话题数据 (通过 Python 解析 JSON)
print('\n=== 同时刻 INS vs DVL 数据 ===')
def get_topic(topic_name):
    cmd = f"bash -c 'source /opt/ros/setup.bash && export ROS_DOMAIN_ID=42 && timeout 3 ros2 topic echo {topic_name} --once 2>&1'"
    stdin, stdout, stderr = ssh.exec_command(cmd)
    out = stdout.read().decode()
    return out

ins_att = get_topic('/ins/attitude')
print('INS /ins/attitude:')
print(ins_att.strip()[-300:])

print()
dvl_status = get_topic('/rov/dvl/status')
print('DVL /rov/dvl/status:')
print(dvl_status.strip()[-300:])

ssh.close()
