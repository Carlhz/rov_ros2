#!/usr/bin/env python3
"""检查 USR IOT 模块 web 配置 + 搜索原始 ROS1 声纳协议"""
import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("172.16.28.82", username="root", password="159357", timeout=10)

def run(cmd, timeout=15):
    stdin, stdout, stderr = ssh.exec_command(cmd, timeout=timeout)
    out = stdout.read().decode("utf-8", errors="replace")
    err = stderr.read().decode("utf-8", errors="replace")
    return out + err

# 1. 尝试访问 USR IOT 模块的常见配置页面
print("=" * 60)
print("1. USR IOT 模块 Web 页面探测")
print("=" * 60)
urls = [
    "http://192.168.0.5/",
    "http://192.168.0.5/index.html",
    "http://192.168.0.5/index.asp",
    "http://192.168.0.5/status.html",
    "http://192.168.0.5/config.html",
    "http://192.168.0.5/net.html",
    "http://192.168.0.5/serial.html",
    "http://192.168.0.5/login.html",
]
for url in urls:
    result = run("curl -s --connect-timeout 3 -o /dev/null -w '%{http_code}' " + url + " 2>&1", timeout=5)
    code = result.strip()
    if code == "200":
        print("  OK(200): " + url)
    elif code == "401":
        print("  AUTH(401): " + url)
    elif code == "301" or code == "302":
        print("  REDIR(" + code + "): " + url)

# 2. 尝试默认用户名密码
print()
print("=" * 60)
print("2. 尝试默认登录")
print("=" * 60)
creds = [("admin", "admin"), ("root", "root"), ("admin", ""), ("admin", "123456"), ("admin", "888888")]
for u, p in creds:
    cmd = "curl -s --connect-timeout 3 -u '" + u + ":" + p + "' http://192.168.0.5/ 2>&1 | head -5"
    result = run(cmd, timeout=5)
    if "Required Authorization" not in result and len(result) > 50:
        print(f"  可能成功: {u}:{p}")
        print(f"  内容: {result[:200]}")
        break
else:
    print("  所有默认凭据均失败")

# 3. 查看原始 ROS1 声纳代码中的协议细节
print()
print("=" * 60)
print("3. 搜索 ROS1 声纳协议参考代码")
print("=" * 60)
# Check if there's ROS1 code on RK3588
print(run("find /opt/ros -name '*sonar*' -o -name '*scanfish*' 2>/dev/null | head -10"))
print(run("find /home -name '*sonar*' -o -name '*scanfish*' 2>/dev/null | head -10"))

# 4. 检查声纳驱动的命令构造代码
print()
print("=" * 60)
print("4. 声纳驱动 C++ 命令构造代码")
print("=" * 60)
print(run("grep -n -A5 'make_cmd\\|cmd_buf\\|sendto\\|CMD_SIZE' /opt/ros/rov_ros2_ws/src/rov_sonar_driver/src/sonar_omni_driver.cpp 2>/dev/null | head -60"))

ssh.close()
