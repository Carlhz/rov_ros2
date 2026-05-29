#!/usr/bin/env python3
"""
RK3588 INS 控制器 - 纯 Python 实现，无需编译接口包
使用标准 String 消息传递 JSON 格式的控制命令
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
import json
import socket
import struct
import threading
import time

# INS 网络配置
INS_IP = "192.168.0.7"
INS_CMD_PORT = 8007  # 命令端口
INS_DATA_PORT = 8008  # 数据端口

# INS 控制命令
INS_CMD_START = b'\x5A\xA5\x47\x01\x01\x00\x00\x47\x55'  # 启动输出
INS_CMD_STOP = b'\x5A\xA5\x47\x00\x01\x00\x00\x46\x55'   # 停止输出

# 工作状态映射
ALIGNMENT_STATUS = {
    0: "未对准",
    1: "粗对准",
    2: "精对准",
    3: "INS导航模式"
}


class RK3588INSController(Node):
    """RK3588 INS 控制器节点"""
    
    def __init__(self):
        super().__init__('rk3588_ins_controller')
        
        # 创建发布者 - 发布 INS 数据
        self.attitude_pub = self.create_publisher(Vector3, '/ins/attitude', 10)
        self.velocity_pub = self.create_publisher(Vector3, '/ins/velocity', 10)
        self.position_pub = self.create_publisher(Vector3, '/ins/position', 10)
        self.status_pub = self.create_publisher(String, '/ins/status', 10)
        
        # 创建订阅者 - 接收控制命令
        self.cmd_sub = self.create_subscription(
            String,
            '/ins/command',
            self.command_callback,
            10
        )
        
        # INS 数据接收线程
        self.ins_socket = None
        self.running = False
        self.ins_thread = None
        
        # 当前状态
        self.current_status = {
            'alignment': 0,
            'pitch': 0.0,
            'roll': 0.0,
            'yaw': 0.0,
            've': 0.0,
            'vn': 0.0,
            'vd': 0.0,
            'lat': 0.0,
            'lon': 0.0,
            'alt': 0.0,
            'sats': 0,
            'temp': 0
        }
        
        self.get_logger().info("RK3588 INS Controller 已启动")
        self.get_logger().info("订阅 /ins/command 接收控制命令")
        self.get_logger().info("发布 /ins/attitude /ins/velocity /ins/position /ins/status")
    
    def command_callback(self, msg):
        """处理控制命令"""
        try:
            cmd = json.loads(msg.data)
            action = cmd.get('action', '')
            
            if action == 'start':
                self.start_ins_output()
            elif action == 'stop':
                self.stop_ins_output()
            elif action == 'connect':
                self.connect_ins()
            elif action == 'disconnect':
                self.disconnect_ins()
            elif action == 'set_position':
                lat = cmd.get('lat', 0)
                lon = cmd.get('lon', 0)
                self.set_position(lat, lon)
            else:
                self.get_logger().warn(f"未知命令: {action}")
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON 解析错误: {e}")
        except Exception as e:
            self.get_logger().error(f"命令处理错误: {e}")
    
    def start_ins_output(self):
        """启动 INS 数据输出"""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.sendto(INS_CMD_START, (INS_IP, INS_CMD_PORT))
            sock.close()
            self.get_logger().info("已发送 INS 启动命令")
        except Exception as e:
            self.get_logger().error(f"启动 INS 失败: {e}")
    
    def stop_ins_output(self):
        """停止 INS 数据输出"""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.sendto(INS_CMD_STOP, (INS_IP, INS_CMD_PORT))
            sock.close()
            self.get_logger().info("已发送 INS 停止命令")
        except Exception as e:
            self.get_logger().error(f"停止 INS 失败: {e}")
    
    def connect_ins(self):
        """连接 INS 数据端口"""
        if self.running:
            self.get_logger().warn("INS 数据接收已在运行")
            return
        
        try:
            self.ins_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.ins_socket.bind(('0.0.0.0', INS_DATA_PORT))
            self.ins_socket.settimeout(1.0)
            self.running = True
            
            self.ins_thread = threading.Thread(target=self.ins_data_loop)
            self.ins_thread.daemon = True
            self.ins_thread.start()
            
            self.get_logger().info(f"已连接到 INS 数据端口 {INS_DATA_PORT}")
        except Exception as e:
            self.get_logger().error(f"连接 INS 失败: {e}")
    
    def disconnect_ins(self):
        """断开 INS 数据连接"""
        self.running = False
        if self.ins_thread:
            self.ins_thread.join(timeout=2)
        if self.ins_socket:
            self.ins_socket.close()
            self.ins_socket = None
        self.get_logger().info("已断开 INS 数据连接")
    
    def ins_data_loop(self):
        """INS 数据接收循环"""
        while self.running and rclpy.ok():
            try:
                data, addr = self.ins_socket.recvfrom(256)
                if len(data) >= 202:
                    self.parse_ins_frame(data)
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    self.get_logger().error(f"INS 数据接收错误: {e}")
    
    def parse_ins_frame(self, data):
        """解析 INS 数据帧"""
        try:
            # 检查帧头帧尾
            if data[0] != 0x5A or data[1] != 0xA5 or data[201] != 0x55:
                return
            
            # 工作状态
            status_byte = data[2]
            alignment = (status_byte >> 1) & 0x03
            
            # 姿态角 (float32, 小端)
            pitch = struct.unpack('<f', data[33:37])[0]
            roll = struct.unpack('<f', data[37:41])[0]
            yaw = struct.unpack('<f', data[41:45])[0]
            
            # 速度
            ve = struct.unpack('<f', data[45:49])[0]
            vn = struct.unpack('<f', data[49:53])[0]
            vd = struct.unpack('<f', data[53:57])[0]
            
            # 位置 (int32 * 1e-7)
            lat_raw = struct.unpack('<i', data[177:181])[0]
            lon_raw = struct.unpack('<i', data[181:185])[0]
            lat = lat_raw * 1e-7
            lon = lon_raw * 1e-7
            
            # 高度
            alt = struct.unpack('<f', data[77:81])[0]
            
            # GNSS 信息
            sats = data[5]
            temp = data[198]
            
            # 更新状态
            self.current_status.update({
                'alignment': alignment,
                'pitch': pitch,
                'roll': roll,
                'yaw': yaw,
                've': ve,
                'vn': vn,
                'vd': vd,
                'lat': lat,
                'lon': lon,
                'alt': alt,
                'sats': sats,
                'temp': temp
            })
            
            # 发布 ROS2 消息
            self.publish_data()
            
        except Exception as e:
            self.get_logger().error(f"解析 INS 帧错误: {e}")
    
    def publish_data(self):
        """发布 ROS2 消息"""
        # 姿态
        attitude = Vector3()
        attitude.x = self.current_status['pitch']
        attitude.y = self.current_status['roll']
        attitude.z = self.current_status['yaw']
        self.attitude_pub.publish(attitude)
        
        # 速度
        velocity = Vector3()
        velocity.x = self.current_status['ve']
        velocity.y = self.current_status['vn']
        velocity.z = self.current_status['vd']
        self.velocity_pub.publish(velocity)
        
        # 位置
        position = Vector3()
        position.x = self.current_status['lat']
        position.y = self.current_status['lon']
        position.z = self.current_status['alt']
        self.position_pub.publish(position)
        
        # 状态 (JSON)
        status_msg = String()
        status_msg.data = json.dumps(self.current_status)
        self.status_pub.publish(status_msg)
    
    def set_position(self, lat, lon):
        """设置初始位置 (预留接口)"""
        self.get_logger().info(f"设置初始位置: lat={lat}, lon={lon}")
        # 这里可以添加发送位置设置命令到 INS 的逻辑
    
    def destroy_node(self):
        """清理资源"""
        self.disconnect_ins()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RK3588INSController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
