#!/usr/bin/env python3
"""
ROS2 Foxy INS Driver Node for RK3588
封装现有的 ins_demo_v1.9.py 功能，通过 ROS2 Topic 发布解析数据
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import socket
import struct
import threading
import queue
import time
import sys
import os

# 导入自定义消息
from rov_ins_interfaces.msg import InsData, InsCommand


class INSDriverNode(Node):
    """INS 驱动节点 - 接收 UDP 数据，解析后发布到 ROS2"""
    
    # INS 配置
    LOCAL_IP = "192.168.0.99"
    LOCAL_PORT = 8008
    INS_IP = "192.168.0.7"
    INS_CMD_PORT = 8007
    
    # INS 命令帧
    START_CMD = bytes([0x5A, 0xA5, 0x01, 0x00, 0x00, 0x00, 0x01, 0x55])
    STOP_CMD = bytes([0x5A, 0xA5, 0x02, 0x00, 0x00, 0x00, 0x02, 0x55])
    
    def __init__(self):
        super().__init__('ins_driver_node')
        
        # 声明参数
        self.declare_parameter('local_ip', self.LOCAL_IP)
        self.declare_parameter('local_port', self.LOCAL_PORT)
        self.declare_parameter('ins_ip', self.INS_IP)
        self.declare_parameter('ins_cmd_port', self.INS_CMD_PORT)
        self.declare_parameter('publish_rate', 50.0)  # Hz
        
        # 获取参数
        self.local_ip = self.get_parameter('local_ip').value
        self.local_port = self.get_parameter('local_port').value
        self.ins_ip = self.get_parameter('ins_ip').value
        self.ins_cmd_port = self.get_parameter('ins_cmd_port').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # QoS 配置 - 传感器数据用 Best Effort
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 创建发布者
        self.data_pub = self.create_publisher(InsData, '/ins/data', qos)
        
        # 创建订阅者 - 接收控制命令
        self.cmd_sub = self.create_subscription(
            InsCommand, '/ins/command', self.command_callback, 10
        )
        
        # 状态变量
        self.sock = None
        self.running = False
        self.frame_queue = queue.Queue(maxsize=100)
        self.stop_event = threading.Event()
        
        # 线程
        self.net_thread = None
        self.parse_thread = None
        
        self.get_logger().info(f'INS Driver Node 初始化')
        self.get_logger().info(f'本地: {self.local_ip}:{self.local_port}')
        self.get_logger().info(f'INS: {self.ins_ip}:{self.ins_cmd_port}')
        
        # 启动网络接收
        self.start_network()
    
    def start_network(self):
        """启动网络接收线程"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind((self.local_ip, self.local_port))
            self.sock.settimeout(1.0)
            self.get_logger().info(f'UDP 监听已启动: {self.local_ip}:{self.local_port}')
            
            self.running = True
            self.stop_event.clear()
            
            # 启动网络接收线程
            self.net_thread = threading.Thread(target=self.network_loop)
            self.net_thread.daemon = True
            self.net_thread.start()
            
            # 启动解析线程
            self.parse_thread = threading.Thread(target=self.parse_loop)
            self.parse_thread.daemon = True
            self.parse_thread.start()
            
        except Exception as e:
            self.get_logger().error(f'网络启动失败: {e}')
    
    def network_loop(self):
        """网络接收循环 - 在独立线程中运行"""
        self.get_logger().info('网络接收线程已启动')
        
        while not self.stop_event.is_set():
            try:
                data, addr = self.sock.recvfrom(2048)
                
                # 查找帧头 0x50
                idx = data.find(b'\x50')
                if idx != -1 and len(data) >= idx + 78:
                    frame = data[idx:idx+78]
                    try:
                        self.frame_queue.put_nowait(frame)
                    except queue.Full:
                        # 队列满，丢弃最旧数据
                        try:
                            self.frame_queue.get_nowait()
                            self.frame_queue.put_nowait(frame)
                        except queue.Empty:
                            pass
                        
            except socket.timeout:
                continue
            except Exception as e:
                if not self.stop_event.is_set():
                    self.get_logger().warning(f'网络接收错误: {e}')
        
        self.get_logger().info('网络接收线程已停止')
    
    def parse_loop(self):
        """解析循环 - 在独立线程中运行"""
        self.get_logger().info('数据解析线程已启动')
        
        while not self.stop_event.is_set():
            try:
                frame = self.frame_queue.get(timeout=0.1)
                ins_msg = self.parse_frame_0x50(frame)
                if ins_msg:
                    self.data_pub.publish(ins_msg)
            except queue.Empty:
                continue
            except Exception as e:
                if not self.stop_event.is_set():
                    self.get_logger().warning(f'解析错误: {e}')
        
        self.get_logger().info('数据解析线程已停止')
    
    def parse_frame_0x50(self, frame):
        """解析 0x50 数据帧 - 基于 ins_demo_v1.9.py"""
        if len(frame) < 78:
            return None
        
        try:
            msg = InsData()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'ins_link'
            
            # 解析数据结构 (小端序)
            # 帧头: 0x50 (1字节)
            # 纬度: 4字节 float (度)
            msg.latitude = struct.unpack('<f', frame[1:5])[0]
            # 经度: 4字节 float (度)
            msg.longitude = struct.unpack('<f', frame[5:9])[0]
            # 高度: 4字节 float (米)
            msg.altitude = struct.unpack('<f', frame[9:13])[0]
            
            # 姿态角 (度)
            msg.roll = struct.unpack('<f', frame[13:17])[0]
            msg.pitch = struct.unpack('<f', frame[17:21])[0]
            msg.yaw = struct.unpack('<f', frame[21:25])[0]
            
            # 速度 (m/s)
            msg.north_vel = struct.unpack('<f', frame[25:29])[0]
            msg.east_vel = struct.unpack('<f', frame[29:33])[0]
            msg.down_vel = struct.unpack('<f', frame[33:37])[0]
            
            # 角速度 (deg/s)
            msg.gyro_x = struct.unpack('<f', frame[37:41])[0]
            msg.gyro_y = struct.unpack('<f', frame[41:45])[0]
            msg.gyro_z = struct.unpack('<f', frame[45:49])[0]
            
            # 加速度 (m/s²)
            msg.acc_x = struct.unpack('<f', frame[49:53])[0]
            msg.acc_y = struct.unpack('<f', frame[53:57])[0]
            msg.acc_z = struct.unpack('<f', frame[57:61])[0]
            
            # 状态字节
            msg.work_status = frame[61]
            msg.dvl_calib_status = frame[62]
            msg.gnss_pos_status = frame[63]
            msg.combination_status = frame[64]
            
            # 状态描述
            msg.work_status_desc = self.parse_work_status(msg.work_status)
            msg.dvl_calib_desc = self.parse_dvl_calib_status(msg.dvl_calib_status)
            msg.gnss_pos_desc = self.parse_gnss_pos_status(msg.gnss_pos_status)
            msg.combination_desc = self.parse_combination_status(msg.combination_status)
            
            # 原始数据
            msg.raw_frame = list(frame)
            msg.valid = True
            
            return msg
            
        except Exception as e:
            self.get_logger().warning(f'帧解析失败: {e}')
            return None
    
    def parse_work_status(self, status_byte):
        """解析工作状态"""
        res = []
        if status_byte & 0x80: res.append("准备启动")
        if status_byte & 0x40: res.append("准备停止")
        if status_byte & 0x20: res.append("正在启动")
        if status_byte & 0x10: res.append("正在停止")
        if status_byte & 0x08: res.append("INS错误")
        if status_byte & 0x04: res.append("IMU未校准")
        
        state_code = status_byte & 0x03
        state_map = {0: "待机", 1: "启动中", 2: "运行中", 3: "INS错误"}
        res.append(f"状态: {state_map.get(state_code, '未知')}")
        return ", ".join(res) if res else "正常"
    
    def parse_dvl_calib_status(self, status_byte):
        """解析DVL校准状态"""
        calib_map = {0: "未校准", 1: "校准中", 2: "已校准"}
        return calib_map.get(status_byte, "未知")
    
    def parse_gnss_pos_status(self, status_byte):
        """解析GNSS定位状态"""
        gnss_map = {
            0: "无定位",
            1: "单点定位(SPS)",
            2: "差分定位(DGNSS)",
            4: "RTK浮点解",
            5: "RTK固定解"
        }
        if status_byte > 5:
            return f"保留值({status_byte})"
        return gnss_map.get(status_byte, "未知")
    
    def parse_combination_status(self, status_byte):
        """解析组合导航状态"""
        res = []
        if status_byte & 0x01: res.append("GNSS有效")
        if status_byte & 0x02: res.append("深度传感器有效")
        if status_byte & 0x04: res.append("DVL有效")
        if status_byte & 0x08: res.append("USBL有效")
        return ", ".join(res) if res else "无有效传感器"
    
    def command_callback(self, msg):
        """处理接收到的控制命令"""
        self.get_logger().info(f'收到命令: {msg.command}')
        
        if msg.command == 'start':
            self.send_ins_command(self.START_CMD)
        elif msg.command == 'stop':
            self.send_ins_command(self.STOP_CMD)
        elif msg.command == 'set_lat':
            self.send_lat_command(msg.latitude)
        elif msg.command == 'set_lon':
            self.send_lon_command(msg.longitude)
        else:
            self.get_logger().warning(f'未知命令: {msg.command}')
    
    def send_ins_command(self, cmd_bytes):
        """发送命令到 INS"""
        try:
            cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            cmd_sock.sendto(cmd_bytes, (self.ins_ip, self.ins_cmd_port))
            cmd_sock.close()
            self.get_logger().info(f'命令已发送: {cmd_bytes.hex()}')
        except Exception as e:
            self.get_logger().error(f'命令发送失败: {e}')
    
    def send_lat_command(self, lat_deg):
        """发送纬度设置命令"""
        payload = b'\x4C' + struct.pack('<f', lat_deg)
        ck = 0
        for b in payload:
            ck ^= b
        cmd = b'\x5A\xA5' + payload + bytes([ck, 0x55])
        self.send_ins_command(cmd)
        self.get_logger().info(f'纬度设置: {lat_deg}')
    
    def send_lon_command(self, lon_deg):
        """发送经度设置命令"""
        payload = b'\x54' + struct.pack('<f', lon_deg)
        ck = 0
        for b in payload:
            ck ^= b
        cmd = b'\x5A\xA5' + payload + bytes([ck, 0x55])
        self.send_ins_command(cmd)
        self.get_logger().info(f'经度设置: {lon_deg}')
    
    def destroy_node(self):
        """节点销毁时的清理"""
        self.get_logger().info('正在关闭 INS Driver...')
        self.running = False
        self.stop_event.set()
        
        if self.sock:
            self.sock.close()
        
        if self.net_thread:
            self.net_thread.join(timeout=2.0)
        if self.parse_thread:
            self.parse_thread.join(timeout=2.0)
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = INSDriverNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('用户中断')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
