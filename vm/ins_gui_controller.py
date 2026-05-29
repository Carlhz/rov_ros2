#!/usr/bin/env python3
"""
VM 端 INS GUI 控制器
通过 ROS2 话题控制 RK3588 上的 INS 驱动
"""

import os
os.environ['LIBGL_ALWAYS_SOFTWARE'] = '1'
os.environ['GDK_BACKEND'] = 'x11'

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
import json
import tkinter as tk
from tkinter import ttk, scrolledtext
import threading


class INSGUIController(Node):
    """INS GUI 控制器节点"""
    
    def __init__(self):
        super().__init__('ins_gui_controller')
        
        # 创建发布者 - 发送控制命令
        self.cmd_pub = self.create_publisher(String, '/ins/command', 10)
        
        # 创建订阅者 - 接收 INS 数据
        self.attitude_sub = self.create_subscription(
            Vector3, '/ins/attitude', self.attitude_callback, 10)
        self.velocity_sub = self.create_subscription(
            Vector3, '/ins/velocity', self.velocity_callback, 10)
        self.position_sub = self.create_subscription(
            Vector3, '/ins/position', self.position_callback, 10)
        self.status_sub = self.create_subscription(
            String, '/ins/status', self.status_callback, 10)
        
        # 数据存储
        self.current_data = {
            'pitch': 0.0, 'roll': 0.0, 'yaw': 0.0,
            've': 0.0, 'vn': 0.0, 'vd': 0.0,
            'lat': 0.0, 'lon': 0.0, 'alt': 0.0,
            'alignment': 0, 'sats': 0, 'temp': 0
        }
        
        self.get_logger().info("INS GUI Controller 已启动")
    
    def send_command(self, cmd_dict):
        """发送控制命令"""
        msg = String()
        msg.data = json.dumps(cmd_dict)
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"发送命令: {cmd_dict}")
    
    def attitude_callback(self, msg):
        self.current_data['pitch'] = msg.x
        self.current_data['roll'] = msg.y
        self.current_data['yaw'] = msg.z
    
    def velocity_callback(self, msg):
        self.current_data['ve'] = msg.x
        self.current_data['vn'] = msg.y
        self.current_data['vd'] = msg.z
    
    def position_callback(self, msg):
        self.current_data['lat'] = msg.x
        self.current_data['lon'] = msg.y
        self.current_data['alt'] = msg.z
    
    def status_callback(self, msg):
        try:
            status = json.loads(msg.data)
            self.current_data.update(status)
        except:
            pass


class INSGUI:
    """INS 控制 GUI"""
    
    ALIGNMENT_NAMES = {
        0: "未对准",
        1: "粗对准",
        2: "精对准", 
        3: "INS导航模式"
    }
    
    def __init__(self, root, ros_node):
        self.root = root
        self.node = ros_node
        self.root.title("INS 控制系统")
        self.root.geometry("700x500")
        
        # 设置字体
        self.font = ('DejaVu Sans', 10)
        self.font_mono = ('DejaVu Sans Mono', 9)
        
        self._build_ui()
        self._start_update_loop()
    
    def _build_ui(self):
        # 主框架
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 控制按钮区域
        ctrl_frame = ttk.LabelFrame(main_frame, text="控制", padding="10")
        ctrl_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        
        ttk.Button(ctrl_frame, text="连接INS", command=self.connect_ins).grid(row=0, column=0, padx=5)
        ttk.Button(ctrl_frame, text="断开INS", command=self.disconnect_ins).grid(row=0, column=1, padx=5)
        ttk.Button(ctrl_frame, text="启动输出", command=self.start_output).grid(row=0, column=2, padx=5)
        ttk.Button(ctrl_frame, text="停止输出", command=self.stop_output).grid(row=0, column=3, padx=5)
        
        # 状态显示区域
        status_frame = ttk.LabelFrame(main_frame, text="状态", padding="10")
        status_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5)
        
        self.status_labels = {}
        status_items = [
            ('alignment', '对准状态', ''),
            ('sats', '卫星数', '颗'),
            ('temp', '温度', 'C'),
        ]
        for i, (key, name, unit) in enumerate(status_items):
            ttk.Label(status_frame, text=f"{name}:", font=self.font).grid(row=i, column=0, sticky=tk.W)
            self.status_labels[key] = ttk.Label(status_frame, text="--", font=self.font_mono)
            self.status_labels[key].grid(row=i, column=1, sticky=tk.W, padx=5)
            if unit:
                ttk.Label(status_frame, text=unit, font=self.font).grid(row=i, column=2, sticky=tk.W)
        
        # 姿态显示
        att_frame = ttk.LabelFrame(main_frame, text="姿态", padding="10")
        att_frame.grid(row=1, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5, padx=5)
        
        self.att_labels = {}
        att_items = [
            ('pitch', '俯仰角', 'deg'),
            ('roll', '横滚角', 'deg'),
            ('yaw', '航向角', 'deg'),
        ]
        for i, (key, name, unit) in enumerate(att_items):
            ttk.Label(att_frame, text=f"{name}:", font=self.font).grid(row=i, column=0, sticky=tk.W)
            self.att_labels[key] = ttk.Label(att_frame, text="--", font=self.font_mono)
            self.att_labels[key].grid(row=i, column=1, sticky=tk.W, padx=5)
            ttk.Label(att_frame, text=unit, font=self.font).grid(row=i, column=2, sticky=tk.W)
        
        # 速度显示
        vel_frame = ttk.LabelFrame(main_frame, text="速度", padding="10")
        vel_frame.grid(row=2, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5)
        
        self.vel_labels = {}
        vel_items = [
            ('ve', '东向速度', 'm/s'),
            ('vn', '北向速度', 'm/s'),
            ('vd', '垂直速度', 'm/s'),
        ]
        for i, (key, name, unit) in enumerate(vel_items):
            ttk.Label(vel_frame, text=f"{name}:", font=self.font).grid(row=i, column=0, sticky=tk.W)
            self.vel_labels[key] = ttk.Label(vel_frame, text="--", font=self.font_mono)
            self.vel_labels[key].grid(row=i, column=1, sticky=tk.W, padx=5)
            ttk.Label(vel_frame, text=unit, font=self.font).grid(row=i, column=2, sticky=tk.W)
        
        # 位置显示
        pos_frame = ttk.LabelFrame(main_frame, text="位置", padding="10")
        pos_frame.grid(row=2, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5, padx=5)
        
        self.pos_labels = {}
        pos_items = [
            ('lat', '纬度', 'deg'),
            ('lon', '经度', 'deg'),
            ('alt', '高度', 'm'),
        ]
        for i, (key, name, unit) in enumerate(pos_items):
            ttk.Label(pos_frame, text=f"{name}:", font=self.font).grid(row=i, column=0, sticky=tk.W)
            self.pos_labels[key] = ttk.Label(pos_frame, text="--", font=self.font_mono)
            self.pos_labels[key].grid(row=i, column=1, sticky=tk.W, padx=5)
            ttk.Label(pos_frame, text=unit, font=self.font).grid(row=i, column=2, sticky=tk.W)
        
        # 日志区域
        log_frame = ttk.LabelFrame(main_frame, text="日志", padding="5")
        log_frame.grid(row=3, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5)
        
        self.log_text = scrolledtext.ScrolledText(
            log_frame, height=6, font=self.font_mono, wrap=tk.WORD)
        self.log_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 配置网格权重
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(3, weight=1)
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
    
    def _start_update_loop(self):
        """启动数据更新循环"""
        self._update_ui()
    
    def _update_ui(self):
        """更新 UI 显示"""
        data = self.node.current_data
        
        # 更新状态
        alignment = data.get('alignment', 0)
        self.status_labels['alignment'].config(text=self.ALIGNMENT_NAMES.get(alignment, "未知"))
        self.status_labels['sats'].config(text=str(data.get('sats', 0)))
        self.status_labels['temp'].config(text=f"{data.get('temp', 0):.1f}")
        
        # 更新姿态
        for key in ['pitch', 'roll', 'yaw']:
            value = data.get(key, 0.0)
            self.att_labels[key].config(text=f"{value:.2f}")
        
        # 更新速度
        for key in ['ve', 'vn', 'vd']:
            value = data.get(key, 0.0)
            self.vel_labels[key].config(text=f"{value:.3f}")
        
        # 更新位置
        for key in ['lat', 'lon', 'alt']:
            value = data.get(key, 0.0)
            self.pos_labels[key].config(text=f"{value:.6f}" if key in ['lat', 'lon'] else f"{value:.2f}")
        
        self.root.after(100, self._update_ui)  # 10Hz 更新
    
    def log(self, message):
        """添加日志"""
        self.log_text.insert(tk.END, f"{message}\n")
        self.log_text.see(tk.END)
    
    def connect_ins(self):
        self.node.send_command({'action': 'connect'})
        self.log("[发送] 连接 INS")
    
    def disconnect_ins(self):
        self.node.send_command({'action': 'disconnect'})
        self.log("[发送] 断开 INS")
    
    def start_output(self):
        self.node.send_command({'action': 'start'})
        self.log("[发送] 启动 INS 输出")
    
    def stop_output(self):
        self.node.send_command({'action': 'stop'})
        self.log("[发送] 停止 INS 输出")


def main():
    rclpy.init()
    node = INSGUIController()
    
    # 在后台线程运行 ROS2
    def ros_spin():
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    
    ros_thread = threading.Thread(target=ros_spin)
    ros_thread.daemon = True
    ros_thread.start()
    
    # 启动 GUI
    root = tk.Tk()
    app = INSGUI(root, node)
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
