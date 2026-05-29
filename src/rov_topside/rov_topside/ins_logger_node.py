#!/usr/bin/env python3
"""
ROS2 Foxy INS Logger Node
记录 INS 数据到 CSV 文件
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from rov_ins_interfaces.msg import InsData

import csv
import os
from datetime import datetime


class INSLoggerNode(Node):
    """INS 数据记录节点"""
    
    def __init__(self):
        super().__init__('ins_logger_node')
        
        # 声明参数
        self.declare_parameter('output_dir', '~/ins_logs')
        self.declare_parameter('filename_prefix', 'ins_data')
        
        output_dir = os.path.expanduser(
            self.get_parameter('output_dir').value
        )
        prefix = self.get_parameter('filename_prefix').value
        
        # 创建输出目录
        os.makedirs(output_dir, exist_ok=True)
        
        # 生成文件名
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.filename = os.path.join(
            output_dir, f'{prefix}_{timestamp}.csv'
        )
        
        # QoS 配置
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 创建订阅者
        self.sub = self.create_subscription(
            InsData, '/ins/data', self.data_callback, qos
        )
        
        # 初始化 CSV
        self.csv_file = open(self.filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'timestamp_sec', 'timestamp_nsec',
            'latitude', 'longitude', 'altitude',
            'roll', 'pitch', 'yaw',
            'north_vel', 'east_vel', 'down_vel',
            'gyro_x', 'gyro_y', 'gyro_z',
            'acc_x', 'acc_y', 'acc_z',
            'work_status', 'dvl_calib_status',
            'gnss_pos_status', 'combination_status'
        ])
        
        self.msg_count = 0
        
        self.get_logger().info(f'INS Logger 已启动')
        self.get_logger().info(f'输出文件: {self.filename}')
    
    def data_callback(self, msg):
        """接收数据并写入 CSV"""
        self.csv_writer.writerow([
            msg.header.stamp.sec, msg.header.stamp.nanosec,
            msg.latitude, msg.longitude, msg.altitude,
            msg.roll, msg.pitch, msg.yaw,
            msg.north_vel, msg.east_vel, msg.down_vel,
            msg.gyro_x, msg.gyro_y, msg.gyro_z,
            msg.acc_x, msg.acc_y, msg.acc_z,
            msg.work_status, msg.dvl_calib_status,
            msg.gnss_pos_status, msg.combination_status
        ])
        
        self.msg_count += 1
        
        # 每 100 条刷新一次
        if self.msg_count % 100 == 0:
            self.csv_file.flush()
            self.get_logger().info(f'已记录 {self.msg_count} 条数据')
    
    def destroy_node(self):
        """节点销毁时关闭文件"""
        self.csv_file.close()
        self.get_logger().info(f'数据已保存到: {self.filename}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = INSLoggerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
