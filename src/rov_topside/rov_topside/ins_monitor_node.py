#!/usr/bin/env python3
"""
ROS2 Foxy INS Monitor Node for Ubuntu VM (上位机)
订阅 /ins/data 话题，显示 INS 解析数据
同时可以通过 /ins/command 发送控制命令
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from rov_ins_interfaces.msg import InsData, InsCommand

import sys
import threading


class INSMonitorNode(Node):
    """INS 监控节点 - 在上位机显示数据并发送控制命令"""
    
    def __init__(self):
        super().__init__('ins_monitor_node')
        
        # QoS 配置
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 创建订阅者
        self.data_sub = self.create_subscription(
            InsData, '/ins/data', self.data_callback, qos
        )
        
        # 创建发布者 - 发送命令
        self.cmd_pub = self.create_publisher(InsCommand, '/ins/command', 10)
        
        # 统计数据
        self.msg_count = 0
        self.last_time = self.get_clock().now()
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('INS Monitor Node 已启动')
        self.get_logger().info('等待 INS 数据...')
        self.get_logger().info('=' * 60)
        
        # 启动交互式命令线程
        self.cmd_thread = threading.Thread(target=self.command_loop)
        self.cmd_thread.daemon = True
        self.cmd_thread.start()
    
    def data_callback(self, msg):
        """接收 INS 数据并显示"""
        self.msg_count += 1
        
        # 每 50 条消息显示一次完整信息
        if self.msg_count % 50 == 1:
            self.print_full_data(msg)
        else:
            # 简要显示
            self.print_brief_data(msg)
    
    def print_full_data(self, msg):
        """打印完整数据"""
        print('\n' + '=' * 70)
        print(f'【INS 数据】序列号: {self.msg_count} | 时间戳: {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}')
        print('-' * 70)
        
        # 位置
        print(f'  位置: 纬度={msg.latitude:.6f}°, 经度={msg.longitude:.6f}°, 高度={msg.altitude:.2f}m')
        
        # 姿态
        print(f'  姿态: 横滚={msg.roll:.2f}°, 俯仰={msg.pitch:.2f}°, 航向={msg.yaw:.2f}°')
        
        # 速度
        print(f'  速度: 北={msg.north_vel:.3f}, 东={msg.east_vel:.3f}, 下={msg.down_vel:.3f} m/s')
        
        # 角速度
        print(f'  角速度: X={msg.gyro_x:.3f}, Y={msg.gyro_y:.3f}, Z={msg.gyro_z:.3f} deg/s')
        
        # 加速度
        print(f'  加速度: X={msg.acc_x:.3f}, Y={msg.acc_y:.3f}, Z={msg.acc_z:.3f} m/s²')
        
        # 状态
        print('-' * 70)
        print(f'  工作状态: {msg.work_status_desc}')
        print(f'  DVL校准: {msg.dvl_calib_desc}')
        print(f'  GNSS定位: {msg.gnss_pos_desc}')
        print(f'  组合导航: {msg.combination_desc}')
        print('=' * 70)
    
    def print_brief_data(self, msg):
        """打印简要数据 - 单行显示"""
        # 使用 \r 实现同一行更新
        line = (f'\r[{self.msg_count:5d}] '
                f'Pos: ({msg.latitude:.4f}, {msg.longitude:.4f}) | '
                f'Att: ({msg.roll:.1f}, {msg.pitch:.1f}, {msg.yaw:.1f}) | '
                f'Vel: ({msg.north_vel:.2f}, {msg.east_vel:.2f}) | '
                f'Status: {msg.work_status_desc[:20]}')
        
        # 截断到终端宽度
        max_len = 100
        if len(line) > max_len:
            line = line[:max_len-3] + '...'
        
        print(line, end='', flush=True)
    
    def command_loop(self):
        """交互式命令循环 - 在独立线程中运行"""
        print('\n')
        print('=' * 70)
        print('命令控制台已启动')
        print('可用命令:')
        print('  start       - 启动 INS 数据输出')
        print('  stop        - 停止 INS 数据输出')
        print('  lat XX.XX   - 设置纬度 (例如: lat 31.23)')
        print('  lon XXX.XXX - 设置经度 (例如: lon 121.47)')
        print('  status      - 显示当前状态')
        print('  help        - 显示帮助')
        print('  quit        - 退出程序')
        print('=' * 70)
        
        while rclpy.ok():
            try:
                cmd = input('\n[INS] > ').strip().lower()
                
                if not cmd:
                    continue
                
                if cmd == 'quit' or cmd == 'exit':
                    self.get_logger().info('正在退出...')
                    rclpy.shutdown()
                    break
                
                elif cmd == 'help':
                    self.print_help()
                
                elif cmd == 'status':
                    self.print_status()
                
                elif cmd == 'start':
                    self.send_command('start')
                    print('✓ 已发送启动命令')
                
                elif cmd == 'stop':
                    self.send_command('stop')
                    print('✓ 已发送停止命令')
                
                elif cmd.startswith('lat '):
                    try:
                        lat = float(cmd.split()[1])
                        self.send_lat_command(lat)
                        print(f'✓ 已发送纬度设置: {lat}')
                    except (ValueError, IndexError):
                        print('✗ 格式错误，使用: lat XX.XX')
                
                elif cmd.startswith('lon '):
                    try:
                        lon = float(cmd.split()[1])
                        self.send_lon_command(lon)
                        print(f'✓ 已发送经度设置: {lon}')
                    except (ValueError, IndexError):
                        print('✗ 格式错误，使用: lon XXX.XXX')
                
                else:
                    print(f'✗ 未知命令: {cmd}')
                    print('输入 help 查看可用命令')
                    
            except EOFError:
                break
            except Exception as e:
                print(f'命令错误: {e}')
    
    def send_command(self, cmd_str):
        """发送命令到 INS"""
        msg = InsCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.command = cmd_str
        self.cmd_pub.publish(msg)
    
    def send_lat_command(self, lat):
        """发送纬度设置命令"""
        msg = InsCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.command = 'set_lat'
        msg.latitude = lat
        self.cmd_pub.publish(msg)
    
    def send_lon_command(self, lon):
        """发送经度设置命令"""
        msg = InsCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.command = 'set_lon'
        msg.longitude = lon
        self.cmd_pub.publish(msg)
    
    def print_help(self):
        """打印帮助信息"""
        print('\n' + '=' * 70)
        print('命令帮助')
        print('=' * 70)
        print('  start       - 发送启动命令，INS 开始输出数据')
        print('  stop        - 发送停止命令，INS 停止输出数据')
        print('  lat XX.XX   - 设置初始纬度（度）')
        print('  lon XXX.XXX - 设置初始经度（度）')
        print('  status      - 显示接收统计信息')
        print('  help        - 显示此帮助')
        print('  quit/exit   - 退出程序')
        print('=' * 70)
    
    def print_status(self):
        """打印状态信息"""
        now = self.get_clock().now()
        elapsed = (now - self.last_time).nanoseconds / 1e9
        rate = self.msg_count / elapsed if elapsed > 0 else 0
        
        print('\n' + '=' * 70)
        print('状态统计')
        print('=' * 70)
        print(f'  接收消息数: {self.msg_count}')
        print(f'  运行时间: {elapsed:.1f} 秒')
        print(f'  平均频率: {rate:.1f} Hz')
        print('=' * 70)


def main(args=None):
    rclpy.init(args=args)
    node = INSMonitorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n\n用户中断')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
