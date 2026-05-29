#!/usr/bin/env python3
"""
INS Control Panel - Interactive TUI
Runs on VM, provides interactive control interface
"""

import sys
import rclpy
from rclpy.node import Node
from rov_ins_interface.srv import INSCommand
from std_msgs.msg import Int32


class INSControlPanel(Node):
    def __init__(self):
        super().__init__('ins_control_panel')
        self.client = self.create_client(INSCommand, '/ins/control')

        # Subscribe to status topics for display
        self.align_status = 0
        self.work_status = 0
        self.current_lat = 0.0
        self.current_lon = 0.0

        self.create_subscription(Int32, '/ins/align_status', self._on_align_status, 10)
        self.create_subscription(Int32, '/ins/work_status', self._on_work_status, 10)
        self.create_subscription(Int32, '/ins/gnss_satellites', self._on_gnss_sats, 10)

        self.gnss_sats = 0

    def _on_align_status(self, msg):
        self.align_status = msg.data

    def _on_work_status(self, msg):
        self.work_status = msg.data

    def _on_gnss_sats(self, msg):
        self.gnss_sats = msg.data

    def wait_for_service(self, timeout_sec=5.0):
        self.get_logger().info('等待 INS 控制服务...')
        return self.client.wait_for_service(timeout_sec=timeout_sec)

    def print_status(self):
        """Print current status"""
        align_names = {0: '监控', 1: '粗对准', 2: '精对准', 3: 'INS导航'}
        align_str = align_names.get(self.align_status, f'未知({self.align_status})')

        print("\n" + "=" * 50)
        print("           INS 控制面板")
        print("=" * 50)
        print(f"  对准状态: {align_str}")
        print(f"  工作状态: 0x{self.work_status:02X}")
        print(f"  卫星数量: {self.gnss_sats}")
        print("=" * 50)

    def print_menu(self):
        """Print menu"""
        print("\n操作选项:")
        print("  1. 停止 INS (进入监控状态)")
        print("  2. 设置初始位置")
        print("  3. 启动 INS (开始对准)")
        print("  4. 获取详细状态")
        print("  5. 刷新显示")
        print("  0. 退出")
        print()

    def send_command(self, cmd_type, lat=0.0, lon=0.0, alt=0.0):
        """Send command to INS"""
        req = INSCommand.Request()
        req.cmd_type = cmd_type
        req.latitude = lat
        req.longitude = lon
        req.altitude = alt

        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is None:
            print("✗ 命令超时，请检查连接")
            return False

        resp = future.result()
        if resp.success:
            print(f"✓ {resp.message}")
            if cmd_type == INSCommand.Request.CMD_GET_STATUS:
                align_names = {0: '监控', 1: '粗对准', 2: '精对准', 3: 'INS导航'}
                align_str = align_names.get(resp.align_status, f'未知({resp.align_status})')
                print(f"  对准状态: {align_str}")
                print(f"  位置: {resp.current_lat:.7f}, {resp.current_lon:.7f}")
            return True
        else:
            print(f"✗ 失败: {resp.message}")
            return False

    def run(self):
        """Main loop"""
        if not self.wait_for_service():
            print("错误: 无法连接到 INS 控制服务")
            print("请检查：")
            print("  1. RK3588 上的驱动是否已启动")
            print("  2. ROS_DOMAIN_ID 是否一致")
            return

        print("\n已连接到 INS 控制服务")

        while rclpy.ok():
            self.print_status()
            self.print_menu()

            try:
                choice = input("请选择操作 [0-5]: ").strip()
            except (EOFError, KeyboardInterrupt):
                print("\n退出")
                break

            if choice == '0':
                print("退出")
                break

            elif choice == '1':
                print("\n停止 INS...")
                self.send_command(INSCommand.Request.CMD_STOP)

            elif choice == '2':
                print("\n设置初始位置")
                try:
                    lat = float(input("  纬度 (度): ").strip())
                    lon = float(input("  经度 (度): ").strip())
                    alt_input = input("  高度 (米) [默认0]: ").strip()
                    alt = float(alt_input) if alt_input else 0.0
                    self.send_command(INSCommand.Request.CMD_SET_POS, lat, lon, alt)
                except ValueError:
                    print("✗ 输入无效，请输入数字")

            elif choice == '3':
                print("\n启动 INS...")
                self.send_command(INSCommand.Request.CMD_START)

            elif choice == '4':
                print("\n获取状态...")
                self.send_command(INSCommand.Request.CMD_GET_STATUS)

            elif choice == '5':
                pass  # Just refresh

            else:
                print("✗ 无效选项")

            # Spin once to update subscriptions
            rclpy.spin_once(self, timeout_sec=0.1)


def main(args=None):
    rclpy.init(args=args)
    node = INSControlPanel()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
