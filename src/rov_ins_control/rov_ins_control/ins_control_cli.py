#!/usr/bin/env python3
"""
INS Control CLI Tool
Runs on VM, sends commands to RK3588 INS driver
Usage:
  ros2 run rov_ins_control ins_control_cli -- stop
  ros2 run rov_ins_control ins_control_cli -- start
  ros2 run rov_ins_control ins_control_cli -- setpos 31.234567 121.456789
  ros2 run rov_ins_control ins_control_cli -- status
"""

import sys
import rclpy
from rclpy.node import Node
from rov_ins_interface.srv import INSCommand


class INSControlCLI(Node):
    def __init__(self):
        super().__init__('ins_control_cli')
        self.client = self.create_client(INSCommand, '/ins/control')

    def wait_for_service(self, timeout_sec=5.0):
        """Wait for service to be available"""
        self.get_logger().info('等待 INS 控制服务...')
        if not self.client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error('服务不可用，请检查：')
            self.get_logger().error('  1. RK3588 上的 ins_driver_controlled 是否已启动')
            self.get_logger().error('  2. ROS_DOMAIN_ID 是否一致')
            self.get_logger().error('  3. 网络连接是否正常')
            return False
        return True

    def send_stop(self):
        """Send stop command"""
        req = INSCommand.Request()
        req.cmd_type = INSCommand.Request.CMD_STOP
        return self._call_service(req, "停止 INS")

    def send_start(self):
        """Send start command"""
        req = INSCommand.Request()
        req.cmd_type = INSCommand.Request.CMD_START
        return self._call_service(req, "启动 INS")

    def send_set_position(self, lat, lon, alt=0.0):
        """Send set position command"""
        req = INSCommand.Request()
        req.cmd_type = INSCommand.Request.CMD_SET_POS
        req.latitude = lat
        req.longitude = lon
        req.altitude = alt
        return self._call_service(req, f"设置位置 ({lat:.7f}, {lon:.7f})")

    def send_get_status(self):
        """Get current status"""
        req = INSCommand.Request()
        req.cmd_type = INSCommand.Request.CMD_GET_STATUS
        return self._call_service(req, "获取状态")

    def _call_service(self, request, action_name):
        """Call service and handle response"""
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is None:
            self.get_logger().error(f'{action_name} 失败: 服务调用超时')
            return False

        response = future.result()
        if response.success:
            self.get_logger().info(f'✓ {action_name} 成功')
            self.get_logger().info(f'  消息: {response.message}')

            # Print status if available
            if request.cmd_type == INSCommand.Request.CMD_GET_STATUS:
                align_names = {0: '监控', 1: '粗对准', 2: '精对准', 3: 'INS导航'}
                align_str = align_names.get(response.align_status, f'未知({response.align_status})')
                self.get_logger().info(f'  对准状态: {align_str}')
                self.get_logger().info(f'  工作状态: 0x{response.work_status:02X}')
                self.get_logger().info(f'  当前位置: lat={response.current_lat:.7f}, lon={response.current_lon:.7f}, alt={response.current_alt:.2f}')

            return True
        else:
            self.get_logger().error(f'✗ {action_name} 失败: {response.message}')
            return False


def print_usage():
    print("""
INS Control CLI - 控制 INS 设备

用法:
  ros2 run rov_ins_control ins_control_cli -- <命令> [参数]

命令:
  stop                    停止 INS，进入监控状态
  start                   启动 INS，开始对准
  setpos <lat> <lon> [alt] 设置初始位置（度）
  status                  获取当前状态

示例:
  ros2 run rov_ins_control ins_control_cli -- stop
  ros2 run rov_ins_control ins_control_cli -- setpos 31.234567 121.456789 10.5
  ros2 run rov_ins_control ins_control_cli -- start
  ros2 run rov_ins_control ins_control_cli -- status
""")


def main(args=None):
    if args is None:
        args = sys.argv[1:]

    if len(args) < 1 or args[0] in ['-h', '--help', 'help']:
        print_usage()
        return 0

    rclpy.init(args=sys.argv)
    node = INSControlCLI()

    try:
        if not node.wait_for_service():
            return 1

        cmd = args[0].lower()

        if cmd == 'stop':
            success = node.send_stop()

        elif cmd == 'start':
            success = node.send_start()

        elif cmd == 'setpos':
            if len(args) < 3:
                print("错误: setpos 需要纬度和经度参数")
                print_usage()
                return 1
            try:
                lat = float(args[1])
                lon = float(args[2])
                alt = float(args[3]) if len(args) > 3 else 0.0
                success = node.send_set_position(lat, lon, alt)
            except ValueError:
                print("错误: 经纬度必须是数字")
                return 1

        elif cmd == 'status':
            success = node.send_get_status()

        else:
            print(f"错误: 未知命令 '{cmd}'")
            print_usage()
            return 1

        return 0 if success else 1

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
