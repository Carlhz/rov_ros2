#!/usr/bin/env python3
"""
INS Control Client
Runs on VM - sends control commands to RK3588 via ROS2 service

Usage:
    ros2 run rov_ins_control ins_control_client -- stop
    ros2 run rov_ins_control ins_control_client -- start
    ros2 run rov_ins_control ins_control_client -- setpos 31.234567 121.456789 0.0
    ros2 run rov_ins_control ins_control_client -- status
"""

import sys
import argparse

import rclpy
from rclpy.node import Node
from rov_ins_interface.srv import INSCommand


class INSControlClient(Node):
    def __init__(self):
        super().__init__('ins_control_client')
        self.cli = self.create_client(INSCommand, '/ins/control')

        # Wait for service
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /ins/control service...')

        self.get_logger().info('Connected to /ins/control service')

    def send_command(self, command, lat=0.0, lon=0.0, alt=0.0):
        req = INSCommand.Request()
        req.command = command
        req.latitude = lat
        req.longitude = lon
        req.altitude = alt

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        try:
            response = future.result()
            return response
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            return None


def main(args=None):
    parser = argparse.ArgumentParser(description='INS Control Client')
    parser.add_argument('action', choices=['stop', 'start', 'setpos', 'status'],
                        help='Control action')
    parser.add_argument('args', nargs='*', help='Additional arguments (lat lon alt for setpos)')

    parsed = parser.parse_args()

    rclpy.init(args=args)
    client = INSControlClient()

    try:
        if parsed.action == 'stop':
            print("Sending STOP command...")
            resp = client.send_command(INSCommand.Request.CMD_STOP)

        elif parsed.action == 'start':
            print("Sending START command...")
            resp = client.send_command(INSCommand.Request.CMD_START)

        elif parsed.action == 'setpos':
            if len(parsed.args) < 2:
                print("Error: setpos requires latitude and longitude")
                print("Usage: setpos <lat> <lon> [alt]")
                return 1

            lat = float(parsed.args[0])
            lon = float(parsed.args[1])
            alt = float(parsed.args[2]) if len(parsed.args) > 2 else 0.0

            print(f"Sending SET_POS command: lat={lat}, lon={lon}, alt={alt}")
            resp = client.send_command(INSCommand.Request.CMD_SET_POS, lat, lon, alt)

        elif parsed.action == 'status':
            print("Getting status...")
            resp = client.send_command(INSCommand.Request.CMD_GET_STATUS)

        if resp:
            print(f"Success: {resp.success}")
            print(f"Message: {resp.message}")
            if hasattr(resp, 'current_status'):
                status_names = {0: '监控', 1: '粗对准', 2: '精对准', 3: 'INS导航'}
                print(f"Current Status: {status_names.get(resp.current_status, 'Unknown')} ({resp.current_status})")
        else:
            print("No response received")

    except Exception as e:
        print(f"Error: {e}")
        return 1

    finally:
        client.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == '__main__':
    sys.exit(main())
