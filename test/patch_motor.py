#!/usr/bin/env python3
import sys

# Read file
with open('/opt/ros/rov_ros2_ws/motor_controller.py', 'r', encoding='utf-8') as f:
    content = f.read()

# Find and replace the _read_sensor_pipe function
old_func = '''    def _read_sensor_pipe(self):
        """从 subprocess 管道读取 INS + 深度数据 (非阻塞)"""
        if not hasattr(self, 'sensor_proc') or self.sensor_proc is None:
            return
        import select as _sel
        try:
            while _sel.select([self.sensor_proc.stdout], [], [], 0.0)[0]:
                line = self.sensor_proc.stdout.readline()
                if not line:
                    break # EOF: 子进程已退出或管道关闭
                data = json.loads(line.strip())
                now = time.time()

                # INS 姿态
                if 'yaw' in data:
                    self.ins_yaw = float(data['yaw'])
                    self.ins_pitch = float(data.get('pitch', 0.0))
                    self.ins_roll = float(data.get('roll', 0.0))
                    self.ins_att_valid = True
                    self.last_att_time = now

                # 深度
                if 'depth' in data:
                    raw = float(data['depth'])
                    if self.depth_valid:
                        self.filtered_depth = 0.5 * raw + 0.5 * self.filtered_depth
                    else:
                        self.filtered_depth = raw
                    self.current_depth = self.filtered_depth
                    self.depth_valid = True
                    self.last_depth_time = now

                if not self._sensor_first and self.ins_att_valid and self.depth_valid:
                    self._sensor_first = True
                    self.get_logger().info(
                        '  传感器首帧: yaw={:.2f}deg pitch={:.2f}deg roll={:.2f}deg depth={:.3f}m'.format(
                            self.ins_yaw, self.ins_pitch, self.ins_roll, self.current_depth))
        except Exception:
            pass'''

new_func = '''    def _read_sensor_pipe(self):
        """从 subprocess 管道读取 INS + 深度数据 (非阻塞)"""
        if not hasattr(self, 'sensor_proc') or self.sensor_proc is None:
            return
        import select as _sel
        try:
            ready = _sel.select([self.sensor_proc.stdout], [], [], 0.0)[0]
            if ready:
                line = self.sensor_proc.stdout.readline()
                if not line:
                    self.get_logger().error('SENSOR_PIPE: EOF!')
                    return
                # 调试: 保存原始数据
                try:
                    with open('/tmp/sensor_pipe_raw.log', 'a') as f:
                        f.write(line)
                except:
                    pass
                data = json.loads(line.strip())
                now = time.time()

                # INS 姿态
                if 'yaw' in data:
                    self.ins_yaw = float(data['yaw'])
                    self.ins_pitch = float(data.get('pitch', 0.0))
                    self.ins_roll = float(data.get('roll', 0.0))
                    self.ins_att_valid = True
                    self.last_att_time = now

                # 深度
                if 'depth' in data:
                    raw = float(data['depth'])
                    if self.depth_valid:
                        self.filtered_depth = 0.5 * raw + 0.5 * self.filtered_depth
                    else:
                        self.filtered_depth = raw
                    self.current_depth = self.filtered_depth
                    self.depth_valid = True
                    self.last_depth_time = now

                if not self._sensor_first and self.ins_att_valid and self.depth_valid:
                    self._sensor_first = True
                    self.get_logger().info(
                        '  传感器首帧: yaw={:.2f}deg pitch={:.2f}deg roll={:.2f}deg depth={:.3f}m'.format(
                            self.ins_yaw, self.ins_pitch, self.ins_roll, self.current_depth))
            else:
                # 管道无数据
                if not hasattr(self, '_pipe_empty_logged'):
                    self.get_logger().error('SENSOR_PIPE: pipe empty (no data from sensor_bridge)')
                    self._pipe_empty_logged = True
        except Exception as e:
            self.get_logger().error('SENSOR_PIPE ERROR: {}'.format(e))
            import traceback
            self.get_logger().error(traceback.format_exc())'''

if old_func in content:
    content = content.replace(old_func, new_func)
    with open('/opt/ros/rov_ros2_ws/motor_controller.py', 'w', encoding='utf-8') as f:
        f.write(content)
    print("Patched successfully!")
    sys.exit(0)
else:
    print("ERROR: Could not find the old function!")
    # Try to find similar content
    if '_read_sensor_pipe' in content:
        print("Found _read_sensor_pipe but content doesn't match exactly")
    sys.exit(1)
