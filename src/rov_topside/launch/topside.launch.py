from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # INS 监控节点
        Node(
            package='rov_topside',
            executable='ins_monitor_node',
            name='ins_monitor_node',
            output='screen',
        ),
        
        # INS 数据记录节点（可选）
        Node(
            package='rov_topside',
            executable='ins_logger_node',
            name='ins_logger_node',
            output='screen',
            parameters=[{
                'output_dir': '~/ins_logs',
                'filename_prefix': 'ins_data',
            }],
        ),
    ])
