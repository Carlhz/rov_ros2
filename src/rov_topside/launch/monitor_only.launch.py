from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """仅启动监控节点（不记录数据）"""
    return LaunchDescription([
        Node(
            package='rov_topside',
            executable='ins_monitor_node',
            name='ins_monitor_node',
            output='screen',
        ),
    ])
