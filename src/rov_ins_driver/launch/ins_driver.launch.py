from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'local_ip',
            default_value='192.168.0.99',
            description='Local IP for UDP binding'
        ),
        DeclareLaunchArgument(
            'local_port',
            default_value='8008',
            description='Local UDP port'
        ),
        DeclareLaunchArgument(
            'ins_ip',
            default_value='192.168.0.7',
            description='INS device IP'
        ),
        DeclareLaunchArgument(
            'ins_cmd_port',
            default_value='8007',
            description='INS command port'
        ),
        
        Node(
            package='rov_ins_driver',
            executable='ins_driver_node',
            name='ins_driver_node',
            output='screen',
            parameters=[{
                'local_ip': LaunchConfiguration('local_ip'),
                'local_port': LaunchConfiguration('local_port'),
                'ins_ip': LaunchConfiguration('ins_ip'),
                'ins_cmd_port': LaunchConfiguration('ins_cmd_port'),
            }],
            remappings=[
                ('/ins/data', '/ins/data'),
                ('/ins/command', '/ins/command'),
            ]
        ),
    ])
