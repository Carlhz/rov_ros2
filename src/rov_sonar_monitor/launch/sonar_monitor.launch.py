from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'frame_id',
            default_value='sonar_omni_link',
            description='声纳 TF 坐标系'
        ),

        Node(
            package='rov_sonar_monitor',
            executable='sonar_monitor_node',
            name='sonar_monitor_node',
            output='screen',
            parameters=[{
                'frame_id': LaunchConfiguration('frame_id'),
            }],
        ),
    ])
