from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('rov_sonar_driver')
    default_config = os.path.join(pkg_dir, 'config', 'sonar_omni.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='声纳驱动配置文件路径'
        ),
        DeclareLaunchArgument(
            'server_ip',
            default_value='192.168.0.5',
            description='全向声纳 IP 地址'
        ),
        DeclareLaunchArgument(
            'sector_width',
            default_value='3600',
            description='扇扫角度 (0.1°单位): 0=固定, 3600=全向PPI'
        ),

        Node(
            package='rov_sonar_driver',
            executable='sonar_omni_driver',
            name='sonar_omni_driver',
            output='screen',
            parameters=[LaunchConfiguration('config_file'), {
                'server_ip':    LaunchConfiguration('server_ip'),
                'sector_width': LaunchConfiguration('sector_width'),
            }],
            remappings=[
                ('sonar/omni/original',  '/sonar/omni/original'),
                ('sonar/omni/rigidity',  '/sonar/omni/rigidity'),
                ('sonar/omni/boundary',  '/sonar/omni/boundary'),
                ('sonar/omni/config',    '/sonar/omni/config'),
            ]
        ),
    ])
