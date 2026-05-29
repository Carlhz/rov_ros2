from setuptools import setup

package_name = 'rov_topside'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/topside.launch.py',
            'launch/monitor_only.launch.py'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='ROV topside control and monitoring for ROS2 Foxy',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ins_monitor_node = rov_topside.ins_monitor_node:main',
            'ins_logger_node = rov_topside.ins_logger_node:main',
        ],
    },
)
