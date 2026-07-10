from setuptools import setup

package_name = 'rov_sonar_monitor'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/sonar_monitor.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ROV Team',
    maintainer_email='rov@example.com',
    description='ROV omnidirectional sonar monitoring node for VM topside',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sonar_monitor_node = rov_sonar_monitor.sonar_monitor_node:main',
        ],
    },
)
