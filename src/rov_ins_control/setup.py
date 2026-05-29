from setuptools import setup

package_name = 'rov_ins_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ROV Team',
    maintainer_email='rov@example.com',
    description='INS control tools for ROV',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ins_control_client = rov_ins_control.ins_control_client:main',
            'ins_gui = rov_ins_control.ins_gui:main',
        ],
    },
)
