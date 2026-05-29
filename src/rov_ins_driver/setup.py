from setuptools import setup

package_name = 'rov_ins_driver'

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
    description='INS driver for ROV (RK3588)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ins_driver = rov_ins_driver.ins_driver_controlled:main',
            'ins_driver_old = rov_ins_driver.ins_driver_full:main',
        ],
    },
)
