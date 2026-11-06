from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_ids'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Faiq Mansoor',
    maintainer_email='faiqmansoor336@gmail.com',
    description='Intrusion detection and safe stop for a ROS 2 mobile robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'monitor = robot_ids.monitor_node:main',
            'safety_gate = robot_ids.safety_gate_node:main',
            'attack_injector = robot_ids.attack_injector_node:main',
            'event_logger = robot_ids.event_logger_node:main',
        ],
    },
)
