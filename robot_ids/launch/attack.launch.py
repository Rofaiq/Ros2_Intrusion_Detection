"""Run one attack against the running simulation.

  ros2 launch robot_ids attack.launch.py attack:=scan_spoof
Attacks: cmd_vel_spoof, scan_spoof, odom_replay, flood.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('attack', default_value='cmd_vel_spoof'),
        DeclareLaunchArgument('start_delay_s', default_value='5.0'),
        DeclareLaunchArgument('duration_s', default_value='20.0'),
        Node(
            package='robot_ids',
            executable='attack_injector',
            name='attack_injector',
            parameters=[{
                'use_sim_time': True,
                'attack': LaunchConfiguration('attack'),
                'start_delay_s': LaunchConfiguration('start_delay_s'),
                'duration_s': LaunchConfiguration('duration_s'),
            }],
            output='screen',
        ),
    ])
