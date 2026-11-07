"""Start the intrusion monitor and the safety gate (no simulator)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('robot_ids'), 'config', 'monitor.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'graph_checks', default_value='true',
            description='false: ignore unexpected publishers and test the other detectors'),
        DeclareLaunchArgument(
            'detectors', default_value='rules',
            description='which detectors may raise the alarm: rules, learned, all or none'),
        DeclareLaunchArgument(
            'model', default_value='~/ids_models/model.json',
            description='trained model file for the learned detectors'),
        Node(
            package='robot_ids',
            executable='monitor',
            name='ids_monitor',
            parameters=[config, {'use_sim_time': use_sim_time,
                                 'graph_checks': LaunchConfiguration('graph_checks'),
                                 'detector_set': LaunchConfiguration('detectors'),
                                 'learned_model_path': LaunchConfiguration('model')}],
            output='screen',
        ),
        Node(
            package='robot_ids',
            executable='safety_gate',
            name='safety_gate',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])
