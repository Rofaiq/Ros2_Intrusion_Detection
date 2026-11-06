"""Run the full detection experiment and write a results table.

  ros2 launch robot_ids experiment.launch.py
  ros2 launch robot_ids experiment.launch.py graph_checks:=false trials:=5 baseline_s:=120

Starts the simulation, monitor and gate, a scripted driver, and the
experiment runner; everything shuts down when the runner is finished.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_launch = os.path.join(get_package_share_directory('robot_ids'),
                              'launch', 'sim.launch.py')
    runner = Node(
        package='robot_ids',
        executable='experiment',
        name='ids_experiment',
        parameters=[{
            'trials': LaunchConfiguration('trials'),
            'baseline_s': LaunchConfiguration('baseline_s'),
        }],
        output='screen',
    )
    return LaunchDescription([
        DeclareLaunchArgument('graph_checks', default_value='true'),
        DeclareLaunchArgument('trials', default_value='10'),
        DeclareLaunchArgument('baseline_s', default_value='300.0'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(sim_launch),
                                 launch_arguments={
                                     'graph_checks': LaunchConfiguration('graph_checks'),
                                 }.items()),
        Node(
            package='robot_ids',
            executable='driver',
            name='driver',
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
        runner,
        RegisterEventHandler(OnProcessExit(
            target_action=runner,
            on_exit=[EmitEvent(event=Shutdown(reason='experiment finished'))])),
    ])
