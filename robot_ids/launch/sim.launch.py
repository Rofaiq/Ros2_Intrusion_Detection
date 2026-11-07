"""TurtleBot3 world in Gazebo plus the intrusion monitor and safety gate.

Drive the robot through the gate, for example:
  ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r cmd_vel:=cmd_vel_in
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # The TurtleBot3 launch files read the model from the environment.
    os.environ.setdefault('TURTLEBOT3_MODEL', 'burger')

    tb3_launch = os.path.join(get_package_share_directory('turtlebot3_gazebo'),
                              'launch', 'turtlebot3_world.launch.py')
    ids_launch = os.path.join(get_package_share_directory('robot_ids'),
                              'launch', 'ids.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            'graph_checks', default_value='true',
            description='false: ignore unexpected publishers and test the other detectors'),
        DeclareLaunchArgument('detectors', default_value='rules',
                              description='rules, learned, all or none'),
        DeclareLaunchArgument('model', default_value='~/ids_models/model.json'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(tb3_launch)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(ids_launch),
                                 launch_arguments={
                                     'use_sim_time': 'true',
                                     'graph_checks': LaunchConfiguration('graph_checks'),
                                     'detectors': LaunchConfiguration('detectors'),
                                     'model': LaunchConfiguration('model'),
                                 }.items()),
    ])
