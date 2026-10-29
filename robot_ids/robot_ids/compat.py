"""Differences between ROS 2 distributions that this package cares about."""

import os

# TurtleBot3 on Humble (Gazebo Classic) drives with geometry_msgs/Twist;
# on Jazzy and later it uses geometry_msgs/TwistStamped.
UNSTAMPED_CMD_VEL_DISTROS = ('foxy', 'galactic', 'humble', 'iron')


def default_cmd_vel_stamped():
    return os.environ.get('ROS_DISTRO', '') not in UNSTAMPED_CMD_VEL_DISTROS
