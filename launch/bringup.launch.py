#!/usr/bin/env python3
"""Lanza odom_corrector con config/wheel_params.yaml.
El agente micro-ROS ya corre como contenedor `micro_ros_agent`; no se incluye el
bringup de Yahboom porque su EKF publicaria otro /odom."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('capytown_esan'), 'config', 'wheel_params.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('wheel_params', default_value=default_params),
        Node(
            package='capytown_esan',
            executable='odom_corrector',
            name='odom_corrector',
            output='screen',
            parameters=[LaunchConfiguration('wheel_params')],
        ),
    ])
