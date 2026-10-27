#!/usr/bin/env python3
"""Simulacion sin robot: lanza fake_robot (publica /odom, acepta /cmd_vel)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    b_eff_true = LaunchConfiguration('b_eff_true', default='0.27')
    wheel_sep = LaunchConfiguration('wheel_separation', default='0.24')
    return LaunchDescription([
        DeclareLaunchArgument('b_eff_true', default_value='0.27',
                              description='b_eff real simulado [m]'),
        DeclareLaunchArgument('wheel_separation', default_value='0.24',
                              description='b_eff que usa la odometria [m]'),
        Node(
            package='capytown_esan',
            executable='fake_robot',
            name='fake_robot',
            output='screen',
            parameters=[{'b_eff_true': b_eff_true, 'wheel_separation': wheel_sep}],
        ),
    ])
