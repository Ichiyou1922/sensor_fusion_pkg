# SPDX-FileCopyrightText: 2025 Kazuha Mogi <mogi2fruits.kazu@gmail.com>
# SPDX-License-Identifier: BSD-3-Clause

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
import os


def generate_launch_description():
    pkg_name = 'sensor_fusion_pkg'
    pkg_share = get_package_share_directory(pkg_name)

    default_param = os.path.join(
            pkg_share, 'config', 'generic_kf_1d2sens.yaml'
            )

    return LaunchDescription([
        DeclareLaunchArgument(
            'param_file',
            default_value=default_param,
            description='Kalman Filter parameter YAML file'
        ),

        DeclareLaunchArgument(
            'use_sim_sensors',
            default_value='true',
            description='Whether to launch simulated noisy sensors'
        ),

        Node(
            package=pkg_name,
            executable='noisy_sensor',
            name='sensor_precise',
            parameters=[{'variance': 1.0, 'sensor_id': 1}],
            condition=IfCondition(LaunchConfiguration('use_sim_sensors'))
        ),

        Node(
            package=pkg_name,
            executable='noisy_sensor',
            name='sensor_noisy',
            parameters=[{'variance': 5.0, 'sensor_id': 2}],
            condition=IfCondition(LaunchConfiguration('use_sim_sensors'))
        ),

        Node(
            package=pkg_name,
            executable='generic_kf_node',
            name='generic_kf',
            parameters=[LaunchConfiguration('param_file')],
        ),
    ])
