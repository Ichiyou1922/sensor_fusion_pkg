# SPDX-FileCopyrightText: 2025 Kazuha Mogi <mogi2fruits.kazu@gmail.com>
# SPDX-License-Identifier: BSD-3-Clause
from launch import LaunchDescription
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_name = 'sensor_fusion_pkg'
    pkg_share = get_package_share_directory(pkg_name)

    # YAML パラメータへのパス
    param_file = os.path.join(
        pkg_share,
        'config',
        'generic_kf_1d2sens.yaml'
    )

    return LaunchDescription([
        # センサ1（精度が高い）
        Node(
            package=pkg_name,
            executable='noisy_sensor',
            name='sensor_precise',
            parameters=[{
                'variance': 1.0,
                'sensor_id': 1,
            }],
        ),

        # センサ2（ノイズが大きい）
        Node(
            package=pkg_name,
            executable='noisy_sensor',
            name='sensor_noisy',
            parameters=[{
                'variance': 5.0,
                'sensor_id': 2,
            }],
        ),

        # 汎用カルマンノード
        Node(
            package=pkg_name,
            executable='generic_kf_node',
            name='generic_kf',
            parameters=[param_file],
        ),
    ])
