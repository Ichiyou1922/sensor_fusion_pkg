from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensor_fusion_pkg',
            executable='noisy_sensor',
            name='sensor_precise',
            parameters[{'vaeiance': 1.0}, {'sensor_id': 1}]
            ),
        Node(
            package='sensor_fusion_pkg',
            executable='noisy_sensor',
            name='sensor_noisy',
            parameters=[{'variance': 5.0}, {'sensor_id': 2}]
            ),
        Node(
            package='sensor_fusion_pkg',
            executable='fusion_node',
            name='fusion_center',
            parameters=[{'var1': 1.0}, {'var2': 5.0}]
            )
        ])
