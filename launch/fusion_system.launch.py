from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Node1: Good sensor
        Node(
            package='sensor_fusion_pkg',
            executable='noisy_sensor',
            name='sensor_precise',
            parameters=[{'variance': 1.0, 'sensor_id': 1}]
        ),
        # Node2: Bad sensor
        Node(
            package='sensor_fusion_pkg',
            executable='noisy_sensor',
            name='sensor_noisy',
            parameters=[{'variance': 5.0, 'sensor_id': 2}]
        ),
        # Node3: KalmanFilter
        Node(
            package='sensor_fusion_pkg',
            executable='fusion_node',
            name='fusion_center',
            parameters=[{
                'r_sensor1': 1.0, 
                'r_sensor2': 5.0, 
                'q_process_noise': 0.01
            }]
        )
    ])
