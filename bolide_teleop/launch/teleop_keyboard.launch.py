import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import os


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            parameters=[
                {'serial_port': '/dev/ttyLIDAR'},
                {'serial_baudrate': 256000},
                {'frame_id': 'laser_frame'},
                {'inverted': False},
                {'angle_compensate': True},
                {'scan_mode': 'Express'}
            ],
            respawn=True
        ),
        Node(
            package='stm32',
            executable='stm32_node',
            name='stm32_node',
            output='screen',
            respawn=True,
            parameters=[
                {'debug': False}
            ]
        ),
        Node(
            package='bolide_stm32',
            executable='cmd_vel_node',
            name='cmd_vel_node',
            output='screen',
            parameters=[
                {'minimal_speed' : 8.2}
            ],
            respawn=True
        ),
        Node(
            package='bolide_direction',
            executable='cmd_dir_node',
            name='cmd_dir_node',
            output='screen',
            respawn=True
        ),
        Node(
            package='bolide_teleop',
            executable='teleop_keyboard',
            name='teleop_keyboard',
            output='screen',
            respawn=True
        ),
    ])
