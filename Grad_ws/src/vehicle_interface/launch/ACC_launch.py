from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, Shutdown, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os
import pathlib

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vehicle_interface',
            executable='actuators',
            name='Vehicle',
            output='screen',
        ),
        Node(
            package='vehicle_interface',
            executable='lcd',
            name='lcd_log',
            output='screen',
        ),
        Node(
            package='vehicle_controller',
            executable='adas',
            name='Controller',
            output='screen',
            parameters=[{
                'is_sim': False,
            }]
        ),
    ])
