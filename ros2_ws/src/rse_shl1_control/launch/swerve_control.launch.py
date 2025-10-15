#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    # Swerve controller node
    # Parameters match your robot's URDF configuration
    swerve_controller_node = Node(
        package='rse_shl1_control',
        executable='swerve_controller',
        name='swerve_controller',
        output='screen',
        parameters=[{
            'wheel_radius': 0.0825,    # From URDF: wheel_radius arg
            'wheelbase': 0.54,          # From URDF: wheelbase_L arg
            'track_width': 0.53,        # From URDF: track_W arg
        }]
    )
    
    return LaunchDescription([
        swerve_controller_node,
    ])