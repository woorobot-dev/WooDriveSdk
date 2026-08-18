"""Starts dual_motor_node with amr_2wheel.yaml. Stage 1 only -- no /cmd_vel,
no kinematics, no odometry/TF. See README.md.

Usage:
    ros2 launch amr_2wheel_robot amr_2wheel.launch.py
    ros2 launch amr_2wheel_robot amr_2wheel.launch.py config:=/path/to/other.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('amr_2wheel_robot'), 'config', 'amr_2wheel.yaml'
    )

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Path to the parameter YAML file (see config/amr_2wheel.yaml)',
    )

    dual_motor_node = Node(
        package='amr_2wheel_robot',
        executable='dual_motor_node',
        name='dual_motor_node',
        output='screen',
        parameters=[LaunchConfiguration('config')],
    )

    return LaunchDescription([config_arg, dual_motor_node])
