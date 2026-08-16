from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory("woodrive_ros2")) / "config" / "woodrive.yaml"
    return LaunchDescription([
        Node(
            package="woodrive_ros2",
            executable="woodrive_node",
            name="woodrive",
            output="screen",
            parameters=[str(config)],
        )
    ])
