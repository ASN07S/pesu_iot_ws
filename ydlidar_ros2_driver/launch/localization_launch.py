from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    map_path = os.path.join(
        os.getenv("HOME"), "lidar_ws", "ros2_ws", "src", "ydlidar_ros2_driver", "maps", "map.yaml"
    )

    return LaunchDescription([
        Node(
            package="nav2_bringup",
            executable="localization_launch.py",
            output="screen",
            parameters=[
                {"use_sim_time": False},
                {"map": map_path}
            ]
        )
    ])

