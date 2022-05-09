from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import os


def generate_launch_description():
    param_file = os.path.join(get_package_share_directory("f110_rrt_star"), "param", "rrt_star.yaml")
    csv_file = os.path.join(get_package_share_directory("f110_rrt_star"), "paths", "rrt_star.csv")
    map_file = os.path.join(get_package_share_directory("f110_rrt_star"), "maps", "rrt_star.yaml")

    return LaunchDescription(
        [
            Node(
                package="f110_wall_follow",
                executable="f110_wall_follow_node_exe",
                name="f110_wall_follow_node",
                output="screen",
                parameters=[param_file],
                remappings=[
                    ("scan", "/scan"),
                    ("drive", "/drive"),
                    ("dynamic_map", "/dynamic_map"),
                    ("waypoint_viz_marker", "/waypoint_viz_marker"),
                    ("tree_viz_marker", "/tree_viz_marker"),
                    ("scan", "/scan"),
                    ("pose", "/pose")
                ],
            ),
        ]
    )