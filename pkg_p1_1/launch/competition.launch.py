# pkg_p1_1/launch/competition.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory("pkg_p1_1")
    default_wp = os.path.join(pkg_share, "waypoints", "problem1-1_path.json")

    domain_id = LaunchConfiguration("domain_id")

    return LaunchDescription([
        DeclareLaunchArgument("domain_id", default_value="1"),

        Node(
            package="pkg_p1_1",
            executable="p1_1",
            name="p1_1",
            output="screen",
            additional_env={"ROS_DOMAIN_ID": domain_id},
            parameters=[
                {"waypoints_json": default_wp},
            ],
        ),
    ])
