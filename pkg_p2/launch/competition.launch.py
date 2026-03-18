# pkg_p2/launch/competition_p2.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory("pkg_p2")

    lane_files = [
        os.path.join(pkg_share, "waypoints", "p2_lane1.json"),
        os.path.join(pkg_share, "waypoints", "p2_lane2.json"),
        os.path.join(pkg_share, "waypoints", "p2_lane3.json"),
    ]
    default_bridge_yaml = os.path.join(pkg_share, "config", "p2_bridge.yaml")

    bridge_domain = LaunchConfiguration("bridge_domain")
    cav_domain = LaunchConfiguration("cav_domain")
    bridge_yaml = LaunchConfiguration("bridge_yaml")



    return LaunchDescription([
        DeclareLaunchArgument("bridge_domain", default_value="99"),
        DeclareLaunchArgument("cav_domain", default_value="1"),
        DeclareLaunchArgument("bridge_yaml", default_value=default_bridge_yaml),


        Node(
            package="domain_bridge",
            executable="domain_bridge",
            name="p2_domain_bridge",
            output="screen",
            additional_env={"ROS_DOMAIN_ID": bridge_domain},
            arguments=[bridge_yaml],
        ),

        Node(
            package="pkg_p2",
            executable="p2",
            name="p2",
            output="screen",
            additional_env={"ROS_DOMAIN_ID": cav_domain},
            parameters=[
                {"lane_waypoints_jsons": lane_files},
                {"use_sim_time": True},
            ],
        ),
    ])
