# pkg_p1_2/launch/competition.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory("pkg_p1_2")

    default_wp_cav1 = os.path.join(pkg_share, "waypoints", "path_cav1.json")
    default_wp_cav2 = os.path.join(pkg_share, "waypoints", "path_cav2.json")
    default_zone_db = os.path.join(pkg_share, "config", "zone_database.json")
    default_bridge_yaml = os.path.join(pkg_share, "config", "v2v_bridge.yaml")

    problem = LaunchConfiguration("problem")

    cav1_domain = LaunchConfiguration("cav1_domain")
    cav2_domain = LaunchConfiguration("cav2_domain")
    bridge_domain = LaunchConfiguration("bridge_domain")
    bridge_yaml = LaunchConfiguration("bridge_yaml")

    wp_cav1 = LaunchConfiguration("wp_cav1")
    wp_cav2 = LaunchConfiguration("wp_cav2")
    zone_db = LaunchConfiguration("zone_db")

    return LaunchDescription([
        DeclareLaunchArgument("problem", default_value="2"),

        DeclareLaunchArgument("cav1_domain", default_value="1"),
        DeclareLaunchArgument("cav2_domain", default_value="2"),
        DeclareLaunchArgument("bridge_domain", default_value="99"),
        DeclareLaunchArgument("bridge_yaml", default_value=default_bridge_yaml),

        DeclareLaunchArgument("wp_cav1", default_value=default_wp_cav1),
        DeclareLaunchArgument("wp_cav2", default_value=default_wp_cav2),
        DeclareLaunchArgument("zone_db", default_value=default_zone_db),

        # Bridge (프로세스 도메인만 99)
        Node(
            package="domain_bridge",
            executable="domain_bridge",
            name="smyd_v2v_bridge",
            output="screen",
            additional_env={"ROS_DOMAIN_ID": bridge_domain},
            arguments=[bridge_yaml],
        ),

        # CAV1
        Node(
            package="pkg_p1_2",
            executable="p1_2",
            name="p1_2_cav1",
            output="screen",
            additional_env={"ROS_DOMAIN_ID": cav1_domain},
            parameters=[
                {"problem": problem},  
                {"waypoints_json": wp_cav1},
                {"conflict_map_json": zone_db},
            ],
        ),

        # CAV2
        Node(
            package="pkg_p1_2",
            executable="p1_2",
            name="p1_2_cav2",
            output="screen",
            additional_env={"ROS_DOMAIN_ID": cav2_domain},
            parameters=[
                {"problem": problem},
                {"waypoints_json": wp_cav2},
                {"conflict_map_json": zone_db},
            ],
        ),
    ])
