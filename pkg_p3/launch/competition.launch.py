# pkg_p3/launch/competition_p3.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("pkg_p3")

    # waypoints: 3_cav1.json ~ 3_cav4.json
    default_wp_cav1 = os.path.join(pkg_share, "waypoints", "3_cav1.json")
    default_wp_cav2 = os.path.join(pkg_share, "waypoints", "3_cav2.json")
    default_wp_cav3 = os.path.join(pkg_share, "waypoints", "3_cav3.json")
    default_wp_cav4 = os.path.join(pkg_share, "waypoints", "3_cav4.json")

    # conflict map (필요시 파일명만 맞춰서 바꾸면 됨)
    default_zone_db = os.path.join(pkg_share, "config", "3_zone_database.json")

    # bridge yamls
    default_bridge_yaml_cav1 = os.path.join(pkg_share, "config", "bridge_cav1.yaml")
    default_bridge_yaml_cav2 = os.path.join(pkg_share, "config", "bridge_cav2.yaml")
    default_bridge_yaml_cav3 = os.path.join(pkg_share, "config", "bridge_cav3.yaml")
    default_bridge_yaml_cav4 = os.path.join(pkg_share, "config", "bridge_cav4.yaml")

    problem = LaunchConfiguration("problem")

    cav1_domain = LaunchConfiguration("cav1_domain")
    cav2_domain = LaunchConfiguration("cav2_domain")
    cav3_domain = LaunchConfiguration("cav3_domain")
    cav4_domain = LaunchConfiguration("cav4_domain")
    bridge_domain = LaunchConfiguration("bridge_domain")

    bridge_yaml_cav1 = LaunchConfiguration("bridge_yaml_cav1")
    bridge_yaml_cav2 = LaunchConfiguration("bridge_yaml_cav2")
    bridge_yaml_cav3 = LaunchConfiguration("bridge_yaml_cav3")
    bridge_yaml_cav4 = LaunchConfiguration("bridge_yaml_cav4")

    wp_cav1 = LaunchConfiguration("wp_cav1")
    wp_cav2 = LaunchConfiguration("wp_cav2")
    wp_cav3 = LaunchConfiguration("wp_cav3")
    wp_cav4 = LaunchConfiguration("wp_cav4")
    zone_db = LaunchConfiguration("zone_db")

    return LaunchDescription([
        # entrypoint.sh에서 problem:=4로 들어오므로 default도 4로
        DeclareLaunchArgument("problem", default_value="4"),

        # domains
        #======Must CHANGE these numbers=====#
        DeclareLaunchArgument("cav1_domain", default_value="1"),
        DeclareLaunchArgument("cav2_domain", default_value="2"),
        DeclareLaunchArgument("cav3_domain", default_value="3"),
        DeclareLaunchArgument("cav4_domain", default_value="4"),
        #======Must CHANGE these numbers=====#
        DeclareLaunchArgument("bridge_domain", default_value="98"),

        # bridge yamls
        DeclareLaunchArgument("bridge_yaml_cav1", default_value=default_bridge_yaml_cav1),
        DeclareLaunchArgument("bridge_yaml_cav2", default_value=default_bridge_yaml_cav2),
        DeclareLaunchArgument("bridge_yaml_cav3", default_value=default_bridge_yaml_cav3),
        DeclareLaunchArgument("bridge_yaml_cav4", default_value=default_bridge_yaml_cav4),

        # params
        DeclareLaunchArgument("wp_cav1", default_value=default_wp_cav1),
        DeclareLaunchArgument("wp_cav2", default_value=default_wp_cav2),
        DeclareLaunchArgument("wp_cav3", default_value=default_wp_cav3),
        DeclareLaunchArgument("wp_cav4", default_value=default_wp_cav4),
        DeclareLaunchArgument("zone_db", default_value=default_zone_db),

        # ---- Bridges (all run in domain 98) ----
        Node(
            package="domain_bridge",
            executable="domain_bridge",
            name="p3_v2v_bridge_cav1",
            output="screen",
            additional_env={"ROS_DOMAIN_ID": bridge_domain},
            arguments=[bridge_yaml_cav1],
        ),
        # Node(
        #     package="domain_bridge",
        #     executable="domain_bridge",
        #     name="p3_v2v_bridge_cav2",
        #     output="screen",
        #     additional_env={"ROS_DOMAIN_ID": bridge_domain},
        #     arguments=[bridge_yaml_cav2],
        # ),
        # Node(
        #     package="domain_bridge",
        #     executable="domain_bridge",
        #     name="p3_v2v_bridge_cav3",
        #     output="screen",
        #     additional_env={"ROS_DOMAIN_ID": bridge_domain},
        #     arguments=[bridge_yaml_cav3],
        # ),
        # Node(
        #     package="domain_bridge",
        #     executable="domain_bridge",
        #     name="p3_v2v_bridge_cav4",
        #     output="screen",
        #     additional_env={"ROS_DOMAIN_ID": bridge_domain},
        #     arguments=[bridge_yaml_cav4],
        # ),

        # ---- CAV1~4 ----
        Node(
            package="pkg_p3",
            executable="p3",
            name="p3_cav1",
            output="screen",
            additional_env={"ROS_DOMAIN_ID": cav1_domain},
            parameters=[
                {"problem": problem},
                {"waypoints_json": wp_cav1},
                {"conflict_map_json": zone_db},
            ],
        ),
        # Node(
        #     package="pkg_p3",
        #     executable="p3",
        #     name="p3_cav2",
        #     output="screen",
        #     additional_env={"ROS_DOMAIN_ID": cav2_domain},
        #     parameters=[
        #         {"problem": problem},
        #         {"waypoints_json": wp_cav2},
        #         {"conflict_map_json": zone_db},
        #     ],
        # ),
        # Node(
        #     package="pkg_p3",
        #     executable="p3",
        #     name="p3_cav3",
        #     output="screen",
        #     additional_env={"ROS_DOMAIN_ID": cav3_domain},
        #     parameters=[
        #         {"problem": problem},
        #         {"waypoints_json": wp_cav3},
        #         {"conflict_map_json": zone_db},
        #     ],
        # ),
        # Node(
        #     package="pkg_p3",
        #     executable="p3",
        #     name="p3_cav4",
        #     output="screen",
        #     additional_env={"ROS_DOMAIN_ID": cav4_domain},
        #     parameters=[
        #         {"problem": problem},
        #         {"waypoints_json": wp_cav4},
        #         {"conflict_map_json": zone_db},
        #     ],
        # ),
    ])
