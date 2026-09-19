# start map sever node in nav2 lib to load a map (OccupancyGrid from file
# system) and publish it to a ROS2 topic (which we can use to visualize
# the map on rviz2 and which other nodes can use to recieve the map of the
# environment).

# 1. use the launch file to launch and configure the map_server, later on
# we will also start global localization system that will use the map
# hosted by the map server in order to localize the robot in the
# environment and to correct the inevitable odometry errors

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python import get_package_share_directory


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")
    map_name_arg = DeclareLaunchArgument("map_name", default_value="room1")
    amcl_config_arg = DeclareLaunchArgument("amcl_config", default_value=os.path.join(
        get_package_share_directory("my_robot_localization"), "config", "amcl.yaml"
    ))

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_name = LaunchConfiguration("map_name")
    amcl_config = LaunchConfiguration("amcl_config")

    lifecycle_nodes = ["map_server", "amcl"]

    # map.yaml file contains metadata on how the map server shud load/ read/
    # interpret the map.pgm file
    map_path = PathJoinSubstitution([get_package_share_directory(
        "my_robot_mapping"), "maps", map_name, "map.yaml"])

    nav2_map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        output="screen",
        parameters=[{"yaml_filename": map_path}, {"use_sim_time": use_sim_time}]
    )

    nav2_amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[amcl_config, {"use_sim_time": use_sim_time}]
    )

    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[{"node_names": lifecycle_nodes}, {"use_sim_time": use_sim_time},
                    {"autostart": True}]
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("my_robot_localization"),
                "rviz",
                "nav2_amcl.rviz")]
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_name_arg,
        amcl_config_arg,
        nav2_map_server,
        nav2_lifecycle_manager,
        nav2_amcl,
        rviz2
    ])
