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
from ament_index_python import get_package_share_directory


def generate_launch_description():

    lifecycle_nodes = ["map_saver_server"]
    ros_distro = os.environ["ROS_DISTRO"]
    if ros_distro != "humble":
        lifecycle_nodes.append("slam_toolbox")

    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")
    slam_config_arg = DeclareLaunchArgument("slam_config", default_value=os.path.join(
        get_package_share_directory("my_robot_mapping"), "config", "slam_toolbox.yaml"
    ))

    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_config = LaunchConfiguration("slam_config")

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_config, {"use_sim_time": use_sim_time}]
    )

    map_saver = Node(
        package="nav2_map_server",
        executable="map_saver_server",
        name="map_saver_server",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time},
                    {"save_map_timeout": 10.0},
                    {"free_thresh_default": 0.196},
                    {"occupied_thresh_default": 0.65}]
    )

    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
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
                get_package_share_directory("my_robot_mapping"),
                "rviz",
                "slam.rviz")]
    )

    return LaunchDescription([
        use_sim_time_arg,
        slam_config_arg,
        slam_toolbox,
        map_saver,
        nav2_lifecycle_manager,
        rviz2
    ])
