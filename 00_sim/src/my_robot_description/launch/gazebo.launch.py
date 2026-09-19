import os
from os import pathsep
from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable, AppendEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    robot_description_dir = get_package_share_directory(
        "my_robot_description")

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(robot_description_dir, "urdf", "robot.urdf.xacro"),
        description="Absolute path to robot UF file"
    )

    world_model_arg = DeclareLaunchArgument(name="world_name", default_value="room1")

    world_path = PathJoinSubstitution([
        robot_description_dir,
        "worlds",
        PythonExpression(expression=["'", LaunchConfiguration("world_name"), "'", " + '.world'"])
    ])

    robot_description = ParameterValue(Command([
        "xacro", ' ', LaunchConfiguration("model")
    ]),
        value_type=str)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}]
    )

    model_path = str(Path(robot_description_dir).parent.resolve())
    model_path += pathsep + os.path.join(robot_description_dir, "models")

    gazebo_resource_path = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", model_path
    )

    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource([
        os.path.join(
            get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
        launch_arguments=[
            ("gz_args", [" -v 4", " -r", " ", world_path])
    ]
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.0",
            "-name", "robot"
        ]
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"
        ],
        remappings=[
            ("/imu", "/imu/out")
        ]
    )

    return LaunchDescription([
        model_arg,
        world_model_arg,
        robot_state_publisher,
        gazebo_resource_path,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge
    ])
