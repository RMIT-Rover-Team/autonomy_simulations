import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():

    # ------------------------------------------------------------------
    # Mode selection:
    #
    #   ros2 launch ... (no args)          -> mode=slam (safe default)
    #   ros2 launch ... mode:=slam         -> SLAM only
    #   ros2 launch ... mode:=amcl         -> AMCL only
    #   ros2 launch ... mode:=slam_nav     -> SLAM + navigation
    #   ros2 launch ... mode:=amcl_nav     -> AMCL + navigation
    # ------------------------------------------------------------------
    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="slam",
        choices=["slam", "amcl", "slam_nav", "amcl_nav"],
        description="Which stack to bring up: slam | amcl | slam_nav | amcl_nav"
    )
    mode = LaunchConfiguration("mode")

    default_bt_xml_filename_arg = DeclareLaunchArgument(
        "default_bt_xml_filename",
        default_value=os.path.join(
            get_package_share_directory("my_robot_navigation"),
            "behavior_tree",
            "simple_navigation_w_replanning_and_recovery.xml"
        ),
        description="Behavior tree XML for bt_navigator to use when mode is slam_nav/amcl_nav"
    )
    default_bt_xml_filename = LaunchConfiguration("default_bt_xml_filename")

    slam_condition = IfCondition(
        PythonExpression(["'", mode, "' in ('slam', 'slam_nav')"])
    )
    amcl_condition = IfCondition(
        PythonExpression(["'", mode, "' in ('amcl', 'amcl_nav')"])
    )
    navigation_condition = IfCondition(
        PythonExpression(["'", mode, "' in ('slam_nav', 'amcl_nav')"])
    )

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("my_robot_description"),
            "launch",
            "gazebo.launch.py"
        ),
    )

    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("my_robot_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_sim_time": "True"
        }.items(),
    )

    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("my_robot_controller"),
            "launch",
            "joystick_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "True"
        }.items()
    )

    ekf = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("my_robot_localization"),
            "launch",
            "local_localization.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "True"
        }.items(),
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("my_robot_mapping"),
            "launch",
            "slam.launch.py"
        ),
        condition=slam_condition
    )

    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("my_robot_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        condition=amcl_condition
    )

    safety_stop = Node(
        package="my_robot_utils",
        executable="safety_stop.py",
        output="screen",
        parameters=[{"use_sim_time": True}]

    )

    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("my_robot_navigation"),
            "launch",
            "navigation.launch.py"
        ),
        launch_arguments={
            "default_bt_xml_filename": default_bt_xml_filename
        }.items(),
        condition=navigation_condition
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",

        name="rviz2_nav",
        arguments=["-d", os.path.join(
                get_package_share_directory("nav2_bringup"),
                "rviz",
                "nav2_default_view.rviz"
        )
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=navigation_condition
    )

    return LaunchDescription([
        mode_arg,
        default_bt_xml_filename_arg,
        gazebo,
        controller,
        joystick,
        ekf,
        localization,
        slam,
        safety_stop,
        navigation,
        rviz
    ])
