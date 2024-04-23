from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration

from launch_ros.actions import Node

import os


def generate_launch_description():

    pkg_dir = get_package_share_directory("wr_devkit_description")

    # Create the launch configuration variables
    namespace = LaunchConfiguration("namespace")
    xacro_file = LaunchConfiguration("xacro_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    is_scout_mini = LaunchConfiguration("is_scout_mini")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="use_sim_time",
    )

    declare_robot_name_cmd = DeclareLaunchArgument(
        "robot_name", default_value="wr_devkit", description="WR Dev Kit"
    )
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value=LaunchConfiguration("robot_name"),
        description="Robot namespace",
    )

    declare_is_scout_mini_cmd = DeclareLaunchArgument(
        "is_scout_mini",
        default_value="false",
        description="Is Scout Mini",
    )

    xacro_file = PathJoinSubstitution(
        [pkg_dir, "urdf", "wr_devkit_ultrasonic.urdf.xacro"]
    )

    ultrasonic_tf_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="ultrasonic_tf_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {
                "robot_description": Command(
                    [
                        "xacro",
                        " ",
                        xacro_file,
                        " ",
                        "namespace:=",
                        namespace,
                        " ",
                        "scout_mini:=",
                        is_scout_mini,
                    ]
                )
            },
        ],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add nodes to LaunchDescription
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_is_scout_mini_cmd)
    ld.add_action(ultrasonic_tf_publisher)
    return ld
