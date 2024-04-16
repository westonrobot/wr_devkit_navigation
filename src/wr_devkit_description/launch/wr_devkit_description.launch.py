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

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
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

    xacro_file = PathJoinSubstitution(
        [pkg_dir, "urdf", "wr_devkit.urdf.xacro"]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
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
                        "gazebo:=ignition",
                        " ",
                        "namespace:=",
                        namespace,
                    ]
                )
            },
        ],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add nodes to LaunchDescription
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    return ld
