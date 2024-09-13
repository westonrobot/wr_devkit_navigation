from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

import os
import xacro


def generate_launch_description():
    # --------- Arguments ---------
    declare_use_namespace_arg = DeclareLaunchArgument(
        "use_namespace",
        default_value="false",
        description="Whether to apply a namespace"
    )

    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Top-level namespace"
    )

    # --------- Namespace ---------
    PushRosNamespace(
        condition=IfCondition(LaunchConfiguration("use_namespace")),
        namespace=LaunchConfiguration("namespace")
    )

    # --------- Description ---------
    xacro_file = os.path.join(
        FindPackageShare("ugv_devkit_v1_bringup").find(
            "ugv_devkit_v1_bringup"),
        "urdf/chassis.urdf.xacro"
    )
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toprettyxml(indent='  ')

    load_description = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="ugv_devkit_v1_sensor_kit_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_desc}],
        remappings=[
            ("robot_description", "ugv_devkit_v1_chassis_description"),
        ]
    )

    return LaunchDescription([
        declare_use_namespace_arg,
        declare_namespace_arg,
        load_description,
    ])
