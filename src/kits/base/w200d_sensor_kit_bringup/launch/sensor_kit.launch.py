from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.conditions import IfCondition

import os
import xacro


def launch_setup(context, *args, **kwargs):
   # --------- Namespace ---------
    PushRosNamespace(
        condition=IfCondition(LaunchConfiguration("use_namespace")),
        namespace=LaunchConfiguration("namespace")
    )

    # --------- Description ---------
    xacro_file = os.path.join(
        FindPackageShare("w200d_sensor_kit_bringup").find(
            "w200d_sensor_kit_bringup"),
        "urdf/sensor_kit.urdf.xacro"
    )
    doc = xacro.process_file(xacro_file,
                             mappings={
                                 'robot_base': LaunchConfiguration("robot_base").perform(context)
                             })
    robot_desc = doc.toprettyxml(indent='  ')
    print(robot_desc)

    load_description = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="w200d_sensor_kit_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_desc}],
    )

    # --------- Drivers ---------

    ultrasonic_bringup = Node(
        package="wrp_ros2",
        name="ultrasonic_sensor_node",
        executable="ultrasonic_sensor_node",
        output="screen",
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('w200d_sensor_kit_bringup'),
                'config',
                'w200d.param.yaml']
            )],
    )

    return [
        load_description,
        ultrasonic_bringup
    ]


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

    declare_robot_base_arg = DeclareLaunchArgument(
        "robot_base",
        default_value="ranger_mini",
        description="What ultrasonic offsets to use, refer to param file"
    )

    return LaunchDescription([
        declare_use_namespace_arg,
        declare_namespace_arg,
        declare_robot_base_arg,
    ]
        + [OpaqueFunction(function=launch_setup)])
