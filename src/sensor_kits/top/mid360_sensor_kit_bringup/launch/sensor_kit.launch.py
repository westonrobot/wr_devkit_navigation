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
        FindPackageShare("mid360_sensor_kit_bringup").find(
            "mid360_sensor_kit_bringup"),
        "urdf/sensor_kit.urdf.xacro"
    )
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toprettyxml(indent='  ')

    load_description = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="mid360_sensor_kit_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_desc}],
    )

    # --------- Drivers ---------

    imu_bringup = GroupAction([
        Node(
            package="wrp_ros2",
            name="ch104m_imu_sensor_node",
            executable="imu_sensor_node",
            output="screen",
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('mid360_sensor_kit_bringup'),
                    'config',
                    'hipnuc.param.yaml'])
            ],
        )
    ])

    user_config_path = PathJoinSubstitution([
        FindPackageShare('mid360_sensor_kit_bringup'),
        "config",
        "MID360_config.json"
    ])

    lidar_bringup = GroupAction([
        Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_lidar_publisher',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('mid360_sensor_kit_bringup'),
                    'config',
                    'hipnuc.param.yaml']),
                {"user_config_path": user_config_path},
            ]
        ),
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="pointcloud_to_laserscan_node",
            output="screen",
            parameters=[{
                "range_min": 0.1,
                "range_max": 100.0,
                "scan_time": 0.1,
                "min_height": -0.025,
                "max_height": 0.025,
                "angle_min": -3.14159265,
                "angle_max": 3.14159265,
                "angle_increment": 0.0174532925,
                "inf_epsilon": 1.0,
                "tf_tolerance": 0.03,
            }],
            remappings=[
                ("/cloud_in", "/livox/lidar"),
                ("/scan", "/scan"),
            ],
        ),
    ])

    return LaunchDescription([
        declare_use_namespace_arg,
        declare_namespace_arg,
        load_description,
        imu_bringup,
    ])
