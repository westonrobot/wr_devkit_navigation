from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition


def generate_launch_description():
    declare_use_namespace_cmd = DeclareLaunchArgument(
        "use_namespace",
        default_value="false",
        description="Whether to apply a namespace to the navigation nodes",
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    SetParameter(
        condition=IfCondition(LaunchConfiguration("use_sim_time")),
        name="use_sim_time",
        value=LaunchConfiguration("use_sim_time"),
    )

    SetParameter(
        condition=IfCondition(LaunchConfiguration("use_namespace")),
        name="namespace",
        value=LaunchConfiguration("namespace"),
    )

    # --------- Robot Base ---------
    robot_base_bringup = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("ranger_base"),
                    "launch",
                    "include",
                    "ranger_robot_base.launch.xml",
                ])
            ]),
            launch_arguments={
                "port_name": "can0",
                "robot_model": "ranger_mini_v2",
                "odom_frame": "odom",
                "base_frame": "base_link",
                "update_rate": "50",
                "odom_topic_name": "odom",
                "publish_odom_tf": "true",
            }.items(),
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_footprint_transform_publisher",
            arguments=['--x', '0.0', '--y', '-0.0', '--z', '0.0',
                       '--yaw', '0', '--pitch', '0', '--roll', '0',
                       '--frame-id', 'base_link', '--child-frame-id', 'base_footprint']
        ),
    ])

    # --------- Chassis ---------
    chassis_bringup = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("ugv_devkit_v1_bringup"),
                    "launch",
                    "chassis.launch.py",
                ])
            ])
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="chassis_transform_publisher",
            arguments=['--x', '0.0', '--y', '-0.0', '--z', '0.335',
                       '--yaw', '0', '--pitch', '0', '--roll', '0',
                       '--frame-id', 'base_link', '--child-frame-id', 'ugv_devkit_v1_base_link']
        ),
    ])

    # --------- Sensor kits ---------
    sensor_kit_bringup = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("mid360_sensor_kit_bringup"),
                    "launch",
                    "sensor_kit.launch.py",
                ])
            ])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("w200d_sensor_kit_bringup"),
                    "launch",
                    "sensor_kit.launch.py",
                ])
            ]),
            launch_arguments={
                "robot_base": "ranger_mini",
            }.items(),
        ),
    ])

    return LaunchDescription([
        declare_use_namespace_cmd,
        declare_namespace_cmd,
        declare_use_sim_time_cmd,
        robot_base_bringup,
        chassis_bringup,
        sensor_kit_bringup
    ])
