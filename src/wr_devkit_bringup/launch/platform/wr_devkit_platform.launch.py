from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition


def generate_launch_description():
    robot_model = LaunchConfiguration("robot_model")

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

    declare_robot_model_cmd = DeclareLaunchArgument(
        "robot_model",
        default_value="ranger_mini_v2",
        description="ranger_mini_v2, scout_mini",
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
            XMLLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("ranger_base"),
                    "launch",
                    "include",
                    "ranger_robot_base.launch.xml",
                ])
            ]),
            condition=IfCondition(PythonExpression(["'", robot_model, "' == 'ranger_mini_v2'"])),
            launch_arguments={
                "port_name": "can0",
                "robot_model": robot_model,
                "odom_frame": "odom",
                "base_frame": "base_link",
                "update_rate": "50",
                "odom_topic_name": "odom",
                "publish_odom_tf": "true",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("scout_base"),
                    "launch",
                    "scout_mini_base.launch.py",
                ])
            ]),
            condition=IfCondition(PythonExpression(["'", robot_model, "' == 'scout_mini'"])),
            launch_arguments={
                "port_name": "can0",
                "is_scout_mini": "true",
                "odom_frame": "odom",
                "base_frame": "base_link",
                "odom_topic_name": "odom",
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
            condition=IfCondition(PythonExpression(["'", robot_model, "' == 'ranger_mini_v2'"])),
            package="tf2_ros",
            executable="static_transform_publisher",
            name="chassis_transform_publisher",
            arguments=['--x', '0.0', '--y', '-0.0', '--z', '0.335',
                       '--yaw', '0', '--pitch', '0', '--roll', '0',
                       '--frame-id', 'base_link', '--child-frame-id', 'ugv_devkit_v1_base_link']
        ),
        Node(
            condition=IfCondition(PythonExpression(["'", robot_model, "' == 'scout_mini'"])),
            package="tf2_ros",
            executable="static_transform_publisher",
            name="chassis_transform_publisher",
            arguments=['--x', '0.0', '--y', '-0.0', '--z', '0.250',
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
            condition=IfCondition(PythonExpression(["'", robot_model, "' == 'ranger_mini_v2'"])),
            launch_arguments={
                "robot_base": "ranger_mini",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("realsense2_camera"),
                    "examples",
                    "align_depth",
                    "rs_aligned_depth.launch.py",
                ])
            ])
        ),
    ])

    return LaunchDescription([
        declare_use_namespace_cmd,
        declare_namespace_cmd,
        declare_use_sim_time_cmd,
        declare_robot_model_cmd,
        robot_base_bringup,
        chassis_bringup,
        sensor_kit_bringup
    ])
