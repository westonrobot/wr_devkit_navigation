from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition


def generate_launch_description():
    use_namespace = LaunchConfiguration("use_namespace")
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    imu_path = LaunchConfiguration("imu_path")
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

    declare_imu_path_cmd = DeclareLaunchArgument(
        "imu_path",
        default_value="/dev/ttyUSB0",
        description="Path to IMU port",
    )

    declare_robot_model_cmd = DeclareLaunchArgument(
        "robot_model",
        default_value="ranger_mini_v2",
        description="Robot base model",
    )

    robot_bringup = GroupAction(
        [
            PushRosNamespace(condition=IfCondition(use_namespace), namespace=namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("wr_devkit_robot_bringup"),
                                "launch",
                                "wr_devkit_sensor_bringup.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "use_namespace": "false",  # Set to false as this could stack twice
                    "namespace": namespace,
                    "use_sim_time": use_sim_time,
                    "imu_path": imu_path,
                }.items(),
            ),
            IncludeLaunchDescription(
                XMLLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("ranger_base"),
                                "launch",
                                "include",
                                "ranger_robot_base.launch.xml",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "publish_odom_tf": "true", # Set to True for nav2 stack
                    "robot_model": robot_model
                }.items(),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="devkit_transform_publisher",
                arguments=[
                    "--x",
                    "0.0",
                    "--y",
                    "-0.0",
                    "--z",
                    "0.465",
                    "--yaw",
                    "0",
                    "--pitch",
                    "0",
                    "--roll",
                    "0",
                    "--frame-id",
                    "base_link",
                    "--child-frame-id",
                    "wr_devkit_base_link",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_footprint_publisher",
                arguments=[
                    "--x",
                    "0.0",
                    "--y",
                    "-0.0",
                    "--z",
                    "0.0",
                    "--yaw",
                    "0",
                    "--pitch",
                    "0",
                    "--roll",
                    "0",
                    "--frame-id",
                    "base_link",
                    "--child-frame-id",
                    "base_footprint",
                ],
            ),
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch arguments
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_imu_path_cmd)
    ld.add_action(declare_robot_model_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(robot_bringup)

    return ld
