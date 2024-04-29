from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition


def generate_launch_description():
    use_namespace = LaunchConfiguration("use_namespace")
    namespace = LaunchConfiguration("namespace")
    device_path = LaunchConfiguration("device_path")

    declare_use_namespace_cmd = DeclareLaunchArgument(
        "use_namespace",
        default_value="false",
        description="Whether to apply a namespace to nodes",
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_device_path_cmd = DeclareLaunchArgument(
        "device_path",
        default_value="/dev/ttyS0",
        description="Path to device port",
    )

    ultrasonic_bringup = GroupAction(
        [
            PushRosNamespace(condition=IfCondition(use_namespace), namespace=namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("wr_devkit_description"),
                                "launch/wr_devkit_ultrasonic.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("wrp_ros2"),
                                "launch/peripheral/ultrasonic_sensor.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "sensor_model": "w200d",
                    "device_path": device_path,
                }.items(),
            ),
        ]
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_device_path_cmd)
    ld.add_action(ultrasonic_bringup)

    return ld
