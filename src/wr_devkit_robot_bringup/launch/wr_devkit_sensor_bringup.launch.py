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
    imu_path = LaunchConfiguration("imu_path")

    declare_use_namespace_cmd = DeclareLaunchArgument(
        "use_namespace",
        default_value="false",
        description="Whether to apply a namespace to the navigation nodes",
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_imu_path_cmd = DeclareLaunchArgument(
        "imu_path",
        default_value="/dev/ttyUSB0",
        description="Path to IMU port",
    )

    description_bringup = GroupAction(
        [
            PushRosNamespace(condition=IfCondition(use_namespace), namespace=namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution([
                            FindPackageShare("wr_devkit_description"),
                            "launch/wr_devkit_description.launch.py"]
                        )
                    ]
                ),
                launch_arguments={}.items(),
            ),
        ]
    )

    livox_bringup = GroupAction(
        [
            PushRosNamespace(condition=IfCondition(use_namespace), namespace=namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution([
                            FindPackageShare("wr_devkit_robot_bringup"),
                            "launch/mid360.launch.py"]
                        )
                    ]
                ),
                launch_arguments={}.items(),
            ),
        ]
    )

    imu_bringup = GroupAction(
        [
            PushRosNamespace(condition=IfCondition(use_namespace), namespace=namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare("wrp_ros2"),
                        "launch/peripheral/imu_sensor.launch.py"]
                    )
                ),
                launch_arguments={
                    "device_path": imu_path,
                    "baud_rate": "115200",
                    "frame_id": "ch104m_imu_frame",
                    "sensor_model": "hipnuc",
                }.items(),
            ),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="pointcloud_to_laserscan_node",
                output="screen",
                parameters=[
                    {"range_min": 0.1},
                    {"range_max": 100.0},
                    {"scan_time": 0.1},
                    {"min_height": -0.025},
                    {"max_height": 0.025},
                    {"angle_min": -3.14159265},
                    {"angle_max": 3.14159265},
                    {"angle_increment": 0.0174532925},
                    {"inf_epsilon": 1.0},
                    {"tf_tolerance": 0.03},
                ],
                remappings=[
                    ("/cloud_in", "/livox/lidar"),
                    ("/scan", "/scan"),
                ],
            ),
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch arguments
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_imu_path_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(description_bringup)
    ld.add_action(livox_bringup)
    ld.add_action(imu_bringup)

    return ld
