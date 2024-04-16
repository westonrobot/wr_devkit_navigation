import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.conditions import IfCondition


def generate_launch_description():
    description_pkg_path = get_package_share_directory("wr_devkit_description")
    livox_pkg_path = get_package_share_directory("livox_ros_driver2")
    wrp_ros2_pkg_path = get_package_share_directory("wrp_ros2")

    use_namespace = LaunchConfiguration("use_namespace")
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")

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

    livox_bringup = GroupAction(
        [
            PushRosNamespace(condition=IfCondition(use_namespace), namespace=namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            get_package_share_directory("livox_ros_driver2"),
                            "launch_ROS2",
                            "rviz_MID360_launch.py",
                        )
                    ]
                ),
                launch_arguments={}.items(),
            ),
        ]
    )

    pointcloud_to_laserscan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
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
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch arguments
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    # Add the actions to launch all of the navigation nodes
    # ld.add_action(description_bringup)
    ld.add_action(livox_bringup)
    ld.add_action(pointcloud_to_laserscan_node)

    return ld
