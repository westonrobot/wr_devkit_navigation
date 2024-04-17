import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory("wr_devkit_nav_bringup"), "params"
    )
    config = LaunchConfiguration("config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    resolution = LaunchConfiguration("resolution")
    publish_period_sec = LaunchConfiguration("publish_period_sec")

    declare_config_cmd = DeclareLaunchArgument(
        "config",
        default_value="cartographer.lua",
        description="Full path to the configuration file to load",
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_resolution_cmd = DeclareLaunchArgument(
        "resolution",
        default_value="0.05",
        description="Resolution of the map",
    )

    declare_publish_period_sec_cmd = DeclareLaunchArgument(
        "publish_period_sec",
        default_value="1.0",
        description="Publish period in seconds",
    )

    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "-configuration_directory",
            config_path,
            "-configuration_basename",
            config,
        ],
        remappings=[("/imu", "/imu_sensor_node/imu")]
    )

    cartographer_occupancy_grid_node = Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_config_cmd)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_resolution_cmd)
    ld.add_action(declare_publish_period_sec_cmd)

    # Declare node
    ld.add_action(cartographer_node)
    ld.add_action(cartographer_occupancy_grid_node)
    return ld
