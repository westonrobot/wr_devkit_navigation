from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition


def generate_launch_description():
    config = LaunchConfiguration("config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    resolution = LaunchConfiguration("resolution")
    publish_period_sec = LaunchConfiguration("publish_period_sec")

    declare_config_path_arg = DeclareLaunchArgument(
        "config_path",
        default_value=PathJoinSubstitution([
            FindPackageShare("wr_devkit_bringup"), "config"
        ]),
        description="Full path to the configuration file to load",
    )

    declare_config_arg = DeclareLaunchArgument(
        "config",
        default_value="cartographer.lua",
        description="Full path to the configuration file to load",
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_resolution_arg = DeclareLaunchArgument(
        "resolution",
        default_value="0.05",
        description="Resolution of the map",
    )

    declare_publish_period_sec_arg = DeclareLaunchArgument(
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
            LaunchConfiguration("config_path"),
            "-configuration_basename",
            config,
        ],
        remappings=[("/imu", "/ch104m_imu_sensor_node/imu")]
    )

    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec])

    return LaunchDescription([
        declare_config_path_arg,
        declare_config_arg,
        declare_use_sim_time,
        declare_resolution_arg,
        declare_publish_period_sec_arg,
        cartographer_node,
        cartographer_occupancy_grid_node,
    ])
