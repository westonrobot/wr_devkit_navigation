from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace, SetParameter, LoadComposableNodes
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from nav2_common.launch import RewrittenYaml, ReplaceString
from launch_ros.descriptions import ComposableNode, ParameterFile

def generate_launch_description():
    # Get the launch directory
    bringup_dir = FindPackageShare("wr_devkit_bringup")

    use_namespace = LaunchConfiguration("use_namespace")
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    robot_param = LaunchConfiguration("robot_param")
    params_file = LaunchConfiguration("params_file")
    container_name = LaunchConfiguration("container_name")
    container_name_full = (namespace, "/", container_name)
    log_level = LaunchConfiguration("log_level")

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    lifecycle_nodes = [
        "amcl",
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "velocity_smoother",
        "collision_monitor",
        "bt_navigator",
        "waypoint_follower",
    ]

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        "autostart": autostart,
        "use_sim_time": use_sim_time,
    }

    # Only it applys when `use_namespace` is True.
    # '<robot_namespace>' keyword shall be replaced by 'namespace' launch argument
    # in config file 'nav2_multirobot_params.yaml' as a default & example.
    # User defined config file should contain '<robot_namespace>' keyword for the replacements.
    params_file = ReplaceString(
        source_file=params_file,
        replacements={"<robot_namespace>": ("/", namespace)},
        condition=IfCondition(use_namespace),
    )

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

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

    declare_robot_param_cmd = DeclareLaunchArgument(
        "robot_param",
        default_value="nav2_scout_mini_rtab.param.yaml",
        description="Path to the scout mini robot parameters file",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution([
            bringup_dir, "config", robot_param,
        ]),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )

    declare_container_name_cmd = DeclareLaunchArgument(
        "container_name",
        default_value="nav2_container",
        description="the name of conatiner that nodes will load in if use composition",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    load_composable_nodes = GroupAction(
        actions=[
            SetParameter("use_sim_time", use_sim_time),
            PushRosNamespace(condition=IfCondition(
                use_namespace), namespace=namespace),
            Node(
                name="nav2_container",
                package="rclcpp_components",
                executable="component_container_isolated",
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
                output="screen",
            ),
            LoadComposableNodes(
                target_container=container_name_full,
                composable_node_descriptions=[
                    ComposableNode(
                        package="nav2_controller",
                        plugin="nav2_controller::ControllerServer",
                        name="controller_server",
                        parameters=[configured_params],
                        remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
                    ),
                    ComposableNode(
                        package="nav2_smoother",
                        plugin="nav2_smoother::SmootherServer",
                        name="smoother_server",
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package="nav2_planner",
                        plugin="nav2_planner::PlannerServer",
                        name="planner_server",
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package="nav2_behaviors",
                        plugin="behavior_server::BehaviorServer",
                        name="behavior_server",
                        parameters=[configured_params],
                        remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
                    ),
                    ComposableNode(
                        package="nav2_bt_navigator",
                        plugin="nav2_bt_navigator::BtNavigator",
                        name="bt_navigator",
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package="nav2_waypoint_follower",
                        plugin="nav2_waypoint_follower::WaypointFollower",
                        name="waypoint_follower",
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package="nav2_velocity_smoother",
                        plugin="nav2_velocity_smoother::VelocitySmoother",
                        name="velocity_smoother",
                        parameters=[configured_params],
                        remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
                    ),
                    ComposableNode(
                        package="nav2_collision_monitor",
                        plugin="nav2_collision_monitor::CollisionMonitor",
                        name="collision_monitor",
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package="nav2_lifecycle_manager",
                        plugin="nav2_lifecycle_manager::LifecycleManager",
                        name="lifecycle_manager_navigation",
                        parameters=[
                            {"autostart": autostart, "node_names": lifecycle_nodes}
                        ],
                    ),
                    ComposableNode(
                        package="nav2_amcl",
                        plugin="nav2_amcl::AmclNode",
                        name="amcl",
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                ],
            ),
        ],
    )

    return LaunchDescription([
        stdout_linebuf_envvar,
        declare_use_namespace_cmd,
        declare_namespace_cmd,
        declare_use_sim_time_cmd,
        declare_robot_param_cmd,
        declare_params_file_cmd,
        declare_autostart_cmd,
        declare_container_name_cmd,
        declare_log_level_cmd,
        load_composable_nodes,
    ])
