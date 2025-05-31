from launch_ros.actions import Node

ukf_localization_node = Node(
    package="robot_localization",
    executable="ukf_node",
    name="ukf_node",
    output="screen",
    parameters=[PathJoinSubstitution([
        FindPackageShare("your_package_name"),
        "config",
        "ukf_localization.yaml"
    ])],
    remappings=[
        ("imu0", "/imu"),
        ("odometry/filtered", "/odom/localization")
    ]
)

