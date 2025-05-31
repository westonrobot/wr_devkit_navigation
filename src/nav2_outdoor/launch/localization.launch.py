#/bin/python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('nav2_outdoor')

  
    map_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_transform',
        output='screen',
        arguments = "--x -1 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 --frame-id map --child-frame-id odom".split(' '),
        )
    
    ukf_localization_node = Node(
        package='robot_localization',
        executable='ukf_node',
        name='ukf_node',
        output='screen',
        respawn=True,
        parameters=[os.path.join(pkg_share, 'config/ukf.yaml')],
        remappings=[
            ('/odometry/filtered', '/odom'),
        ]
        )

    return LaunchDescription(
        [
            ukf_localization_node,
            map_transform_node,
        ]
    )


