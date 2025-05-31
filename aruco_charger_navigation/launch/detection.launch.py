import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # パッケージのパスを取得
    aruco_pkg_dir = FindPackageShare('aruco_charger_navigation')
    
    # パラメータ宣言
    camera_topic = LaunchConfiguration('camera_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    marker_id = LaunchConfiguration('marker_id')
    marker_size = LaunchConfiguration('marker_size')
    
    return LaunchDescription([
        # Launch引数の宣言
        DeclareLaunchArgument(
            'camera_topic',
            default_value='/image_raw',
            description='カメラ画像トピック'
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/camera_info',
            description='カメラキャリブレーション情報トピック'
        ),
        DeclareLaunchArgument(
            'marker_id',
            default_value='0',
            description='検出対象のAruCoマーカーID'
        ),
        DeclareLaunchArgument(
            'marker_size',
            default_value='0.05',
            description='マーカーのサイズ（メートル）'
        ),
        
        # AruCoマーカー検出ノード
        Node(
            package='aruco_charger_navigation',
            executable='aruco_detector',
            name='aruco_detector',
            parameters=[{
                'camera_topic': camera_topic,
                'camera_info_topic': camera_info_topic,
                'marker_id': marker_id,
                'marker_size': marker_size
            }],
            output='screen'
        )
    ])
