import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # パラメータ宣言
    camera_topic = LaunchConfiguration('camera_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    marker_id = LaunchConfiguration('marker_id')
    marker_size = LaunchConfiguration('marker_size')
    approach_distance = LaunchConfiguration('approach_distance')
    linear_speed = LaunchConfiguration('linear_speed')
    angular_speed = LaunchConfiguration('angular_speed')
    
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
        DeclareLaunchArgument(
            'approach_distance',
            default_value='0.5',
            description='マーカーへの接近距離（メートル）'
        ),
        DeclareLaunchArgument(
            'linear_speed',
            default_value='0.2',
            description='直線速度の最大値（メートル/秒）'
        ),
        DeclareLaunchArgument(
            'angular_speed',
            default_value='0.3',
            description='角速度の最大値（ラジアン/秒）'
        ),
        
        # 各ノードを起動
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
        ),
        
        Node(
            package='aruco_charger_navigation',
            executable='charger_navigator',
            name='charger_navigator',
            parameters=[{
                'approach_distance': approach_distance,
                'linear_speed': linear_speed,
                'angular_speed': angular_speed
            }],
            output='screen'
        )
    ])
