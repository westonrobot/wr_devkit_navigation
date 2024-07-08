from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')
    localization = LaunchConfiguration('localization')

    parameters={
        'frame_id':'base_link',
        'use_sim_time':use_sim_time,
        'subscribe_depth':True,
        'subscribe_rgbd':False,
        'subscribe_rgb':True,
        'subscribe_scan':True,
        'use_action_for_goal':True,
        'wait_for_transform':0.2,
        'qos_image':qos,
        'qos_scan':qos,
        'qos_camera_info':qos,
        'approx_sync':True,
        'Reg/Force3DoF':'true',
        'Optimizer/GravitySigma':'0',
        'Grid/Sensor':'2'
    }

    remappings=[
        ('odom','/odom'),
        ('scan','/scan'),
        ('rgb/image', '/camera/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/camera/color/camera_info'),
        ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw')
    ]

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            'qos', default_value='2', description='QoS used for input sensor topics'),
            
        DeclareLaunchArgument(
            'localization', default_value='false', description='Launch in localization mode.'),

        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', 
            executable='rtabmap', 
            output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=['-d']
        ),
        
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', 
            executable='rtabmap', 
            output='screen',
            parameters=[parameters,
              {'Mem/IncrementalMemory':'False',
               'Mem/InitWMWithAllNodes':'True'}],
            remappings=remappings
        ),

        Node(
            package='rtabmap_viz', 
            executable='rtabmap_viz', 
            output='screen',
            parameters=[parameters],
            remappings=remappings
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_base_camera',
            arguments=['--x', '0.0', '--y', '0.0', '--z', '0.150',
                    '--yaw', '0', '--pitch', '0', '--roll', '0',
                    '--frame-id', 'base_link', '--child-frame-id', 'camera_link']
        )
    ])