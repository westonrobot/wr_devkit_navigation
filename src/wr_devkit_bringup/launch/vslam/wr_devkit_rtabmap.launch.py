from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, AndSubstitution, NotSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')
    localization = LaunchConfiguration('localization')
    rviz = LaunchConfiguration('rviz')

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
        # RTAB-Map's internal parameters are strings:
        'Reg/Force3DoF':'true',
        'Optimizer/GravitySigma':'0',
        'Grid/RangeMax':'0',
        'RGBD/ProximityMaxGraphDepth':'0',
        'RGBD/ProximityPathMaxNeighbors':'1',
        'RGBD/AngularUpdate':'0.05',
        'RGBD/LinearUpdate':'0.05',
        'RGBD/CreateOccupancyGrid':'false',
        'Mem/NotLinkedNodesKept':'false',
        'Mem/STMSize':'30',
        'Mem/LaserScanNormalK':'20',
        'Reg/Strategy':'1',
        'Icp/VoxelSize':'0.1',
        'Icp/PointToPlaneK':'20',
        'Icp/PointToPlaneRadius':'0',
        'Icp/PointToPlane':'true',
        'Icp/Iterations':'10',
        'Icp/Epsilon':'0.001',
        'Icp/MaxTranslation':'3',
        'Icp/MaxCorrespondenceDistance':'1',
        'Icp/Strategy':'1',
        'Icp/OutlierRatio':'0.7',
        'Icp/CorrespondenceRatio':'0.2'
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

        DeclareLaunchArgument(
            'rviz', default_value='false', description='Launch rviz2'),

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

        # Uncomment to launch RTAB-Map's visualization tool:
        # Node(
        #     package='rtabmap_viz', 
        #     executable='rtabmap_viz', 
        #     output='screen',
        #     parameters=[parameters],
        #     remappings=remappings
        # ),

        Node(
            condition=IfCondition(AndSubstitution(NotSubstitution(localization), rviz)),
            package='rviz2', 
            executable='rviz2', 
            output='screen',
            arguments=[PathJoinSubstitution([FindPackageShare('wr_devkit_bringup'), 'rtabmap.rviz'])]
        ),

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='base_link_to_base_camera',
        #     arguments=['--x', '0.0', '--y', '0.0', '--z', '0.150',
        #             '--yaw', '0', '--pitch', '0', '--roll', '0',
        #             '--frame-id', 'base_link', '--child-frame-id', 'camera_link']
        # )
    ])