from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
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
        'GFTT/MinDistance':'5',
        'GFTT/QualityLevel':'0.0001',
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
        ('rgb/image', '/front_d435/front_d435/color/image_raw'),
        ('rgb/camera_info', '/front_d435/front_d435/color/camera_info'),
        ('depth/image', '/front_d435/front_d435/depth/image_rect_raw')
    ]

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_qos_cmd = DeclareLaunchArgument(
        'qos', 
        default_value='2', 
        description='QoS used for input sensor topics'
    )

    declare_localization_cmd = DeclareLaunchArgument(
        'localization',
        default_value='false',
        description='Launch in localization mode.'
    )

    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch rviz2'
    )

    # ----------- RTAB-Map -----------
    rtabmap_slam_bringup = GroupAction([
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
            condition=IfCondition(AndSubstitution(NotSubstitution(localization), rviz)),
            package='rviz2', 
            executable='rviz2', 
            arguments=[PathJoinSubstitution([FindPackageShare('wr_devkit_bringup'), 'rtabmap.rviz'])],
            output='screen'
        ),
    ])

    rtabmap_localization_bringup = GroupAction([
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
    ])

    # Uncomment to launch RTAB-Map's visualization tool:
    rtabmap_viz_bringup = GroupAction([
        # Node(
        #     package='rtabmap_viz', 
        #     executable='rtabmap_viz', 
        #     output='screen',
        #     parameters=[parameters],
        #     remappings=remappings
        # ),
    ])

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_qos_cmd,
        declare_localization_cmd,
        declare_rviz_cmd,
        rtabmap_slam_bringup,
        rtabmap_localization_bringup,
        rtabmap_viz_bringup
    ])