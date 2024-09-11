from launch import LaunchDescription
from launch.actions import GroupAction, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.launch_context import LaunchContext
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitution import Substitution
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.utilities import perform_substitutions, normalize_to_list_of_substitutions
from typing import Dict, Text

import os
import xacro


class Xacro(Substitution):
    def __init__(self, file_path: SomeSubstitutionsType, mappings: Dict[str, SomeSubstitutionsType] = {}, verbose: bool = False):
        super().__init__()
        self.__file_path = normalize_to_list_of_substitutions(file_path)
        self.__mappings = {key: normalize_to_list_of_substitutions(
            value) for key, value in mappings.items()}
        self.__verbose = verbose

    def describe(self) -> Text:
        return f"Xacro: {self.__file_path}"

    def perform(self, context: LaunchContext) -> Text:
        file_path = perform_substitutions(context, self.__file_path)
        mappings = {key: perform_substitutions(
            context, value) for key, value in self.__mappings.items()}

        if self.__verbose:
            print(f"xacro file_path: {file_path}")
            print(f"xacro mappings: {mappings}")
        document = xacro.process_file(file_path, mappings=mappings)
        document_string = document.toprettyxml(indent="  ")

        if self.__verbose:
            print(f"xacro result: {document_string}")
        return document_string


def generate_launch_description():

    # --------- Arguments ---------
    declare_use_namespace_arg = DeclareLaunchArgument(
        "use_namespace",
        default_value="false",
        description="Whether to apply a namespace"
    )

    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Top-level namespace"
    )

    declare_front_camera_arg = DeclareLaunchArgument(
        "front_camera",
        default_value="",
        description="Front camera type",
    )

    declare_rear_camera_arg = DeclareLaunchArgument(
        "rear_camera",
        default_value="",
        description="Rear camera type",
    )

    declare_left_camera_arg = DeclareLaunchArgument(
        "left_camera",
        default_value="",
        description="Left camera type",
    )

    declare_right_camera_arg = DeclareLaunchArgument(
        'right_camera',
        default_value="",
        description='Right camera type'
    )

    front_camera_type = LaunchConfiguration("front_camera")
    rear_camera_type = LaunchConfiguration("rear_camera")
    left_camera_type = LaunchConfiguration("left_camera")
    right_camera_type = LaunchConfiguration("right_camera")

    # --------- Namespace ---------
    PushRosNamespace(
        condition=IfCondition(LaunchConfiguration("use_namespace")),
        namespace=LaunchConfiguration("namespace")
    )

    # --------- Description ---------
    load_description = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="vision_sensor_kit_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": Xacro(
                file_path=os.path.join(
                    FindPackageShare("vision_sensor_kit_bringup").find(
                        "vision_sensor_kit_bringup"),
                    "urdf/sensor_kit.urdf.xacro"
                ),
                mappings={
                    'front_camera_type': front_camera_type,
                    'rear_camera_type': rear_camera_type,
                    'left_camera_type': left_camera_type,
                    'right_camera_type': right_camera_type
                },
            )
            }
        ],
        remappings=[
            ("robot_description", "vision_sensor_kit_description"),
        ]
    )

    # --------- Drivers ---------
    # --------- Cameras ---------
    camera_types = [front_camera_type, rear_camera_type,
                    left_camera_type, right_camera_type]
    camera_positions = ['front', 'rear', 'left', 'right']

    realsense_camera_bringup = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("realsense2_camera"),
                    "launch",
                    "rs_launch.py",
                ])
            ]),
            condition=IfCondition(PythonExpression(
                ["'", cam_type, "' == 'realsense_d435i'"])),
            launch_arguments={
                "camera_name": "d435i",
                "camera_namespace": pos + "_cam",
                "config_file": PathJoinSubstitution([FindPackageShare("vision_sensor_kit_bringup"), 'config', pos + '_d435.param.yaml'])
            }.items(),
        ) for cam_type, pos in zip(camera_types, camera_positions)
    ])

    rgb_camera_bringup = GroupAction([
        Node(
            condition=IfCondition(PythonExpression(
                ["'", cam_type, "' == 'rgb_camera'"])),
            package="usb_cam",
            executable="usb_cam_node_exe",
            namespace=pos + "_cam",
            name="rgb",
            output="screen",
            parameters=[
                PathJoinSubstitution([FindPackageShare(
                    "vision_sensor_kit_bringup"), 'config', 'rgb_camera.param.yaml'])
            ],
            remappings=[
                ('image_raw', f'{pos}_rgb/image_raw'),
                ('image_raw/compressed', f'{pos}_rgb/image_raw/compressed'),
                ('image_raw/compressedDepth',
                 f'{pos}_rgb/image_raw/compressedDepth'),
                ('image_raw/theora', f'{pos}_rgb/image_raw/theora'),
                ('camera_info', f'{pos}_rgb/camera_info')
            ]
        ) for cam_type, pos in zip(camera_types, camera_positions)
    ])

    return LaunchDescription([
        declare_use_namespace_arg,
        declare_namespace_arg,
        declare_front_camera_arg,
        declare_rear_camera_arg,
        declare_left_camera_arg,
        declare_right_camera_arg,
        load_description,
        realsense_camera_bringup,
        rgb_camera_bringup
    ])
