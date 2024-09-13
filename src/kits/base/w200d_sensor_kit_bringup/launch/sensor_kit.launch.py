from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.substitution import Substitution
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.conditions import IfCondition
from launch.launch_context import LaunchContext
from launch.utilities import perform_substitutions, normalize_to_list_of_substitutions
from typing import Dict, Text

import os
import xacro

class Xacro(Substitution):
    def __init__(self, file_path: SomeSubstitutionsType, mappings: Dict[str, SomeSubstitutionsType] = {}, verbose: bool = False):
        super().__init__()
        self.__file_path = normalize_to_list_of_substitutions(file_path)
        self.__mappings = {key: normalize_to_list_of_substitutions(value) for key, value in mappings.items()}
        self.__verbose = verbose
    
    def describe(self) -> Text:
        return f"Xacro: {self.__file_path}"

    def perform(self, context: LaunchContext) -> Text:
        file_path = perform_substitutions(context, self.__file_path)
        mappings = {key: perform_substitutions(context, value) for key, value in self.__mappings.items()}

        if self.__verbose:
            print(f"xacro file_path: {file_path}")
            print(f"xacro mappings: {mappings}")
        document = xacro.process_file(file_path, mappings=mappings)
        document_string = document.toprettyxml(indent="  ")

        if self.__verbose:
            print(f"xacro result: {document_string}")
        return document_string

def launch_setup(context, *args, **kwargs):
   # --------- Namespace ---------
    PushRosNamespace(
        condition=IfCondition(LaunchConfiguration("use_namespace")),
        namespace=LaunchConfiguration("namespace")
    )

    # --------- Description ---------
    load_description = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="w200d_sensor_kit_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": Xacro(
                    file_path=os.path.join(
                        FindPackageShare("w200d_sensor_kit_bringup").find(
                            "w200d_sensor_kit_bringup"),
                        "urdf/sensor_kit.urdf.xacro"
                    ),
                    mappings={
                        'robot_base': LaunchConfiguration("robot_base")
                    }
                )
            }
        ],
        remappings=[
            ("robot_description", "w200d_sensor_kit_description"),
        ]
    )

    # --------- Drivers ---------

    ultrasonic_bringup = Node(
        package="wrp_ros2",
        name="ultrasonic_sensor_node",
        executable="ultrasonic_sensor_node",
        output="screen",
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('w200d_sensor_kit_bringup'),
                'config',
                'w200d.param.yaml']
            )],
    )

    return [
        load_description,
        ultrasonic_bringup
    ]


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

    declare_robot_base_arg = DeclareLaunchArgument(
        "robot_base",
        default_value="ranger_mini",
        description="What ultrasonic offsets to use, refer to param file"
    )

    return LaunchDescription([
        declare_use_namespace_arg,
        declare_namespace_arg,
        declare_robot_base_arg,
    ]
        + [OpaqueFunction(function=launch_setup)])
