from launch import LaunchDescription, LaunchContext
from launch.launch_context import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.substitution import Substitution
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.utilities import normalize_to_list_of_substitutions
from launch.utilities import perform_substitutions
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

    declare_chassis_extension_arg = DeclareLaunchArgument(
        "chassis_extension",
        default_value="false",
        description="UGV Devkit Chassis Extension V1.1"
    )

    # --------- Namespace ---------
    PushRosNamespace(
        condition=IfCondition(LaunchConfiguration("use_namespace")),
        namespace=LaunchConfiguration("namespace")
    )

    # --------- Description ---------
    xacro_file = os.path.join(
        FindPackageShare("ugv_devkit_bringup").find(
            "ugv_devkit_bringup"),
        "urdf/chassis.urdf.xacro"
    )

    load_description = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="ugv_devkit_sensor_kit_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": Xacro(
                file_path=xacro_file,
                mappings={"chassis_ext": LaunchConfiguration(
                    "chassis_extension")}
            )}
        ],
        remappings=[
            ("robot_description", "ugv_devkit_chassis_description"),
        ]
    )

    return LaunchDescription([
        declare_use_namespace_arg,
        declare_namespace_arg,
        declare_chassis_extension_arg,
        load_description,
    ])
