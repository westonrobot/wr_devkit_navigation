# Weston Robot Devkit Bringup

## Overview
This package contains sample launch and config files to launch Weston Robot's Devkit and sample applications.

This package is meant to be a starting point for users to build upon and customize to their needs, and shows how to compose the various kits of the Devkit platform.

## Launch files
* [wr_devkit_platform.launch.py](./launch/platform/wr_devkit_platform.launch.py)
  * Sample launch file to launch the Weston Robot Devkit platform with the following configuration
    * Robot base: Ranger mini v2 / Scout mini
    * Chassis: ugv_devkit_v1
    * Base sensor kit: w200d_sensor_kit
    * Top sensor kit: mid360_sensor_kit
    * Camera model: realsense_d435 / rgb_camera
  
    | Argument      | Description                  | Default Value    |
    | ------------- | ---------------------------- | ---------------- |
    | use_namespace | Whether to apply a namespace | False            |
    | namespace     | Top-level namespace          | ""               |
    | use_sim_time  | Whether to use sim time      | False            |
    | robot_model   | Whether to use sim time      | ranger_mini_v2   |
    | front_camera  | Front camera model           | none             |
    | rear_camera   | Rear camera model            | none             |
    | left_camera   | Left camera model            | none             |
    | right_camera  | Right camera model           | none             |

* [wr_devkit_cartographer.launch.py](./launch/slam/wr_devkit_cartographer.launch.py)
  * Sample launch file to perform 2D SLAM using cartographer
  * Assumes the following configuration by default
    * Top sensor kit: mid360_sensor_kit

  | Argument           | Description                   | Default Value            |
  | ------------------ | ----------------------------- | ------------------------ |
  | config_path        | Path to config file directory | wr_devkit_bringup/config |
  | config             | Config file name              | cartographer.lua         |
  | resolution         | Map resolution                | 0.05                     |
  | publish_period_sec | Publish period                | 1.0                      |
  | use_sim_time       | Whether to use sim time       | False                    |

* [wr_devkit_rtabmap.launch.py](./launch/slam/wr_devkit_cartographer.launch.py)
  * Sample launch file to perform 2D vSLAM using rtabmap
  * Assumes the following configuration by default
    * Top sensor kit: mid360_sensor_kit + realsense d435

  | Argument           | Description                                 | Default Value |
  | ------------------ | ------------------------------------------- | ------------- |
  | use_sim_time       | Whether to use simulation time              | False         |
  | qos                | Quality of Service                          | 2             |
  | localization       | Whether to use RTAB-Map localization mode   | False         |
  | rviz               | Whether to launch RViz automatically        | False         |

* [wr_devkit_nav2.launch.py](./launch/nav2/wr_devkit_nav2.launch.py)
  * Sample launch file to perform 2D navigation using nav2
    * Robot base: Ranger mini v2
    * Chassis: ugv_devkit_v1
    * Top sensor kit: mid360_sensor_kit

  | Argument       | Description                  | Default Value                                        |
  | -------------- | ---------------------------- | ---------------------------------------------------- |
  | use_namespace  | Whether to apply a namespace | False                                                |
  | namespace      | Top-level namespace          | ""                                                   |
  | robot_param    | Robot Nav2 param file        | nav2_ranger_mini.param.yaml                          |
  | params_file    | Nav2 param file              | wr_devkit_bringup/config/robot_param                 |
  | map            | Map file                     | map.yaml                                             |
  | autostart      | To autostart nodes           | True                                                 |
  | container_name | Node container name          | "nav2_container"                                     |
  | log_level      | LOG level                    | "info                                                |
  | use_sim_time   | Whether to use sim time      | False                                                |

* [wr_devkit_nav2_rtab.launch.py](./launch/nav2/wr_devkit_nav2.launch.py)
  * Sample launch file to perform 2D navigation using nav2
    * Robot base: Scout mini
    * Chassis: ugv_devkit_v1
    * Top sensor kit: mid360_sensor_kit + realsense d435

  | Argument       | Description                  | Default Value                                        |
  | -------------- | ---------------------------- | ---------------------------------------------------- |
  | use_namespace  | Whether to apply a namespace | False                                                |
  | namespace      | Top-level namespace          | ""                                                   |
  | robot_param    | Robot Nav2 param file        | nav2_scout_mini.param.yaml                          |
  | params_file    | Nav2 param file              | wr_devkit_bringup/config/robot_param                 |                           |
  | autostart      | To autostart nodes           | True                                                 |
  | container_name | Node container name          | "nav2_container"                                     |
  | log_level      | LOG level                    | "info                                                |
  | use_sim_time   | Whether to use sim time      | False                                                |