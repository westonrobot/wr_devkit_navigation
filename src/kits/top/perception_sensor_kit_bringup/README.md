# Perception Sensor Kit Bringup

![Mid360 Sensor Kit](docs/mid360_sensor_kit.png)

## Overview
This package contains launch and config files to launch Weston Robot's Mid360 Sensor Kit (top).

## Usage
To launch the Mid360 Sensor Kit, use the provided launch file:
```bash
ros2 launch perception_sensor_kit_bringup sensor_kit.launch.py
```

## Launch Files
* [sensor_kit.launch.py](./launch/sensor_kit.launch.py)
  * Sample launch file to launch the top sensor kit sensors with the following configuration
    * Camera model: realsense_d435 / rgb_camera

  | Argument      | Description                  | Default Value |
  | ------------- | ---------------------------- | ------------- |
  | use_namespace | Whether to apply a namespace | False         |
  | namespace     | Top-level namespace          | ""            |
  | front_camera  | Front camera model           | none          |
  | rear_camera   | Rear camera model            | none          |
  | left_camera   | Left camera model            | none          |
  | right_camera  | Right camera model           | none          |

## Nodes
The package will launch the various sensors and their associated driver nodes/supporting nodes

* Description
  * Package: robot_state_publisher
  * Executable/Plugin: robot_state_publisher
  * Name: perception_sensor_kit_state_publisher
* IMU
  * Driver
    * Package: wrp_ros2
    * Executable/Plugin: imu_sensor_node
    * Name: ch104m_imu_sensor_node
* Lidar
  * Driver
    * Package: livox_ros_driver2
    * Executable/Plugin: livox_ros_driver2_node
    * Name: livox_lidar_publisher
  * Pointcloud -> LaserScan
    * Package: pointcloud_to_laserscan
    * Executable/Plugin: pointcloud_to_laserscan_node
    * Name: pointcloud_to_laserscan_node

## Configuration
To better suit your needs/setup, you may need to adjust these accordingly
* [Configuration files](./config/)
* [Launch files](./launch/)