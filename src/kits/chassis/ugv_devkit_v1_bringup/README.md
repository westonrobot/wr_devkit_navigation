# UGV Development Kit V1.0 Bringup

![UGV Development Kit V1.0](docs/ugv_devkit_v1.jpg)

## Overview
This package contains launch and config files to launch Weston Robot's UGV Development Kit V1.0 (chassis).

## Usage
To launch the UGV Development Kit V1.0, use the provided launch file:
```bash
ros2 launch ugv_devkit_v1_bringup chassis.launch.py
```

## Launch Files
* [chassis.launch.py](./launch/chassis.launch.py)
  | Argument      | Description                  | Default Value |
  | ------------- | ---------------------------- | ------------- |
  | use_namespace | Whether to apply a namespace | False         |
  | namespace     | Top-level namespace          | ""            |

## Nodes
The package will launch the various sensors and their associated driver nodes/supporting nodes

* Description
  * Package: robot_state_publisher
  * Executable/Plugin: robot_state_publisher
  * Name: ugv_devkit_v1_sensor_kit_state_publisher

## Configuration
To better suit your needs/setup, you may need to adjust these accordingly
* [Launch files](./launch/)