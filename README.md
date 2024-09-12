# WR Devkit Navigation

![default workflow](https://github.com/westonrobot/wr_devkit_navigation/actions/workflows/default.yml/badge.svg?branch=main)

This repository provides a head start for mobile robot development when using the development kit from Weston Robot. 

The code contained in this repository is meant to be used as a reference for integrating Weston Robot's hardware with ROS2's Navigation2 stack. It includes sample launch files for bringing up the robot, running SLAM, and navigation applications.

**Code is provided `as-is` and may require modifications to work with your specific hardware configuration.**

## Hardware Requirements
The following hardware configurations are supported:

### Chassis
| Name                     | Documentation                                                       | Source Code                                                        |
| ------------------------ | ------------------------------------------------------------------- | ------------------------------------------------------------------ |
| UGV Development Kit V1.0 | [wiki](https://docs.westonrobot.com/wr_dev_kit/ugv_dev_kit_v1.html) | [ugv_devkit_v1_bringup](./src/kits/chassis/ugv_devkit_v1_bringup/) |

### Sensor Kits
| Sensor kit               | Mount Location | Documentation                                                                                                                             | Source Code                                                            |
| ------------------------ | -------------- | ----------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------- |
| Livox Mid360 Lidar + IMU | Top            | [docs](https://docs.westonrobot.com/wr_dev_kit/ugv_dev_kit_v1/ugv_devkit_mid360_extension.html#ref-ugv-devkit-livox-mid360-imu-extension) | [mid360_sensor_kit_bringup](./src/kits/top/mid360_sensor_kit_bringup/) |
| Vision                   | Top            | [docs](https://docs.westonrobot.com/wr_dev_kit/ugv_dev_kit_v1/ugv_devkit_vision_extension.html#ref-ugv-devkit-vision-extension)           | [vision_sensor_kit_bringup](./src/kits/top/vision_sensor_kit_bringup/)   |
| W200D Ultrasonic Sensors | Base           | TBD                                                                                                                                       | [w200d_sensor_kit_bringup](./src/kits/base/w200d_sensor_kit_bringup/)  |

### Robot Bases
| Base            | Documentation                                                                      |
| --------------- | ---------------------------------------------------------------------------------- |
| Ranger Mini 2.0 | [wiki](https://docs.westonrobot.com/robot_user_guide/agilex/ranger_mini_v2.0.html) |
| Scout Mini      | [wiki](https://docs.westonrobot.com/robot_user_guide/agilex/scout_mini.html)       |

**Note**: Robot bases are used for testing and development purposes only. Other robot bases should also be compatible with the provided chassis and sensor kits.

### Onboard Computer
| Operating System | Framework                                                                              |
| ---------------- | -------------------------------------------------------------------------------------- |
| Ubuntu 22.04     | [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) |

## Installation
Please refer to the [installation guide](/docs/INSTALLATION.md) for detailed instructions on setting up your development environment.

## Running the packages
Please refer to the [running guide](/docs/RUN.md) for detailed instructions on running the sample applications.
