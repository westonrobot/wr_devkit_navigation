# Overview
This folder contains "bringup" packages for the different sensor/chassis kits we support for the [Weston Robot Development Kit](https://docs.westonrobot.com/wr_dev_kit/wr_dev_kit.html) (devkit). 

## Kits
* [Chassis](./chassis/)
  * Devkit chassis and frame
* [Top Sensor Kits](./top/)
  * Sensor kits designed to be mounted on the top of the devkit chassis.
  * Typically takes reference from the top of the chassis kit. (e.g top_sensor_kit_link)
* [Base Sensor Kits](./base/)
  * Sensor kits designed to be mounted on the robot base itself.
  * Typically takes reference from the robot base itself. (e.g. base_link)

## Supported Kits
* Chassis Kits
  * [UGV Devkit V1.0](./chassis/ugv_devkit_v1_bringup/)
* Top Sensor Kits
  * [Mid360 Lidar + IMU + Camera](./top/perception_sensor_kit_bringup/)
* Base Sensor Kits
  * [W200d ultrasonic](./base/w200d_sensor_kit_bringup/)

## Configuration
Each type of kit typically has 2 functions (when applicable):
1. Publishes its own transforms (and any default defined links to other kits).
   1. You can adjust the transforms using the respective config/xacro files.
2. Loads and runs any sensor drivers needed for the kit.
   1. You can adjust the sensor drivers using the respective launch/config files.

## Usage
Through the combination of multiple kits, you can potentially build a development kit with different capabilities and configurations.

**NOTE**:
Please note that while we have designed the kits to be modular, there may be some dependencies between kits. For example, the top sensor kits may depend on the chassis kits for the top_sensor_kit_link reference.
Please adjust accordingly to suit your needs/setup.