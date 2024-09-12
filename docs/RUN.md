# Running Guide
This guide will walk you through running the sample applications included in the Weston Robot Development Kit.

This repository includes 2 sample mapping setups:
 1. [2D SLAM](#cartographer-2d-slam-bringup) with Cartographer
 2. [2D VSLAM](#rtabmap-2d-vslam-bringup) with RTABMap

Both setups are integrated with Nav2 navigation, enabling autonomous waypoint navigation for your robot.

Sample launch files can be found in the [wr_devkit_bringup](/src/wr_devkit_bringup/) package. They are meant to be used as a starting point for your own development and can be customized to your needs.

> A more detailed guide can be found on our [documentation site](https://docs.westonrobot.com/wr_dev_kit/ugv_dev_kit_v1/ugv_devkit_ros2_navigation_sample_setup_guide.html#ugv-devkit-ros2-navigation-sample-setup-guide)

Below is the typical workflow to bring up the robot and run some sample applications.

## Setup hardware
> **NOTE:** This example assumes a ranger mini v2 robot base
* Bringup CAN Bus for Robot bases
  ```bash
  $ sudo ip link set can0 up type can bitrate 500000
  $ sudo ip link set can0 txqueuelen 1000
  ```

## Platform Bringup
  > **Remember to source the ROS Workspace first**

  The platform bringup launch file initializes the robot hardware and required ROS2 nodes. It is a prerequisite for running the other applications.
  ```bash
    $ ros2 launch wr_devkit_bringup wr_devkit_platform.launch.py robot_model:=ranger_mini_v2
  ```

## Mapping
### Cartographer 2D SLAM Bringup
  * Start the Cartographer SLAM node
    ```bash
      $ ros2 launch wr_devkit_bringup wr_devkit_cartographer.launch.py
    ```
  * Control the robot using the teleop_twist_keyboard or other methods around the environment to create a map
    ```bash
      $ ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```
  * Save the map
    ```bash
      $ cd <path-to-workspace>/wr_devkit_navigation/src/wr_devkit_bringup/maps
      $ ros2 run nav2_map_server map_saver_cli -f <your_map_name>
    ```

### RTABMAP 2D VSLAM Bringup
  > **Note:** RTAB-Map requires an RGB-D camera (e.g., Intel RealSense D435) for operation.
  * Start the RTAB-Map node
    ```bash
      $ ros2 launch wr_devkit_bringup wr_devkit_rtabmap.launch.py
    ```
  * Control the robot using the teleop_twist_keyboard or other methods around the environment to create a map
    ```bash
      $ ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```
  * Save the map
    ```
    After building the map, you can exit the program directly. The map will automatically save as *rtabmap.db* in the main directory under *.ros*.
    ```

## Nav2 Navigation
### If you are using Cartographer SLAM:
  * Launch Nav2 with your map
    ```bash
      $ ros2 launch wr_devkit_bringup wr_devkit_nav2.launch.py robot_param:=nav2_ranger_mini.param.yaml map:=<your_map_yaml>
    ```
  > **Note:** Replace `<your_map_yaml>` with the path to the map file you saved earlier.
### If you are using RTAB-Map:
  * Launch RTAB-Map node in localization mode
    ```bash
    $ ros2 launch wr_devkit_bringup wr_devkit_rtabmap.launch.py localization:=true
    ```
  * Launch Nav2
    ```bash
    $ ros2 launch wr_devkit_bringup wr_devkit_nav2_rtab.launch.py robot_param:=nav2_scout_mini_rtab.param.yaml
    ```

## Notes
* The sample applications (Nav2/SLAM/vSLAM) are designed to be ran separately and `should not` be ran at the same time.
* For more detailed instructions on running the different launch files, refer to the README file in [wr_devkit_bringup](/src/wr_devkit_bringup/).