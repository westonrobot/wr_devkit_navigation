# WR Devkit Navigation

![default workflow](https://github.com/westonrobot/wr_devkit_navigation/actions/workflows/default.yml/badge.svg?branch=main)

This repository provides a reference setup for using Nav2 stack to do 2D navigation with the mobile robot development kit from Weston Robot.

## Requirements

The following hardware configurations are supported: 

* WR Devkit V1.0
  * With Livox Mid-360 Lidar

* AgileX Base
  * Ranger Mini 2.0
  * More to come

The onboard computer with the devkit should have been configured with the following software environment:

* Ubuntu 22.04 
* ROS Humble

## Installation

* Install the Weston Robot Platform SDK (wrp-sdk)

    ```bash
    $ sudo install -m 0755 -d /etc/apt/keyrings
    $ curl -fsSL http://deb.westonrobot.net/signing.key | sudo gpg --dearmor -o /etc/apt/keyrings/weston-robot.gpg
    $ sudo chmod a+r /etc/apt/keyrings/weston-robot.gpg

    $ echo \
        "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/weston-robot.gpg] http://deb.westonrobot.net/$(lsb_release -cs) $(lsb_release -cs) main" | \
        sudo tee /etc/apt/sources.list.d/weston-robot.list > /dev/null
    $ sudo apt-get update

    $ sudo apt-get install wrp-sdk

    # Install drivers for peripherals
    $ sudo add-apt-repository ppa:lely/ppa
    $ sudo apt-get update

    $ sudo apt-get install liblely-coapp-dev liblely-co-tools python3-dcf-tools pkg-config
    ```

    Please refer to [this page](https://docs.westonrobot.net/software/installation_guide.html) for more details of the installation steps.

* Install ros-dev-tools **(Make sure you have ros2 installed first)**
    ```bash
    $ sudo apt install ros-dev-tools
    ```

* Install ugv_sdk dependencies
    ```bash
    sudo apt-get install build-essential git cmake libasio-dev
    ```

* Install Livox SDK2 (if you have the devkit variant with the Livox Mid-360 Lidar)

    ```bash
    $ cd ~
    $ git clone https://github.com/Livox-SDK/Livox-SDK2.git
    $ cd Livox-SDK2
    $ mkdir build && cd build && cmake .. && make
    $ sudo make install
    ```

    Note: you can build and install the Livox-SDK2 at your preferred places other than "~/Livox-SDK2". And you can optionally remove the "Livox-SDK2" folder after installation.

* Import the ROS packages into the workspace and build

    ```bash
    $ cd <your-workspace>/wr_devkit_navigation
    # Clone dependencies
    $ vcs import --recursive src < ./navigation.repos

    $ source /opt/ros/humble/setup.bash
    $ colcon build --symlink-install
    ```

    The build process should finish without any errors.

## Running the packages
Remember to source the ROS Workspace first and optionally set ROS_DOMAIN_ID

* Bringup Robot
    ```bash
    $ sudo ip link set can0 up type can bitrate 500000
    $ sudo ip link set can0 txqueuelen 1000
    $ ros2 launch wr_devkit_robot_bringup wr_devkit_robot_bringup.launch.py
    ```

* 2D SLAM (Bringup robot first)
    ```bash
    $ ros2 launch wr_devkit_nav_bringup wr_devkit_cartographer.launch.py 

    # Control via RC or teleop
    $ ros2 run teleop_twist_keyboard teleop_twist_keyboard.py

    # Save map
    $ ros2 run nav2_map_server map_saver_cli -f <your_map_name>
    ```

* Sample Nav2 (Bringup robot first)
    ```bash
    $ ros2 launch wr_devkit_nav_bringup wr_devkit_nav_bringup.launch.py map:=<your_map_yaml>
    ```

    * You may want to hardcode the absolute path of the pgm file in the map yaml file
    * Map to odom frame will not be published until you provide an initial pose estimate 
    * You can run rviz2 on another pc via
      ```bash
      ros2 launch nav2_bringup rviz_launch.py
      ```

## Additional Notes

* Sample launch files for ultrasonic sensors (not integrated with Nav2)
  
    * Ranger Mini 2.0

    ```bash
    $ ros2 launch wr_devkit_robot_bringup wr_devkit_ultrasonic_bringup.launch.py
    ```

* Depending on the specific hardware configurations, you may need to modify the sample launch files and configuration files to adapt to your setup.  
   Take note of the below in particluar:
   1. IP addresses of the lidar and the data ports it uses. ([config file](./src/wr_devkit_robot_bringup/config/MID360_config.json))
   2. Device path of the IMU in the [launch file](./src/wr_devkit_robot_bringup/launch/wr_devkit_sensor_bringup.launch.py).