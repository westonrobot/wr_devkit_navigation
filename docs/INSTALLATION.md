# Installation Guide

## Getting this repository
This repository is intended to be used as a ROS2 colcon workspace itself (i.e. you do not need to create your own colcon workspace, though it is possible to do so).

```bash
$ mkdir -p <parent-directory>
$ cd <parent-directory>
$ git clone https://github.com/westonrobot/wr_devkit_navigation.git
```

## Install Dependencies
Each sensor kit and robot base has its own set of dependencies. Please follow the instructions below to install the necessary items for your specific hardware configuration.

### Common Dependencies
The following dependencies are common to multiple sensor kits and robot bases and should be installed regardless of the specific hardware configuration.

  * Weston Robot Platform SDK
    ```bash
    $ sudo install -m 0755 -d /etc/apt/keyrings
    $ curl -fsSL http://deb.westonrobot.net/signing.key | sudo gpg --dearmor -o /etc/apt/keyrings/weston-robot.gpg
    $ sudo chmod a+r /etc/apt/keyrings/weston-robot.gpg

    $ echo \
        "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/weston-robot.gpg] http://deb.westonrobot.net/$(lsb_release -cs) $(lsb_release -cs) main" | \
        sudo tee /etc/apt/sources.list.d/weston-robot.list > /dev/null

    $ sudo apt-get update
    $ sudo apt-get install wrp-sdk

    # Install drivers/dependencies for peripherals
    $ sudo apt-get install -y software-properties-common 
    $ sudo add-apt-repository ppa:lely/ppa
    $ sudo apt-get update
    $ sudo apt-get install liblely-coapp-dev liblely-co-tools python3-dcf-tools pkg-config

    ```
    > Please refer to [this page](https://docs.westonrobot.com/software/installation_guide.html) for more details of the installation steps.  
    > Documentation for the Weston Robot Platform SDK can be found [here](https://github.com/westonrobot/wrp_sdk).

### Robot Bases Dependencies
The following dependencies are specific to each robot base and should be installed according to the specific hardware configuration.

#### Ranger Mini 2.0 & Scout Mini
* ugv_sdk dependencies
  ```bash
  sudo apt-get install build-essential git cmake libasio-dev
  ```

### Sensor Kits Dependencies
The following dependencies are specific to each sensor kit and should be installed according to the specific hardware configuration.

#### Livox Mid360 Lidar + IMU Sensor Kit
  * Livox SDK2
    ```bash
    $ cd ~
    $ git clone https://github.com/Livox-SDK/Livox-SDK2.git
    $ cd Livox-SDK2
    $ mkdir build && cd build && cmake .. && make
    $ sudo make install
    ```
    > **Note:** You can build and install the Livox-SDK2 at your preferred places other than "~/Livox-SDK2". And you can optionally remove the "Livox-SDK2" folder after installation.

### ROS Dependencies
The following dependencies are required for building the workspace.

```bash
$ sudo apt-get install ros-humble-pcl-ros ros-humble-camera-info-manager* ros-humble-diagnostic-updater ros-humble-xacro ros-humble-pointcloud-to-laserscan
```

### ROS Driver Packages
All ROS driver packages are listed in the [navigation.repos](/navigation.repos) file. You can import the ROS driver packages by running the following command:
  ```bash
  $ vcs import --recursive src < ./navigation.repos
  ```
  > **Note:** You might not need to build all the driver packages. You can comment out the packages you do not need in the `navigation.repos` file. Refer to the table below for the ROS driver packages for each sensor kit and robot base.


  | Component                           | ROS Driver Packages Needed     |
  | ----------------------------------- | ------------------------------ |
  | Livox Mid360 Lidar + IMU Sensor Kit | livox_ros_driver2<br/>wrp_ros2 |
  | Vision Sensor Kit                   | realsense-ros<br/>usb_cam      |
  | W200d Sensor Kit                    | wrp_ros2                       |
  | Ranger Mini 2.0                     | ranger_ros2<br/>ugv_sdk        |
  | Scout Mini                          | scout_ros2<br/>ugv_sdk         |

## Build the Workspace

After installing the dependencies, you can build the workspace by running the following commands:
```bash
$ cd <parent-directory>/wr_devkit_navigation
$ source /opt/ros/humble/setup.bash
$ colcon build --symlink-install
```
