# Weston Robot Devkit Bringup

## Overview
This package contains sample launch and config files to launch Weston Robot's Devkit and sample applications.

This package is meant to be a starting point for users to build upon and customize to their needs, and shows how to compose the various kits of the Devkit platform.

## Launch files
* [wr_devkit_platform.launch.py](./launch/wr_devkit_platform.launch.py)
  * Sample launch file to launch the Weston Robot Devkit platform with the following configuration
    * Robot base: Ranger mini v2
    * Chassis: ugv_devkit_v1
    * Base sensor kit: w200d_sensor_kit
    * Top sensor kit: mid360_sensor_kit
  
    | Argument      | Description                  | Default Value |
    | ------------- | ---------------------------- | ------------- |
    | use_namespace | Whether to apply a namespace | False         |
    | namespace     | Top-level namespace          | ""            |
    | use_sim_time  | Whether to use sim time      | False         |