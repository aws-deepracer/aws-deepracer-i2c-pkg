# AWS DeepRacer I2C package

## Overview

The AWS DeepRacer I2C ROS package creates the `battery_node`, which is part of the core AWS DeepRacer application and launches from the `deepracer_launcher`. For more information about the application and the components, see the [`aws-deepracer-launcher` repository](https://github.com/aws-deepracer/aws-deepracer-launcher)

This node is responsible for providing the vehicle battery level information. It provides functions to read the battery level values from the I2C bus and return the corresponding level information. 

## License

The source code is released under Apache 2.0 (https://aws.amazon.com/apache-2-0/).

## Installation

Follow these steps to install the AWS DeepRacer I2C package.

### Prerequisites

The AWS DeepRacer device comes with all the prerequisite packages and libraries installed to run the `i2c_pkg`. For more information about the pre-installed set of packages and libraries on the AWS DeepRacer and about installing the required build systems, see [Getting started with AWS DeepRacer OpenSource](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md).

The `i2c_pkg` specifically depends on the following ROS 2 packages as build and execute dependencies:

* `deepracer_interfaces_pkg`: This package contains the custom message and service-type definitions used across the AWS DeepRacer core application.

## Downloading and building

Open a terminal on the AWS DeepRacer device and run the following commands as the root user.

1. Switch to the root user before you source the ROS 2 installation:

        sudo su

1. Source the ROS 2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Create a workspace directory for the package:

        mkdir -p ~/deepracer_ws
        cd ~/deepracer_ws

1. Clone the i2c_pkg on the DeepRacer device:

        git clone https://github.com/aws-deepracer/aws-deepracer-i2c-pkg.git

1. Fetch unreleased dependencies:

        cd ~/deepracer_ws/aws-deepracer-i2c-pkg
        rosws update

1. Resolve the `i2c_pkg` dependencies:

        cd ~/deepracer_ws/aws-deepracer-i2c-pkg && rosdep install -i --from-path . --rosdistro foxy -y

1. Build the `i2c_pkg` and `deepracer_interfaces_pkg`:

        cd ~/deepracer_ws/aws-deepracer-i2c-pkg && colcon build --packages-select i2c_pkg deepracer_interfaces_pkg

## Usage

Although the `battery_node` is built to work with the AWS DeepRacer application, you can run it independently for development, testing, and debugging purposes.

### Run the node

To launch the built `battery_node` as the root user on the AWS DeepRacer device, open another terminal on the AWS DeepRacer device and run the following commands as the root user:

1. Switch to the root user before you source the ROS 2 installation:

        sudo su

1. Source the ROS 2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Source the setup script for the installed packages:

        source ~/deepracer_ws/aws-deepracer-i2c-pkg/install/setup.bash

1. Launch the `battery_node` using the launch script:

        ros2 launch i2c_pkg i2c_pkg_launch.py

## Launch files

The `i2c_pkg_launch.py` included in this package provides an example demonstrating how to launch the `deepracer_navigation_node`.

    from launch import LaunchDescription
    from launch_ros.actions import Node

    def generate_launch_description():
        return LaunchDescription([
            Node(
                package='i2c_pkg',
                namespace='i2c_pkg',
                executable='battery_node',
                name='battery_node'
            )
        ])

## Node details

### `battery_node`

#### Services

| Service name | Service type | Description |
| ---------- | ------------ | ----------- |
| `battery_level`|`BatteryLevelSrv`|A service that is called to get the current vehicle battery level information in the range of [0 to 11].|

## Resources

* [Getting started with AWS DeepRacer OpenSource](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md)
