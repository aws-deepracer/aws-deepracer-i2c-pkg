# DeepRacer I2C Package

## Overview

The DeepRacer I2C ROS package creates the *battery_node* which is part of the core AWS DeepRacer application and will be launched from the deepracer_launcher. More details about the application and the components can be found [here](https://github.com/awsdeepracer/aws-deepracer-launcher).

This node is responsible for providing the vehicle battery level information. It provides functions to read the battery level values from the I2C bus and return the corresponding level information. 

## License

The source code is released under Apache 2.0 (https://aws.amazon.com/apache-2-0/).

## Installation

### Prerequisites

The DeepRacer device comes with all the pre-requisite packages and libraries installed to run the i2c_pkg. More details about pre installed set of packages and libraries on the DeepRacer, and installing required build systems can be found in the [Getting Started](https://github.com/awsdeepracer/aws-deepracer-launcher/blob/main/getting-started.md) section of the AWS DeepRacer Opensource page.

The i2c_pkg specifically depends on the following ROS2 packages as build and execute dependencies:

* *deepracer_interfaces_pkg* - This packages contains the custom message and service type definitions used across the AWS DeepRacer core application.

## Downloading and Building

Open up a terminal on the DeepRacer device and run the following commands as root user.

1. Source the ROS2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Create a workspace directory for the package:

        mkdir -p ~/deepracer_ws
        cd ~/deepracer_ws

1. Clone the i2c_pkg on the DeepRacer device:

        git clone https://github.com/awsdeepracer/aws-deepracer-i2c-pkg.git

1. Fetch unreleased dependencies:

        cd ~/deepracer_ws/aws-deepracer-i2c-pkg
        rosws update

1. Resolve the i2c_pkg dependencies:

        cd ~/deepracer_ws/aws-deepracer-i2c-pkg && rosdep install -i --from-path . --rosdistro foxy -y

1. Build the i2c_pkg and deepracer_interfaces_pkg:

        cd ~/deepracer_ws/aws-deepracer-i2c-pkg && colcon build --packages-select i2c_pkg deepracer_interfaces_pkg

## Usage

Although the *battery_node* is built to work with the AWS DeepRacer application, it can be run independently for development/testing/debugging purposes.

### Run the node

To launch the built battery_node as root user on the DeepRacer device open up another terminal on the DeepRacer device and run the following commands as root user:

1. Source the ROS2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Source the setup script for the installed packages:

        source ~/deepracer_ws/aws-deepracer-i2c-pkg/install/setup.bash

1. Launch the battery_node using the launch script:

        ros2 launch i2c_pkg i2c_pkg_launch.py

## Launch Files

The  i2c_pkg_launch.py is also included in this package that gives an example of how to launch the deepracer_navigation_node.

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

## Node Details

### battery_node

#### Services

| Service Name | Service Type | Description |
| ---------- | ------------ | ----------- |
| battery_level|BatteryLevelSrv|A service that is called to get the current vehicle battery level information in the range of [0 to 11].|

## Resources

* AWS DeepRacer Opensource getting started: [https://github.com/awsdeepracer/aws-deepracer-launcher/blob/main/getting-started.md](https://github.com/awsdeepracer/aws-deepracer-launcher/blob/main/getting-started.md)
