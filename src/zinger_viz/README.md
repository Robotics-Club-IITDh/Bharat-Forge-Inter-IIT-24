# zinger_viz

Provides the [RViz](http://wiki.ros.org/rviz) config files for the Zinger robot.

## Dependencies

The configurations in this repository assume you have the following prerequisites installed on the
device on which you want to run this code. That device might be an Ubuntu machine or a physical
robot using Raspberry Pi OS.

* [zinger_description](https://github.com/pvandervelde/zinger_description) - Contains the geometric
  description of the Zinger robot for ROS to work with.

## Contents

This repository contains different folders for different parts of the RViz configuration for
zinger.

* The [launch/view_robot.launch.py](launch/view_robot.launch.py) file is used to launch RViz and
  the robot description.
* The [rviz/robot.rviz](rviz/robot.rviz) file provides the configuration for RViz.

## Usage

When launching the zinger nodes manually it is recommended to use the
[launch/view_robot.launch.py](launch/view_robot.launch.py) file as follows

    ros2 launch zinger_viz view_robot.launch.py use_sim_time:=false use_fake_hardware:=true description:=true

This will launch all the required nodes to display the robot in RViz.
