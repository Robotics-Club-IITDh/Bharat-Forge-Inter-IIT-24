# Bharat-Forge | Inter IIT 2024

This is an official repository for Bharat-Forge Problem statement for Inter IIT Tech Meet 13.0

# Dependancies

- ROS2 Humble
- Gazebo 11
- ROS2_Control
- Colcon (if you wish to make changes)

# Setup and Usage
TO DO
# Launching and Operation
TO DO
# How the code works (TO BE UPDATED WITH EACH PUSH)

The core of the bot uses ROS2 Humble as base and // Talk about model

The build is divided into two parts, first is the `robot_gazebo` package which holds data and launch files for launching gazebo and `slam` which holds model design, and controller scripts for the bots, the bots are identical and we can spawn any number of bots in the world, each with their custom `namespace` defined in the launching command with their position in `(x, y ,z)` format