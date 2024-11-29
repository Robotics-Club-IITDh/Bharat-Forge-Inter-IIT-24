# Bharat-Forge | Inter IIT 2024

This is an official repository for Bharat-Forge Problem statement for Inter IIT Tech Meet 13.0

# Dependancies

- ROS2 Humble
- Gazebo 11
- ROS2_Control
- Colcon (if you wish to make changes)

# Setup and Usage
## If Using for the First time on device
Clone this repository into your `src` folder.
```
git clone 
```
Install the required dependancies automatically, replace `~/your_ros2_workspace` with the path to your current workspace (NOTE: Requires internet, download times may vary).
```
cd ~/your_ros2_workspace
rosdep install --from-paths src --ignore-src -r -y
```

Colcon build the workspace after installation.
```
colcon build
```

# Launching and Operation

First, Open a few terminals, each for one robot and one extra for running the World Spawner, and source ROS2 Setup and Workspace setup files
```
source /opt/ros/humble/setup.bash
source install/setup.bash
```

In the first terminal, launch the gazebo world, there are two world options, a `empty.world` and a `factory.world`, which both contain a empty map and Problem statment specified maps

Use this for launching the gazebo world, it accepts another world's file path as argument if u want to launch in different world
```
ros2 launch robot_gazebo world_launch.py
```
or launch the factory world
```
ros2 launch robot_gazebo world_launch.py world:=<Path_to_world_file> 
```
### Finally launch each robot with its own namespace, and position
```
ros2 launch slam launch.py namespace:=<Robot_name> x_pos:=<X_Coordinate> y_pos:=<Y_Coordinate>
// Sample
ros2 launch slam launch.py namespace:=robot_1 x_pos:=0.0 y_pos:=0.0
```


# How the code works (TO BE UPDATED WITH EACH PUSH)

The core of the bot uses ROS2 Humble as base and // Talk about model

The build is divided into two parts, first is the `robot_gazebo` package which holds data and launch files for launching gazebo and `slam` which holds model design, and controller scripts for the bots, the bots are identical and we can spawn any number of bots in the world, each with their custom `namespace` defined in the launching command with their position in `(x, y ,z)` format
