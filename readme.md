# Bharat-Forge | Inter IIT 2024

This is an official repository for Bharat-Forge Problem statement for Inter IIT Tech Meet 13.0

# Dependancies

- Ubuntu 22.04
- ROS2 Humble ----- ([ROS2 Humble Official Documentation](https://docs.ros.org/en/humble/Installation.html))
- Gazebo 11  ----- ([Gazebo 11 Official Documentation](https://classic.gazebosim.org/tutorials?tut=install_ubuntu))
- Colcon
- Numpy
- PyTorch

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

### Giving Targets to the model

Targets to a model are given by a node names set_target, open up another terminal and run this command to give the robot a target in form of (x,y,z)
```
ros2 run slam set_target
```

# Monitoring output

launch RViz2 to monitor map creation
```
rviz2
```

Once RViz2 opens, click the add button and add a map data, select to topic as <<TOPIC_GOES_HERE!!!>>
