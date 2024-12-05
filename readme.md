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
ros2 launch robot_gazebo world_launch.py world:=<Path_to_world_file> robot_names:='["robot_1", "robot_2"]'
// Add the same robot names as you wish to spawn, otherwise resulting in unintended behaviour
```
### Finally launch each robot with its own namespace, and position
```
ros2 launch slam launch.py namespace:=<Robot_name> x_pos:=<X_Coordinate> y_pos:=<Y_Coordinate>
// Sample
ros2 launch slam launch.py namespace:=robot_1 x_pos:=0.0 y_pos:=0.0
```

**######### Make sure to launch all robots from same coordinates for accurate map merging ########**

### Giving Targets to the model

Targets to a model are given by a node `Master_Controller`, we use `geometry_msgs/Point` to set Target to the system, the master controller will then calculate the nearest robot to the target and shall assign task to it, targets can be given by publishing a point message on `/target` topic, and you shall get the name of the assigned robot back on topic `/target_return`
```
ros2 topic pub --once /target geometry_msgs/msg/Point "{x: 3.0, y: 0.0, z: 0.0}"
```

This will set target to (3,0,0)

# Monitoring output

launch RViz2 to monitor map creation
```
rviz2
```

Once RViz2 opens, click the add button and add a map data, select to topic as /merged_map

# Scaling
The system is designed with scaling in mind, to change the number of robots for testing the system, do the following

- **Tune the Map Merger and Master_Controllers for the number of bots**
  Go into `world_launch.py`, go to `line 28` and update the names of the bots you wish to use while spawning them

