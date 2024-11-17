# Bharat-Forge-Inter-IIT-24
This is an official repository for the Bharat Forge problem statement for Inter IIT Tech Meet 13.0, 2024-25.
## This readme is ONLY for slam pkg

# SLAM Robot with Navigation Abilities

This is a package of a simple Diff-Drive robot that features mapping abilities and navigations abilities, currently mapping has to be done manually using a controller like teleops_keyboard, once mapping is done, one can use Nav2 to have the robot autonomously navigate throught the System.

# Setup

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

# Running 

Open Three terminals for running all the required commands for launching and visualizing the robot, first setup all three terminals for running ros2 in your workspace
```
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### Now run these commands
Terminal 1: For running Teleops Keyboard
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Terminal 2: For Launching the Gazebo world, and spawning model with slam and nav2 pkgs
```
ros2 launch slam launch.py
```

Terminal 3: For Launching RViz2 for visualizing data
```
ros2 run rviz2 rviz2 -d install/slam/share/slam/config/slam.rviz
```

Using the Teleops Keyboard, run around and create a map, once the map is successfully created, one can use the 2D Target Pose button on top bar of RViz2 and click on point on map to have the Bot run to it...

# Working of the code

The bot is designed in such a way that it can be operated both by keyboard and autonomous navigation using Nav2, it features a `diff_drive_controller`

As of now, the teleops_keyboard and Nav2 both publish `geometry_msgs/Twist` type messages on `/cmd_vel` topic, which are forwarded to `/diff_drive_controller/cmd_ved_unstamped` topic by the `controller` node present in `operatorNode.py`

The LiDAR present in URDF publishes `sensor_msgs/LaserScan` type messages to `/laser_controller/out` topic which is remmapped to `/scan` during the launch of the Node. SLAM toolbox subscribes to `/scan` and `diff_drive_controller/odom` topics to operate and design a map that is visualised in RViz2, and publishes the map to `/map` topic 

Nav2 takes input from `/map` and depending on target, spits out `geometry_msgs/Twist` commands to `/cmd_vel` topics which drives the robot
