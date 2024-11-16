# Bharat-Forge-Inter-IIT-24
This is an official repository for the Bharat Forge problem statement for Inter IIT Tech Meet 13.0, 2024-25.


# Dependancies
- ROS2 Humble
- Gazebo 11
- ROS2_Control
- Colcon (if you wish to debug)
- Might Require a version of numpy

# Setup and Usage
First, get to the source folder and initialize setup file for each terminal you are gonna use

### Terminal 1
launching the Swerve Controller
```ros2 launch zinger_swerve_controller swerve_controller.launch.py use_sim_time:=true```

### Terminal 2
launching the Controller Manager
```ros2 launch zinger_description controller_manager.launch.py```

### Terminal 2
launching the bot itself
```ros2 launch zinger_ignition ignition.launch.py rviz:=true world:=empty_world```

# Current issues
THE CONTROLLER DOESN'T F****NG WORK
  atleast the robot spawns... for now

- Might give a building error for CMakeList
