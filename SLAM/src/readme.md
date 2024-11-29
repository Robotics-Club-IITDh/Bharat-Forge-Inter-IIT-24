# Multiple Spawning Robots in ROS2


Use this for launching the gazebo world, it accepts another world's file path as argument if u want to launch in diffrent world
```
ros2 launch robot_gazebo world_launch.py
```

Use this to spawning each new robot individually
```
ros2 launch slam launch.py namespace:=robot_1 x_pos:=0.0 y_pos:=0.0
ros2 launch slam launch.py namespace:=robot_2 x_pos:=2.0 y_pos:=0.0
```

define namespace, x_pos, y_pos, z_pos, defaults to "robot_1", (0,0,0)

supports multiple spawning
