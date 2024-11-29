# Simple Diff_Drive_Bot

Listing of data for Each bot, bots operate independantly of each other and are identical

# Data input for AI Model 

### TF Transform messages

Relation Data:
- `Time Stamp`, with respect to simuation time
- `frame_id`, or the parent frame, aka the frame which is origin of data
- `child_frame`, the frame to which vector points to


Pose:
- Vector going from parent to child frame == (x, y ,z) == Position Offset
- Orientation of child wrt parent == Quaternion format == (x,y,z,w) == Rotation Offset

JSON Sample
```
{
  "header": {
    "stamp": {"sec": 1636788302, "nanosec": 123456789},
    "frame_id": "odom"
  },
  "child_frame_id": "base_link",
  "transform": {
    "translation": {"x": 1.0, "y": 2.0, "z": 0.0},
    "rotation": {"x": 0.0, "y": 0.0, "z": 0.707, "w": 0.707}
  }
}
```

### Odometry messages 

Relation Data:
- `Time Stamp`, with respect to simuation time
- `frame_id`, or the parent frame, aka the frame which is origin of data
- `child_frame`, the frame to which vector points to

Pose:
- Vector going from parent to child frame == (x, y ,z) == Position Offset
- Orientation of child wrt parent == Quaternion format == (x,y,z,w) == Rotation Offset

Velocities
- Linear Velocity `Vector` (x, y, z) 
- Angular Velocity `Vector` (x, y, z)

JSON Sample
```
{
  "header": {
    "stamp": {"sec": 1636788302, "nanosec": 123456789},
    "frame_id": "odom"
  },
  "child_frame_id": "base_link",
  "pose": {
    "pose": {
      "position": {"x": 1.0, "y": 0.5, "z": 0.0},
      "orientation": {"x": 0.0, "y": 0.0, "z": 0.707, "w": 0.707}
    },
    "covariance": [0.1, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, ...]
  },
  "twist": {
    "twist": {
      "linear": {"x": 0.5, "y": 0.0, "z": 0.0},
      "angular": {"x": 0.0, "y": 0.0, "z": 0.1}
    },
    "covariance": [0.1, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, ...]
  }
}
```

# Data output for MI Model

Assuming model only wants to drive the bot, it will require these components

### Twist Velocities

Velocities
- Linear Velocity `Vector` (x, y, z) 
- Angular Velocity `Vector` (x, y, z)

```
"twist": {
      "linear": {"x": 0.5, "y": 0.0, "z": 0.0},
      "angular": {"x": 0.0, "y": 0.0, "z": 0.1}
    }
```


# Robot Dimensions

- Size="0.4 0.3 0.1" (meter)
- Wheel Radius = 0.05 m
- Wheel Width = 0.02 m
- Bot Mass = 3.1 kg
- LiDAR Offset from plane = 0.18 m 