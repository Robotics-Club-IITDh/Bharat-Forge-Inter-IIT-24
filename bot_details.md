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

# **Map data**

The `nav_msgs/OccupancyGrid` message in ROS 2 represents a 2D grid map of the environment. Each cell in the grid contains information about its occupancy status, which is critical for SLAM (Simultaneous Localization and Mapping) and navigation tasks.

---

## **Message Structure**

### **1. Header**
Contains metadata about the map:
- **`stamp`**: Timestamp indicating when the map was generated or last updated.
- **`frame_id`**: Coordinate frame in which the map is defined (e.g., `"map"`).

---

### **2. Info (`MapMetaData`)**
Describes the structure and resolution of the map grid.

| Field             | Description                                                                                     | Example Value           |
|--------------------|-----------------------------------------------------------------------------------------------|-------------------------|
| **`map_load_time`** | Time when the map was loaded into memory.                                                      | `1698765400`            |
| **`resolution`**   | Size of each grid cell in meters.                                                              | `0.05` (5 cm)           |
| **`width`**        | Number of cells along the X-axis.                                                              | `200` (10 meters wide)  |
| **`height`**       | Number of cells along the Y-axis.                                                              | `150` (7.5 meters tall) |
| **`origin`**       | Pose of the bottom-left corner of the map in the coordinate frame.                             |                         |
| - **Position**     | (x, y, z) coordinates of the origin in meters.                                                 | `x: -5.0, y: -3.75, z: 0.0` |
| - **Orientation**  | Quaternion representing the rotation of the map.                                               | `x: 0.0, y: 0.0, z: 0.0, w: 1.0` |

---

### **3. Data**
The `data` field contains a flattened 1D array representing the 2D grid map.

| Value | Meaning            |
|-------|--------------------|
| `-1`  | Unknown (e.g., unexplored areas) |
| `0`   | Free (e.g., traversable areas)   |
| `100` | Occupied (e.g., obstacles)       |

- The array length is `width × height`, where each value corresponds to a grid cell.
- Example:
  - For a 3×2 grid:
    ```
    [  0,  0, 100 ]
    [ -1,  0,   0 ]
    ```
  - The `data` array is:
    ```
    [ 0, 0, 100, -1, 0, 0 ]
    ```

---

## **Sample Message**
```yaml
header:
  stamp:
    sec: 1698765432
    nanosec: 123456789
  frame_id: "map"
info:
  map_load_time:
    sec: 1698765400
    nanosec: 0
  resolution: 0.05  # 5 cm per grid cell
  width: 200        # 10 meters wide (200 cells × 0.05 meters)
  height: 150       # 7.5 meters tall (150 cells × 0.05 meters)
  origin:
    position:
      x: -5.0
      y: -3.75
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
data:
  [-1, -1, -1, 0, 0, 100, -1, 0, 100, 100, ...]  # Flattened 1D array



# Robot Dimensions

- Size="0.4 0.3 0.1" (meter)
- Wheel Radius = 0.05 m
- Wheel Width = 0.02 m
- Bot Mass = 3.1 kg
- LiDAR Offset from plane = 0.18 m 
