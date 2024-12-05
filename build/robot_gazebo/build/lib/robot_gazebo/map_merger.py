
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
from typing import Dict, Optional
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class ScalableMapMerger(Node):
    def __init__(self):
        super().__init__('map_merger')
        
        # Declare parameters
        self.declare_parameter('robot_namespaces', ['robot_1', 'robot_2'])
        self.declare_parameter('publish_rate', 1.0)  # Hz
        
        # Get parameters
        self.robot_namespaces = self.get_parameter('robot_namespaces').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # Configure QoS profile for map data
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Initialize publisher
        self.publisher = self.create_publisher(
            OccupancyGrid,
            '/merge_map',
            map_qos
        )
        
        # Store maps
        self.maps: Dict[str, OccupancyGrid] = {}
        
        # Create subscribers for each robot
        self.subscribers = {}
        for robot_ns in self.robot_namespaces:
            topic = f'/{robot_ns}/map'
            self.subscribers[robot_ns] = self.create_subscription(
                OccupancyGrid,
                topic,
                lambda msg, ns=robot_ns: self.map_callback(msg, ns),
                map_qos
            )
            self.get_logger().info(f'Added subscriber for robot: {robot_ns}')
        
        # Create timer for publishing merged map
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_merged_map
        )
        
        self.get_logger().info('Map merger initialized')

    def map_callback(self, msg: OccupancyGrid, robot_ns: str):
        """Handle incoming map messages."""
        self.maps[robot_ns] = msg
        self.get_logger().debug(f'Received map from {robot_ns}')

    def merge_maps(self) -> Optional[OccupancyGrid]:
        """Merge available maps."""
        if not self.maps:
            return None
            
        map_list = list(self.maps.values())
        if len(map_list) == 1:
            merged_map = OccupancyGrid()
            merged_map.header = map_list[0].header
            merged_map.header.frame_id = 'map'  # Changed from 'merge_map' to 'map'
            merged_map.info = map_list[0].info
            merged_map.data = map_list[0].data
            return merged_map

        # Initialize boundaries
        min_x = float('inf')
        min_y = float('inf')
        max_x = float('-inf')
        max_y = float('-inf')
        min_resolution = float('inf')

        # Find boundaries and minimum resolution
        for map_msg in map_list:
            min_resolution = min(min_resolution, map_msg.info.resolution)
            x1 = map_msg.info.origin.position.x
            y1 = map_msg.info.origin.position.y
            x2 = x1 + (map_msg.info.width * map_msg.info.resolution)
            y2 = y1 + (map_msg.info.height * map_msg.info.resolution)
            min_x = min(min_x, x1)
            min_y = min(min_y, y1)
            max_x = max(max_x, x2)
            max_y = max(max_y, y2)

        # Create merged map
        merged_map = OccupancyGrid()
        merged_map.header.frame_id = 'map'  # Changed from 'merge_map' to 'map'
        merged_map.header.stamp = self.get_clock().now().to_msg()
        merged_map.info.resolution = min_resolution
        merged_map.info.width = int(np.ceil((max_x - min_x) / min_resolution))
        merged_map.info.height = int(np.ceil((max_y - min_y) / min_resolution))
        merged_map.info.origin.position.x = min_x
        merged_map.info.origin.position.y = min_y
        merged_map.info.origin.orientation.w = 1.0

        # Initialize merged map data
        merged_data = np.full((merged_map.info.height, merged_map.info.width), -1, dtype=np.int8)

        # Merge maps
        for map_msg in map_list:
            # Skip empty maps
            if not map_msg.data:
                continue
                
            # Convert to numpy for efficient processing
            current_map = np.array(map_msg.data, dtype=np.int8).reshape(
                map_msg.info.height, map_msg.info.width)
            
            # Calculate offsets
            x_offset = int((map_msg.info.origin.position.x - min_x) / min_resolution)
            y_offset = int((map_msg.info.origin.position.y - min_y) / min_resolution)
            
            for y in range(map_msg.info.height):
                for x in range(map_msg.info.width):
                    if current_map[y, x] == -1:  # Skip unknown cells
                        continue
                        
                    merged_x = x_offset + x
                    merged_y = y_offset + y
                    
                    if (0 <= merged_x < merged_map.info.width and 
                        0 <= merged_y < merged_map.info.height):
                        if merged_data[merged_y, merged_x] == -1:
                            merged_data[merged_y, merged_x] = current_map[y, x]

        merged_map.data = merged_data.flatten().tolist()
        return merged_map

    def publish_merged_map(self):
        """Merge and publish maps."""
        if not self.maps:
            self.get_logger().debug('No maps available for merging')
            return

        merged_map = self.merge_maps()
        if merged_map is not None:
            self.publisher.publish(merged_map)

def main(args=None):
    rclpy.init(args=args)
    node = ScalableMapMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
