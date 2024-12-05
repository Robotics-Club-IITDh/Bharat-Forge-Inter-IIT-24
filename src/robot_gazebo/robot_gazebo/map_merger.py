import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np


def merge_maps(map_list):
    """
    Merge a list of OccupancyGrid maps into a single map.
    """
    if not map_list:
        return None

    # Initialize merged map properties from the first map
    merged_map = OccupancyGrid()
    merged_map.header = map_list[0].header
    merged_map.header.frame_id = 'merged_map'

    # Calculate the boundaries of the merged map
    min_x = float('inf')
    min_y = float('inf')
    max_x = float('-inf')
    max_y = float('-inf')

    for m in map_list:
        origin_x = m.info.origin.position.x
        origin_y = m.info.origin.position.y
        max_x = max(max_x, origin_x + (m.info.width * m.info.resolution))
        max_y = max(max_y, origin_y + (m.info.height * m.info.resolution))
        min_x = min(min_x, origin_x)
        min_y = min(min_y, origin_y)

    # Determine merged map dimensions
    resolution = min([m.info.resolution for m in map_list])
    merged_map.info.resolution = resolution
    merged_map.info.width = int(np.ceil((max_x - min_x) / resolution))
    merged_map.info.height = int(np.ceil((max_y - min_y) / resolution))
    merged_map.info.origin.position.x = min_x
    merged_map.info.origin.position.y = min_y
    merged_map.info.origin.orientation.w = 1.0

    # Initialize the merged map data with -1 (unknown)
    merged_map.data = [-1] * (merged_map.info.width * merged_map.info.height)

    # Merge all maps
    for m in map_list:
        for y in range(m.info.height):
            for x in range(m.info.width):
                i = x + y * m.info.width
                if m.data[i] == -1:
                    continue  # Ignore unknown cells

                merged_x = int(
                    np.floor((m.info.origin.position.x + x * m.info.resolution - min_x) / resolution))
                merged_y = int(
                    np.floor((m.info.origin.position.y + y * m.info.resolution - min_y) / resolution))
                merged_i = merged_x + merged_y * merged_map.info.width

                # Only overwrite if the merged map cell is unknown
                if merged_map.data[merged_i] == -1:
                    merged_map.data[merged_i] = m.data[i]

    return merged_map


class MergeMapNode(Node):
    def __init__(self):
        super().__init__('merge_map_node')

        # Declare parameters
        self.declare_parameter('robot_names', ['robot1', 'robot2'])

        # Get robot names from parameters
        robot_names = self.get_parameter('robot_names').get_parameter_value().string_array_value
        self.map_topics = [f'/{name}/map' for name in robot_names]

        # Publisher for merged map
        self.publisher = self.create_publisher(OccupancyGrid, '/merged_map', 10)

        # Initialize subscriptions and map storage
        self.map_data = {topic: None for topic in self.map_topics}
        self.subscriptions = [
            self.create_subscription(OccupancyGrid, topic, self.map_callback_factory(topic), 10)
            for topic in self.map_topics
        ]

        # Timer to publish merged maps periodically
        self.timer = self.create_timer(1.0, self.publish_merged_map)

    def map_callback_factory(self, topic):
        """
        Generate a callback function for each map topic.
        """
        def callback(msg):
            self.map_data[topic] = msg
        return callback

    def publish_merged_map(self):
        """
        Merge all maps and publish the result.
        """
        map_list = [map_msg for map_msg in self.map_data.values() if map_msg is not None]
        if len(map_list) < len(self.map_topics):
            self.get_logger().warn('Not all maps are available yet.')
            return

        merged_map = merge_maps(map_list)
        if merged_map:
            merged_map.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(merged_map)


def main(args=None):
    rclpy.init(args=args)
    node = MergeMapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
