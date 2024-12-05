import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from std_msgs.msg import String
import math

class MasterController(Node):
    def __init__(self):
        # First, initialize the node
        super().__init__('master_controller')
        
        # Then declare the parameter before trying to access it
        self.declare_parameter('robot_names', ['robot_1', 'robot_2'])  # Provide default values
        
        # Now we can safely get the parameter
        robot_names = self.get_parameter('robot_names').get_parameter_value().string_array_value
        
        # Initialize the rest of the node
        self.robot_positions = {name: None for name in robot_names}
        self.target_position = None
        
        # Publisher for /target_return topic
        self.target_return_publisher = self.create_publisher(String, '/target_return', 10)
        
        # Subscribe to /target topic for receiving target position
        self.create_subscription(
            Point,
            '/target',
            self.update_target_position,
            10
        )
        
        # Subscribe to robot odometry topics
        for name in robot_names:
            self.create_subscription(
                Odometry,
                f'/{name}/odom',
                lambda msg, name=name: self.update_position(name, msg),
                10
            )
        
        # Fixed the method call
        self.get_logger().info("Master Controller Started")

    def update_position(self, name, msg):
        """Callback to update robot positions from Odometry messages."""
        self.robot_positions[name] = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        # self.get_logger().info(f'Updated position for {name}: {self.robot_positions[name]}')

    def update_target_position(self, msg):
        """Callback to update the target position."""
        self.target_position = (msg.x, msg.y)
        self.get_logger().info(f'Received new target position: {self.target_position}')
        # Find and publish the nearest robot
        self.find_and_publish_nearest_robot()

    def find_and_publish_nearest_robot(self):
        """Find the nearest robot to the target and publish its name."""
        if self.target_position is None:
            self.get_logger().warning("Target position not set.")
            return
            
        nearest_robot = None
        nearest_distance = float('inf')
        
        for name, position in self.robot_positions.items():
            if position is not None:  # Ensure position is available
                distance = math.sqrt(
                    (position[0] - self.target_position[0]) ** 2 +
                    (position[1] - self.target_position[1]) ** 2
                )
                if distance < nearest_distance:
                    nearest_robot = name
                    nearest_distance = distance
        
        if nearest_robot:
            ############################## ADD STUFF ON WHAT TO DO WITH NEAREST ROBOT HERE ##########################################
            self.get_logger().info(
                f'Nearest robot to {self.target_position}: {nearest_robot} at distance {nearest_distance:.2f}'
            )
            self.target_return_publisher.publish(String(data=nearest_robot))
        else:
            self.get_logger().info("No robot positions available yet.")

def main():
    rclpy.init()
    finder = MasterController()
    
    try:
        rclpy.spin(finder)  # Process messages until shutdown
    except KeyboardInterrupt:
        pass
        
    finder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
