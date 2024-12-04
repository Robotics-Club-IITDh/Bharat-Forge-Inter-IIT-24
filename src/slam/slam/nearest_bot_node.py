import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

class NearestRobotFinder(Node):
    def __init__(self, robot_names):
        super().__init__('nearest_robot_finder')
        
        self.robot_positions = {name: None for name in robot_names}  # Store robot positions
        
        # Subscribe to robot odometry topics
        for name in robot_names:
            self.create_subscription(
                Odometry,
                f'/{name}/odom',  # Assume robots publish odometry on /robot_name/odom
                lambda msg, name=name: self.update_position(name, msg),
                10
            )
    
    def update_position(self, name, msg):
        """Callback to update robot positions from Odometry messages."""
        self.robot_positions[name] = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        self.get_logger().info(f'Updated position for {name}: {self.robot_positions[name]}')
    
    def find_nearest_robot(self, target_position):
        """Find the nearest robot to a given position."""
        nearest_robot = None
        nearest_distance = float('inf')
        
        for name, position in self.robot_positions.items():
            if position is not None:  # Ensure position is available
                distance = math.sqrt(
                    (position[0] - target_position[0]) ** 2 +
                    (position[1] - target_position[1]) ** 2
                )
                if distance < nearest_distance:
                    nearest_robot = name
                    nearest_distance = distance
        
        return nearest_robot, nearest_distance


def main():
    rclpy.init()
    
    # Define robot names
    robot_names = ['robot_1', 'robot_2']
    
    finder = NearestRobotFinder(robot_names)
    
    # Define a target position
    target_position = (5.0, 5.0)  # Example target position
    
    try:
        while rclpy.ok():
            rclpy.spin_once(finder, timeout_sec=0.1)  # Process incoming messages
            
            nearest_robot, distance = finder.find_nearest_robot(target_position)
            if nearest_robot:
                finder.get_logger().info(
                    f'Nearest robot to {target_position}: {nearest_robot} at distance {distance:.2f}'
                )
            else:
                finder.get_logger().info("Waiting for robot positions...")
    
    except KeyboardInterrupt:
        pass
    
    finder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
