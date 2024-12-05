#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraProcessor(Node):
    def __init__(self):
        super().__init__('camera_processor')
        
        # Declare and get namespace parameter
        self.declare_parameter('namespace', 'robot_1')
        self.namespace = self.get_parameter('namespace').value
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Create subscription to camera topic
        self.subscription = self.create_subscription(
            Image,
            f'/{self.namespace}_camera/image_raw',
            self.image_callback,
            10
        )
        self.subscription
        
        self.get_logger().info(f'Camera processor initialized for {self.namespace}')
        
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
        ########################## CV STUFF HERE ##########################



        ###################################################################
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    processor = CameraProcessor()
    
    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()