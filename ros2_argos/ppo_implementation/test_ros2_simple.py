#!/usr/bin/env python3
"""
Simple ROS2 test to verify the Docker container can run ROS2 nodes
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import numpy as np

class SimpleTestNode(Node):
    def __init__(self):
        super().__init__('simple_test_node')

        # Create publisher
        self.publisher = self.create_publisher(Float32MultiArray, '/test/observation', 10)

        # Create timer to publish fake observations
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.counter = 0
        self.get_logger().info('Simple ROS2 test node started!')
        self.get_logger().info('Publishing to /test/observation every second')

    def timer_callback(self):
        # Create fake observation (15 values like PPO expects)
        msg = Float32MultiArray()
        msg.data = np.random.rand(15).tolist()

        self.publisher.publish(msg)
        self.counter += 1
        self.get_logger().info(f'Published observation #{self.counter}')

def main(args=None):
    print("Starting simple ROS2 test...")
    print("This verifies ROS2 is working in the container.")
    print("You should see messages being published.")
    print("Press Ctrl+C to stop.\n")

    rclpy.init(args=args)
    node = SimpleTestNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()