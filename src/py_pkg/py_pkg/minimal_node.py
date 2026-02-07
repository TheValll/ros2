#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class MininalNode(Node):

    def __init__(self):
        super().__init__("minimal_node")
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("Minimal node is running ...")

def main(args=None):
    rclpy.init(args=args)
    node = MininalNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()