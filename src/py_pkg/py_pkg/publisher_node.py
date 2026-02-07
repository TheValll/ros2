#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class PublisherNode(Node):

    def __init__(self):
        super().__init__("publisher")
        self.publisher_ = self.create_publisher(String, "simple_topic", 10)
        self.timer_ = self.create_timer(0.5, self.publish_example)
        self.get_logger().info("Publisher has been started ...")

    def publish_example(self):
        msg = String()
        msg.data = "Simple publisher"
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()