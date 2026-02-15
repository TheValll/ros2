#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class ParametersNode(Node):

    def __init__(self):
        super().__init__("parameters_node")
        self.declare_parameter("message", "Simple publisher")
        self.publisher_ = self.create_publisher(String, "simple_topic", 10)
        self.timer_ = self.create_timer(0.5, self.publish_example)
        self.get_logger().info("Publisher with parameters has been started ...")

    def publish_example(self):
        msg = String()
        msg.data = self.get_parameter("message").value
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ParametersNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()