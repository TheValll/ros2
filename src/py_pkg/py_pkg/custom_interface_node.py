#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_interfaces.msg import MinimalInterface


class CustomInterfaceNode(Node):

    def __init__(self):
        super().__init__("custom_interface_node")
        self.publisher_ = self.create_publisher(MinimalInterface, "custom_interface", 10)
        self.timer_ = self.create_timer(0.5, self.publish_example)
        self.get_logger().info("Custom interface node is running ...")

    def publish_example(self):
        msg = MinimalInterface()
        msg.a = 12.56
        msg.b = True
        msg.c = "Basic text"
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CustomInterfaceNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()