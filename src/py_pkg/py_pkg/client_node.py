#!/usr/bin/env python3
# Based on the AddTwoInts interface
import rclpy
from functools import partial
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class ClientNode(Node):

    def __init__(self):
        super().__init__("client_node")
        self.client_ = self.create_client(AddTwoInts, "add_two_ints")
        self.get_logger().info("Client node is running ...")

    def call_service(self, a, b):
        while not self.client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for the server ...")

        req = AddTwoInts.Request()
        req.a = a
        req.b = b

        future = self.client_.call_async(req)
        future.add_done_callback(partial(self.callback_response, req))

    def callback_response(self, req, future):
        res = future.result()
        self.get_logger().info(f"{req.a} + {req.b} = {res.sum}")

def main(args=None):
    rclpy.init(args=args)
    node = ClientNode()
    node.call_service(3, 7)
    node.call_service(10, 3)
    node.call_service(5, 4)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()