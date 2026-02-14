#!/usr/bin/env python3
# Based on the AddTwoInts interface
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class ServerNode(Node):

    def __init__(self):
        super().__init__("server_node")
        self.server_ = self.create_service(AddTwoInts, "add_two_ints", self.callback_server)
        self.get_logger().info("Server node is running ...")

    def callback_server(self, req: AddTwoInts.Request, res: AddTwoInts.Response):
        res.sum = req.a + req.b
        return res

def main(args=None):
    rclpy.init(args=args)
    node = ServerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()