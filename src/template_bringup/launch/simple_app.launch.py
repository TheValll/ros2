from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    publisher = Node(package="cpp_pkg", executable="publisher")

    subscriber = Node(package="cpp_pkg", executable="subscriber")

    ld.add_action(publisher)
    ld.add_action(subscriber)

    return ld
