# ROS 2 Node Template

This repository serves as a template for creating ROS 2 nodes using both C++ (`cpp_pkg`) and Python (`py_pkg`).

## ROS 2 Command Cheat Sheet

**Build the entire workspace:**

```bash
colcon build
```

**Build only a specific package:**

```bash
colcon build --packages-select <package_name>
```

**Run a specific node from a package:**

```bash
ros2 run <package_name> <executable_name>
```

**Run a node and rename it at runtime (Remapping):**

```bash
ros2 run <package_name> <executable_name> --ros-args -r __node:=<new_node_name>
```

**Run a topic and rename it at runtime (Remapping):**

```bash
ros2 run <package_name> <executable_name> --ros-args -r __node:=<new_node_name> -r <topic_name>:=<new_topic_name>
```

**List all active nodes:**

```bash
ros2 node list
```

**Get detailed information about a specific node:**

```bash
ros2 node info <node_name>
```

**List all active topics:**

```bash
ros2 topic list
```

**Get info about a specific topic:**

```bash
ros2 topic info <topic_name>
```

**Subscribe to a specific topic:**

```bash
ros2 topic echo <topic_name>
```

**Visualize the node and topic graph (GUI):**

```bash
rqt_graph
```

**Show the structure of a message interface:**

```bash
ros2 interface show example_interfaces/msg/String
```

**Publish to a specific topic:**

```bash
ros2 topic pub -r <frequency> <topic_name> <type:example_interfaces/msg/String> <data>
```

**Show available bag commands:**

```bash
ros2 bag -h
```

**Record a specific topic:**

```bash
ros2 bag record <topic_name>
```

**Record a topic with a custom bag name:**

```bash
ros2 bag record -o <bag_name> <topic_name>
```

**Record all topics:**

```bash
ros2 bag record -o <bag_name> -a
```

**Display bag information:**

```bash
ros2 bag info <bag_name>
```

**Play back a recorded bag:**

```bash
ros2 bag play <bag_name>
```

**Call a server in command line:**

```bash
ros2 service call <server_name> <server_type> <request>
```

## Project Structure

```text
ROS_WS/
├── bags/                     # Bag file storage directory
│
└── src/
    ├── cpp_pkg/              # C++ Package
    │   ├── CMakeLists.txt    # Build configuration for CMake
    │   ├── package.xml       # Package metadata and dependencies
    │   ├── include/          # Header files
    │   └── src/
    │       ├── minimal_node.cpp     # Basic node example
    │       ├── publisher_node.cpp   # Publisher example
    │       └── subscriber_node.cpp  # Subscriber example
            ├── server_node.cpp      # Server example based on the AddTwoInts interface
            └── client_node.cpp      # Client example based on the AddTwoInts interface
    │
    └── py_pkg/               # Python Package
        ├── setup.py          # Build configuration for Python
        ├── package.xml       # Package metadata
        ├── resource/         # Resource markers for ament
        └── py_pkg/
            ├── __init__.py
            ├── minimal_node.py      # Basic node example
            ├── publisher_node.py    # Publisher example
            ├── subscriber_node.py   # Subscriber example
            ├── server_node.py       # Server example based on the AddTwoInts interface
            └── client_node.py       # Client example based on the AddTwoInts interface
```
