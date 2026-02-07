# ROS 2 Minimal Node Template

This repository serves as a template for creating minimal ROS 2 nodes using both C++ (`cpp_pkg`) and Python (`py_pkg`).

## ROS 2 Command Cheat Sheet

Below is a list of corrected common commands used to interact with these nodes.

### Running Nodes

**Run a specific node from a package:**

```bash
ros2 run <package_name> <executable_name>
```

**Run a node and rename it at runtime (Remapping):**

```bash
ros2 run <package_name> <executable_name> --ros-args -r __node:=<new_node_name>
```

### Introspection

**List all active nodes:**

```bash
ros2 node list
```

**Get detailed information about a specific node:**
(Displays publishers, subscribers, services, and actions)

```bash
ros2 node info <node_name>
```

## Project Structure

```text
ROS_WS/
└── src/
    ├── cpp_pkg/              # C++ Package
    │   ├── CMakeLists.txt    # Build configuration for CMake
    │   ├── package.xml       # Package metadata and dependencies
    │   ├── include/          # Header files
    │   └── src/
    │       └── minimal_node.cpp  # C++ Source entry point
    │
    └── py_pkg/               # Python Package
        ├── setup.py          # Build configuration for Python
        ├── package.xml       # Package metadata
        ├── resource/         # Resource markers for ament
        └── py_pkg/
            ├── __init__.py
            └── minimal_node.py   # Python Source entry point
```
