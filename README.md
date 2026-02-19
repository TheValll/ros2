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

**Run a node in a specific namespace (Remapping):**

```bash
ros2 run <package_name> <executable_name> --ros-args -r __ns:=/<namespace_name>
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

**List all active services:**

```bash
ros2 service list
```

**Show the type of a specific service:**

```bash
ros2 service type <service_name>
```

**Call a server in command line:**

```bash
ros2 service call <server_name> <server_type> <request>
```

**Run a service and rename it at runtime (Remapping):**

```bash
ros2 run <package_name> <executable_name> --ros-args -r <old_service_name>:=<new_service_name>
```

**List parameters:**

```bash
ros2 param list
```

**Run a specific node from a package with parameters:**

```bash
ros2 run <package_name> <executable_name> --ros-args -p <parameter_name>:=<value>
ros2 run <package_name> <executable_name> --ros-args -p <parameter_name1>:=<value1> -p <parameter_name2>:=<value2>
```

**Get the value of a specific parameter:**

```bash
ros2 param get <node_name> <parameter_name>
```

**Run node with parameters file:**

```bash
ros2 run <package_name> <executable_name> --ros-args --params-file <path_of_your_params_file>
```

**Change parameters at runtime:**

```bash
ros2 param set <package_name> <parameter_name> <value> # Need the parameters_callback (no template available in this repo)
```

**Launch at launch file:**

```bash
ros2 launch <package_name> <launch_file_name>
```

**Run a node in a specific namespace in XML launch file config:**

```xml
<node pkg="<package_name>" exec="<executable_name>" namespace="<namespace_name>" />
```

**Remapping topic in XML launch file config:**

```xml
<node pkg="<package_name>" exec="<executable_name>">
  <remap from="<old_topic_name>" to="<new_topic_name>" />
</node>
```

**Add parameters in XML launch file config:**

```xml
<node pkg="<package_name>" exec="<executable_name>">
  <param name="<parameter_name>" value="<value>"/>
</node>
```

**Add parameters in XML launch file config via yaml file:**

```xml
<node pkg="<package_name>" exec="<executable_name>">
  <param from="$(find-pkg-share <package_name>)/path_of_your_params_file" />
</node>
```

**Generate a PDF file showing the TF frame tree:**

```bash
sudo apt install ros-jazzy-tf2-tools
ros2 run tf2_tools view_frames # Listens to the /tf topic and generates a PDF of the frame tree
```

**Open an URDF file with RViz**

```bash
sudo apt install ros-jazzy-urdf-tutorial
ros2 launch urdf_tutorial display.launch.py model:=<path_of_your_urdf_file>
```

## Project Structure

```text
ROS_WS/
├── bags/                       # ROS2 bag files storage
│
├── yaml_params/
│   └── minimal_params.yaml     # Parameters file
│
└── src/
    ├── template_bringup/             # Launch file package
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── launch/
    │   │   ├── simple_app.launch.xml          # XML launch file configuration
    │   │   └── simple_app.launch.py           # Python launch file configuration
    │   └── config/
    │       └── minimal_params.yaml            # Parameters file (same file from yaml_params folder)
    │
    ├── custom_interfaces/            # Custom interface package
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── msg/
    │   │   └── MinimalInterface.msg   # Custom message definition
    │   └── srv/
    │       └── MinimalService.srv     # Custom service definition
    │
    ├── cpp_pkg/                      # C++ Package
    │   ├── CMakeLists.txt            # Build configuration for CMake
    │   ├── package.xml               # Package metadata and dependencies
    │   ├── include/
    │   │   └── cpp_pkg/
    │   └── src/
    │       ├── minimal_node.cpp              # Basic node example
    │       ├── publisher_node.cpp            # Publisher example
    │       ├── parameters_node.cpp           # Publisher example with parameters declaration
    │       ├── subscriber_node.cpp           # Subscriber example
    │       ├── server_node.cpp               # Server example (AddTwoInts)
    │       ├── client_node.cpp               # Client example (AddTwoInts)
    │       └── custom_interface_node.cpp     # Publisher using MinimalInterface
    │
    └── py_pkg/                       # Python Package
        ├── package.xml               # Package metadata
        ├── setup.py                  # Build configuration for Python
        ├── setup.cfg
        ├── resource/
        │   └── py_pkg
        └── py_pkg/
            ├── __init__.py
            ├── minimal_node.py              # Basic node example
            ├── publisher_node.py            # Publisher example
            ├── parameters_node.py           # Publisher example with parameters declaration
            ├── subscriber_node.py           # Subscriber example
            ├── server_node.py               # Server example (AddTwoInts)
            ├── client_node.py               # Client example (AddTwoInts)
            └── custom_interface_node.py     # Publisher using MinimalInterface
```
