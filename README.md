# ROS 2 Node Template

This repository serves as a template for creating ROS 2 nodes using both C++ (`cpp_pkg`) and Python (`py_pkg`).

## Adding Gazebo Models

To use custom models (meshes, walls, shelves, etc.) in a Gazebo world:

1. Place your model in `src/basic_description/models/<model_name>/` with the following structure:
   ```text
   models/
   └── my_model/
       ├── model.config       # Model metadata
       ├── model.sdf          # Model SDF (use relative paths for meshes)
       └── meshes/
           └── my_mesh.dae    # Mesh files
   ```

2. The `models` directory is already installed via `CMakeLists.txt`. Just rebuild:
   ```bash
   colcon build --packages-select basic_description
   ```

3. Reference the model in your world SDF with a `model://` URI:
   ```xml
   <include>
     <uri>model://my_model</uri>
     <name>my_model_instance</name>
     <pose>0 0 0 0 0 0</pose>
   </include>
   ```

4. In your launch file, set `GZ_SIM_RESOURCE_PATH` so Gazebo can resolve `model://` URIs:
   ```xml
   <set_env name="GZ_SIM_RESOURCE_PATH" value="$(find-pkg-share basic_description)/models" />
   ```

> Standalone examples of models are also available in `utils/gazebo_basic_world/assets/`.

## ros2_control Hardware Interface

The file `src/ros2_control/urdf/mobile_base.ros2_control.xacro` defines the hardware plugin used by the controller manager.

By default it uses `mock_components/GenericSystem` with `calculate_dynamics: true`, which simulates a perfect loopback (velocity commands are directly reflected as state feedback). This is useful for testing the full ros2_control chain without real hardware.

To connect real servos, edit this file and replace the plugin:
```xml
<!-- Replace this -->
<plugin>mock_components/GenericSystem</plugin>
<param name="calculate_dynamics">true</param>

<!-- With your hardware interface plugin -->
<plugin>ros2_control_hardware_template/MobileBaseHardware</plugin>
```

The `ros2_control_hardware_template` package provides a hardware interface with the LX-225 servo driver. If you use a different servo, replace `LX225Driver.hpp` with your own driver and update the calls in `mobile_base_hardware_interface.cpp`.

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

**Open an URDF file with RViz without urdf_tutorial**

```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro <path_of_your_urdf_file>)"
sudo apt install ros-jazzy-joint-state-publisher-gui
ros2 run rviz2 rviz2
```

**Launch RViz with a saved config file:**

```bash
ros2 run rviz2 rviz2 -d <path_of_your_config>
```

**Launch Gazebo**
```bash
sudo apt install ros-jazzy-ros-gz
gz sim or gz sim <path_of_your_gazebo_file>
```

**Show Gazebo topics**
```bash
gz topic -l
```

**Show topics type**
```bash
gz topic -i -t <topic_name>
```

**Launch Gazebo from ROS2**
```bash
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=<path_of_your_gazebo_file>
```

**Launch Gazebo from ROS2 with empty SDF file and timer started**
```bash
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="empty.sdf -r"
```

**Spawn an URDF file in Gazebo**
```bash
ros2 run ros_gz_sim create -topic robot_description # You should start the robot_state_publisher before this command  
```

**Gazebo plugins repository**
```text
https://github.com/gazebosim/gz-sim/tree/gz-sim10/src/systems
```
**Add a bridge between ROS2 and Gazebo**
```bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="$(<path_of_your_config_file>)"
```

**ROS2 topic type convert to Gazebo topic type**
```text
https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge
```

**Gazebo models example**
```text
https://app.gazebosim.org/fuel/models
```

**Move a robot in a Gazebo world**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Install ROS2 control**
```bash
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers
```

**ROS2 control repository**
```text
https://github.com/ros-controls/ros2_control
```

**ROS2 controllers repository**
```text
https://github.com/ros-controls/ros2_controllers 
```

**Launch a controller manager file**
```bash
ros2 run controller_manager ros2_control_node --ros-args --params-file <path_of_your_controller_manager_config_file>
```

**Launch the spawner control manager**
```bash
ros2 run controller_manager spawner <name_of_your_controller>
```

**Show controllers list**
```bash
ros2 control list_controllers
```

**Show all controllers**
```bash
ros2 control list_controller_types
```

**Show command interfaces and state interfaces**
```bash
ros2 control list_hardware_interfaces
```

**Show components**
```bash
ros2 control list_hardware_components
```


## Project Structure

```text
ROS_WS/
├── bags/                       # ROS2 bag files storage
│
├── utils/
│   ├── basic_urdf/
│   │   └── basic_urdf.urdf         # Basic URDF example
│   ├── gazebo_basic_world/
│   │   └── basic_world.sdf         # Basic Gazebo world example
│   ├── yaml_params/
│   │   └── minimal_params.yaml     # Parameters file
│   ├── rviz_config/
│   │   └── urdf_config.rviz        # RViz configuration
│   └── LX-225-driver/
│       ├── LX225Driver.hpp          # LX-225 servo driver header
│       ├── LX225Driver_test.cpp     # Driver test source
│       └── LX225Driver_test         # Compiled driver test binary
│
└── src/
    ├── basic_description/            # URDF description package
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── config/
    │   │   └── gazebo_bridge.yaml             # Gazebo-ROS2 bridge configuration
    │   ├── launch/
    │   │   ├── display.launch.py              # Python launch file (robot_state_publisher + rviz)
    │   │   └── display.launch.xml             # XML launch file
    │   ├── rviz/
    │   │   └── urdf_config.rviz               # RViz configuration
    │   ├── urdf/
    │   │   ├── basic_urdf.urdf                # URDF robot description
    │   │   └── basic_urdf.urdf.xacro          # URDF robot description with Xacro
    │   └── worlds/
    │       └── basic_world.sdf                # Gazebo world file
    │
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
    ├── ros2_control/                  # ROS2 Control package
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── config/
    │   │   └── template_controllers.yaml    # Controllers configuration
    │   ├── launch/
    │   │   ├── display.launch.py            # Python launch file
    │   │   ├── display.launch.xml           # XML launch file
    │   │   └── old_display.launch.xml       # Old XML launch file
    │   ├── rviz/
    │   │   └── urdf_config.rviz             # RViz configuration
    │   └── urdf/
    │       ├── common_properties.xacro      # Common URDF properties
    │       ├── mobile_base.ros2_control.xacro # ros2_control hardware interface
    │       ├── mobile_base.xacro            # Mobile base URDF description
    │       └── my_robot.urdf.xacro          # Main robot URDF
    │
    ├── ros2_control_hardware_template/  # ros2_control hardware interface package
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── include/
    │   │   └── ros2_control_hardware_template/
    │   │       ├── LX225Driver.hpp                # LX-225 servo driver
    │   │       └── mobile_base_hardware_interface.hpp # Hardware interface header
    │   └── src/
    │       └── mobile_base_hardware_interface.cpp  # Hardware interface (loopback mock)
    │
    ├── lx225_driver_test/              # LX-225 servo driver test package
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── include/
    │   │   └── lx225_driver_test/
    │   │       └── LX225Driver.hpp          # LX-225 servo driver header
    │   └── src/
    │       └── lx225_test.cpp               # Driver test node
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
