# Quiz 10 — ros2_control URDF

> **Related part:** [Part 10](../parts/10-ros2-control-urdf.md)

---

## Questions

**Q1.** What does `type="system"` in `<ros2_control>` mean?
- A) It's an operating system driver
- B) The hardware manages multiple joints together as a system (maps to SystemInterface)
- C) It's a sensor
- D) It's a single actuator

**Q2.** What does `<plugin>mobile_base_hardware/MobileBaseHardware</plugin>` specify?
- A) A Python module to import
- B) The pluginlib name used to dynamically load the hardware interface class at runtime
- C) A ROS2 topic name
- D) A launch file to include

**Q3.** Where do the `<param>` values (servo_id, baudrate, port) end up in the C++ code?
- A) In environment variables
- B) In `info_.hardware_parameters` map, accessible in `on_init()`
- C) In the ROS2 parameter server
- D) In a YAML file

**Q4.** What does `<command_interface name="velocity" />` declare?
- A) A topic named "velocity"
- B) A double value in shared memory that controllers can write to (e.g., target wheel speed)
- C) A service call
- D) A log level

**Q5.** Why must the joint name in `<ros2_control>` match a joint name in the URDF kinematic section?
- A) It doesn't need to match
- B) So the Controller Manager can link hardware interfaces to the physical joint in the robot model
- C) For color matching
- D) Because XML requires unique names

**Q6.** What does `PLUGINLIB_EXPORT_CLASS` do in the C++ code?
- A) Creates a new node
- B) Registers the class so `dlopen()` can find it when the Controller Manager loads the plugin dynamically
- C) Compiles the URDF
- D) Sends a network packet

**Q7.** Why is the hardware interface built as a **shared library** (`add_library(SHARED)`) instead of an executable?
- A) Because shared libraries are faster
- B) Because the Controller Manager loads it dynamically at runtime via pluginlib — it's not a standalone program
- C) Because executables don't support C++
- D) Because ROS2 only supports libraries

**Q8.** What is `mock_components/GenericSystem`?
- A) A real hardware driver
- B) A fake hardware interface for testing — echoes commands as states, no physical hardware needed
- C) A Gazebo plugin
- D) A visualization tool

**Q9.** What three files are needed to register a pluginlib plugin?
- A) package.xml, CMakeLists.txt, setup.py
- B) The C++ macro (PLUGINLIB_EXPORT_CLASS), the XML descriptor, and the CMake registration (pluginlib_export_plugin_description_file)
- C) URDF, YAML, launch file
- D) Header, source, makefile

**Q10.** What is the full interface name for the right wheel's velocity command?
- A) `velocity`
- B) `base_right_wheel_joint/velocity`
- C) `right_wheel/cmd`
- D) `/cmd_vel`

---

## My Answers

| Q | Answer |
|---|--------|
| 1 |        |
| 2 |        |
| 3 |        |
| 4 |        |
| 5 |        |
| 6 |        |
| 7 |        |
| 8 |        |
| 9 |        |
| 10 |       |

---

## Corrections

_To be filled after answering._

---

## Score: ___ / 10
