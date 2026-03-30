# Quiz 6 — Launch Files

> **Related part:** [Part 6](../parts/06-launch-files.md)

---

## Questions

**Q1.** What is the purpose of a launch file?
- A) To compile ROS2 packages
- B) To start multiple nodes at once with their configuration, from a single command
- C) To define message types
- D) To create URDF files

**Q2.** What does `$(find-pkg-share template_bringup)` resolve to?
- A) The source directory of the package
- B) The installed share directory of the package (e.g., `install/template_bringup/share/template_bringup`)
- C) The home directory
- D) The build directory

**Q3.** What does `<let name="urdf_path" value="..." />` do?
- A) Creates a ROS2 parameter
- B) Declares a launch-time variable that can be reused with `$(var urdf_path)`
- C) Sets an environment variable
- D) Creates a new file

**Q4.** What does `$(command 'xacro file.xacro')` do?
- A) Compiles xacro into C++
- B) Runs the `xacro` command at launch time and substitutes its output (the processed URDF XML)
- C) Creates a new xacro file
- D) Sends a service request

**Q5.** What does `<include>` do in a launch file?
- A) Includes a C++ header
- B) Includes (nests) another launch file, optionally passing arguments to it
- C) Imports a Python module
- D) Downloads a package

**Q6.** In the CMakeLists.txt, why is `install(DIRECTORY launch config ...)` needed?
- A) To compile the launch files
- B) To copy launch and config files into the install directory so `ros2 launch` can find them
- C) To create the directories
- D) To delete old files

**Q7.** When you press Ctrl+C on a launch file, what happens?
- A) Only the launch process stops, nodes keep running
- B) The launch system sends SIGINT to all managed child processes, shutting them down gracefully
- C) The computer reboots
- D) Nothing — you must kill each node individually

**Q8.** What does `<set_env name="GZ_SIM_RESOURCE_PATH" value="..." />` do?
- A) Creates a ROS2 parameter
- B) Sets an OS environment variable for the launched processes
- C) Publishes to a topic
- D) Creates a directory

**Q9.** How many launch file formats does ROS2 support?
- A) Only Python
- B) Only XML
- C) Three: XML (.launch.xml), Python (.launch.py), and YAML (.launch.yaml)
- D) Two: XML and JSON

**Q10.** When are substitutions like `$(find-pkg-share ...)` resolved?
- A) At compile time (colcon build)
- B) At launch time (when you run `ros2 launch`)
- C) At runtime (while the node is spinning)
- D) Never — they are passed as literal strings

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
