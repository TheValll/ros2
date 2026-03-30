# Quiz 1 — Nodes, DDS & the Graph

> **Related part:** [Part 1](../parts/01-nodes-dds-graph.md)

---

## Questions

**Q1.** What is a Node in ROS2?
- A) A configuration file
- **B) A process (or part of a process) that performs a specific task** :white_check_mark:
- C) A message sent over the network
- D) A type of sensor

**Q2.** What does `rclcpp::init()` do?
- A) Compiles the code
- B) Creates a publisher
- **C) Initializes the ROS2 context and the DDS DomainParticipant** :white_check_mark:
- D) Starts the spin loop

**Q3.** What does `rclcpp::spin(node)` do?
- A) Compiles and runs the node
- B) Sends a message once
- **C) Runs an infinite loop that waits for and processes events (timers, messages, etc.)** :white_check_mark:
- D) Stops the node

**Q4.** Where is the `MinimalNode` object allocated in memory when using `std::make_shared`?
- A) On the stack
- **B) On the heap** :white_check_mark:
- C) In a file on disk
- D) In GPU memory

**Q5.** How do ROS2 nodes discover each other?
- A) Via a central server (rosmaster)
- B) By reading a shared configuration file
- **C) Via UDP multicast packets sent by DDS** :white_check_mark:
- D) The user must connect them manually

**Q6.** What is the Domain ID in DDS?
- A) The robot's IP address
- **B) A "radio channel": only nodes on the same Domain ID can see each other** :white_check_mark:
- C) The ROS2 version number
- D) The web server port

**Q7.** What is the main under-the-hood difference between `rclcpp` and `rclpy`?
- A) They use different network protocols
- **B) `rclcpp` calls the C/DDS layer directly, `rclpy` is a Python wrapper around the same C code** :white_check_mark:
- C) `rclpy` cannot use timers
- D) `rclcpp` doesn't support topics

**Q8.** What does this line in `setup.py` declare: `"minimal_py_node = py_pkg.minimal_node:main"`?
- A) A build dependency
- **B) An entry point: when you run `ros2 run py_pkg minimal_py_node`, it calls the `main()` function** :white_check_mark:
- C) A topic to publish
- D) A default parameter

**Q9.** Why is communication faster between two nodes on the same machine?
- A) Because Python is faster than C++
- **B) Because DDS can use shared memory instead of UDP networking** :white_check_mark:
- C) Because the nodes share the same PID
- D) Because the timer is shorter

**Q10.** What does `ament_target_dependencies(minimal_cpp_node rclcpp)` do in CMakeLists.txt?
- A) Launches the node
- **B) Automatically adds rclcpp's headers and libraries during compilation** :white_check_mark:
- C) Creates a package.xml file
- D) Installs ROS2

---

## My Answers

| Q | Answer | Result |
|---|--------|--------|
| 1 | B | :white_check_mark: |
| 2 | C | :white_check_mark: |
| 3 | C | :white_check_mark: |
| 4 | B | :white_check_mark: |
| 5 | C | :white_check_mark: |
| 6 | B | :white_check_mark: |
| 7 | B | :white_check_mark: |
| 8 | B | :white_check_mark: |
| 9 | B | :white_check_mark: |
| 10 | B | :white_check_mark: |

---

## Score: 10 / 10
