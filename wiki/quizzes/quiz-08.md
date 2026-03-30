# Quiz 8 — ros2_control Architecture

> **Related part:** [Part 8](../parts/08-ros2-control-architecture.md)

---

## Questions

**Q1.** What are the three layers of ros2_control?
- A) Application, Controller, Hardware Interface
- B) Topic, Service, Action
- C) Publisher, Subscriber, Service
- D) URDF, Launch, Config

**Q2.** What is the order of operations in each control loop cycle?
- A) write -> update -> read
- B) update -> read -> write
- C) read -> write -> update
- D) read -> update -> write

**Q3.** If `update_rate: 50`, how often does the control loop run?
- A) Every 50 seconds
- B) Every 50ms
- C) Every 20ms (50 times per second)
- D) Once per minute

**Q4.** What is a "command interface"?
- A) A CLI command
- B) A named double value that controllers write to and hardware reads from (e.g., desired velocity)
- C) A DDS topic
- D) A launch file parameter

**Q5.** What is a "state interface"?
- A) A named double value that hardware writes to and controllers read from (e.g., current position)
- B) A boolean flag
- C) A service response
- D) A log message

**Q6.** How do controllers and hardware exchange data in ros2_control?
- A) Via DDS topics (serialized messages over the network)
- B) Via files on disk
- C) Via HTTP REST API
- D) Via direct shared memory — plain doubles, no serialization, no network

**Q7.** What does the `spawner` node do?
- A) Creates a new robot
- B) Starts Gazebo
- C) Asks the Controller Manager to load and activate a specific controller
- D) Compiles the hardware interface

**Q8.** What is the main benefit of ros2_control's architecture?
- A) It separates hardware communication from control logic — swap hardware or controllers independently
- B) It makes the robot faster
- C) It removes the need for URDF
- D) It only works with one type of robot

**Q9.** Which component manages the control loop and all controllers?
- A) robot_state_publisher
- B) The Controller Manager (ros2_control_node)
- C) DDS
- D) RViz

**Q10.** Why should hardware interface `read()` and `write()` be fast?
- A) Because slow operations use more RAM
- B) Because DDS requires fast operations
- C) Because Python is slow
- D) Because they run in a fixed-rate loop — if they take too long, the loop falls behind (real-time violation)

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
