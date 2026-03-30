# Quiz 7 — URDF & Visualization

> **Related part:** [Part 7](../parts/07-urdf-visualization.md)

---

## Questions

**Q1.** What are the two fundamental building blocks of a URDF?
- A) Nodes and topics
- B) Publishers and subscribers
- C) Links (rigid bodies) and joints (connections between links)
- D) Parameters and services

**Q2.** What joint type allows infinite rotation (like a wheel)?
- A) `continuous`
- B) `fixed`
- C) `revolute`
- D) `prismatic`

**Q3.** What does Xacro add to URDF?
- A) Physics simulation
- B) Network communication
- C) Compilation support
- D) Properties (variables), macros (reusable templates), and include (file splitting)

**Q4.** Why does the wheel cylinder need `rpy="${pi/2} 0 0"` in its origin?
- A) Because URDF cylinders are vertical by default, and the wheel needs to be horizontal (rotated 90 degrees around X)
- B) To make it invisible
- C) To change its color
- D) To double its size

**Q5.** What is the inertia matrix used for?
- A) Displaying colors in RViz
- B) The physics simulator uses it to calculate how the robot resists rotation when forces are applied
- C) Network routing
- D) File compression

**Q6.** For a box with mass m and dimensions x, y, z — what is the formula for Ixx?
- A) `Ixx = m * x * x`
- B) `Ixx = (2/5) * m * r^2`
- C) `Ixx = (m/12) * (y^2 + z^2)`
- D) `Ixx = m * g * h`

**Q7.** What does `robot_state_publisher` do?
- A) Publishes sensor data
- B) Controls the motors
- C) Compiles the URDF
- D) Reads the URDF and broadcasts TF transforms so every node knows where each link is

**Q8.** What coordinate convention does URDF use?
- A) x=forward, y=left, z=up
- B) x=up, y=right, z=forward
- C) x=left, y=forward, z=down
- D) It depends on the platform

**Q9.** What are the three sub-elements a link can have?
- A) Publisher, subscriber, service
- B) Visual (appearance), collision (physics contacts), inertial (mass and inertia)
- C) Header, body, footer
- D) Input, output, feedback

**Q10.** In TF2, how do you find the position of frame C relative to frame A, going through frame B?
- A) Add the positions: pos_A + pos_B + pos_C
- B) Subtract: pos_C - pos_A
- C) Multiply the transformation matrices: T(A->B) * T(B->C)
- D) It's not possible — you must measure directly

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
