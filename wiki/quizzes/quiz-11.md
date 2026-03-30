# Quiz 11 — Controllers: DiffDrive & JointStateBroadcaster

> **Related part:** [Part 11](../parts/11-controllers-diffdrive.md)

---

## Questions

**Q1.** What does the JointStateBroadcaster do?
- A) Commands the wheels
- B) Reads state interfaces and publishes joint positions/velocities to the `/joint_states` topic
- C) Parses the URDF
- D) Controls the camera

**Q2.** What topic does the DiffDriveController subscribe to for motion commands?
- A) `/joint_states`
- B) `/odom`
- C) `/cmd_vel` (geometry_msgs/Twist)
- D) `/simple_topic`

**Q3.** For a differential drive robot, if both wheels spin at the same speed, the robot:
- A) Turns left
- B) Turns right
- C) Goes straight
- D) Spins in place

**Q4.** Given wheel_radius=0.1, wheel_separation=0.45, v=1.0 m/s, omega=0 — what are the wheel speeds?
- A) w_L = 5, w_R = 15
- B) w_L = 10, w_R = 10
- C) w_L = 0, w_R = 20
- D) w_L = 10, w_R = 0

**Q5.** What is the inverse kinematics formula for the left wheel?
- A) `w_L = v * r`
- B) `w_L = (v - omega * L/2) / r`
- C) `w_L = omega / r`
- D) `w_L = v + omega`

**Q6.** If the right wheel spins faster than the left wheel, the robot:
- A) Goes straight
- B) Turns right
- C) Turns left (the faster right wheel pushes the right side forward more)
- D) Stops

**Q7.** What is odometry?
- A) A type of sensor
- B) The estimation of the robot's position and orientation over time, by integrating wheel velocities
- C) A network protocol
- D) A type of controller

**Q8.** In the odometry equations, what does `theta` represent?
- A) The robot's speed
- B) The robot's heading angle (yaw) — the direction it's facing
- C) The wheel radius
- D) The timestamp

**Q9.** Why does odometry drift over time?
- A) Because the wheels are too small
- B) Because Euler integration accumulates small errors, and wheels can slip
- C) Because ROS2 has a bug
- D) Because the URDF is wrong

**Q10.** What does the `pose_covariance_diagonal` parameter represent?
- A) The maximum speed
- B) The uncertainty (error estimate) in the odometry position, used by sensor fusion algorithms
- C) The wheel dimensions
- D) The controller update rate

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
