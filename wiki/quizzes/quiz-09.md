# Quiz 9 — Hardware Interface

> **Related part:** [Part 9](../parts/09-hardware-interface.md)

---

## Questions

**Q1.** What does `SystemInterface` mean?
- A) A single sensor
- B) The operating system interface
- C) A hardware interface that manages multiple joints as one system (like two wheels on the same bus)
- D) A network protocol

**Q2.** What is the correct lifecycle order for a hardware interface?
- A) on_init -> on_configure -> on_activate -> read/write loop -> on_deactivate
- B) activate -> init -> configure -> read/write
- C) read -> write -> configure -> activate
- D) configure -> read -> write -> init

**Q3.** What does `on_init()` do in this implementation?
- A) Opens the serial port
- B) Sends commands to the servo
- C) Starts the control loop
- D) Parses URDF parameters (servo_id, baudrate, port), zeros arrays, creates the driver object

**Q4.** What does `on_configure()` do?
- A) Parses the URDF
- B) Opens the serial port connection via `driver_->init()`
- C) Reads sensor data
- D) Publishes joint states

**Q5.** In the `read()` function, what does `hw_positions_[i] += hw_commands_[i] * period.seconds()` compute?
- A) Euler integration: estimates position by adding velocity * time step
- B) The acceleration of the wheel
- C) The torque on the wheel
- D) The distance in meters

**Q6.** Why does `write()` check `std::isnan(cmd)`?
- A) To check if the servo is connected
- B) To measure performance
- C) Because uninitialized command interfaces may contain NaN, and sending NaN to hardware would be dangerous
- D) Because NaN means the motor is overheating

**Q7.** What does `PLUGINLIB_EXPORT_CLASS` do?
- A) Compiles the class
- B) Creates a ROS2 node
- C) Generates the URDF
- D) Registers the class so the Controller Manager can load it at runtime by name, without compile-time knowledge

**Q8.** Where do the hardware parameters (servo_id, baudrate, port) come from?
- A) From environment variables
- B) From the `<param>` tags inside the `<ros2_control><hardware>` section of the URDF
- C) From the command line
- D) From a Python script

**Q9.** What is stored in `hw_commands_[0]` and `hw_commands_[1]`?
- A) The target velocities for left and right wheels (in rad/s), set by the controller
- B) The servo IDs
- C) The serial port names
- D) Error codes

**Q10.** What method is called when the hardware is being shut down?
- A) `read()`
- B) `on_init()`
- C) `on_deactivate()` — which closes the serial port and returns servo to default position
- D) `write()`

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
