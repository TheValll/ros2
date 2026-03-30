# Final Quiz — 50 Questions

> Covers all 12 parts: ROS2 Basics + ros2_control

---

## Block A: ROS2 Basics (Questions 1-28)

### Part 1 — Nodes, DDS & the Graph

**Q1.** What middleware does ROS2 use for communication between nodes?
- A) HTTP
- B) DDS (Data Distribution Service)
- C) MQTT
- D) gRPC

**Q2.** What does `rclcpp::spin(node)` do internally?
- A) Creates a new thread for each topic
- B) Runs an infinite event loop that waits for and dispatches callbacks (timers, messages, services)
- C) Sends one message and exits
- D) Compiles the node

**Q3.** Where is the node object allocated when using `std::make_shared<MyNode>()`?
- A) Stack
- B) Heap
- C) GPU memory
- D) Disk

**Q4.** What replaced `rosmaster` from ROS1 in ROS2?
- A) A new centralized server
- B) DDS auto-discovery via UDP multicast — no central server needed
- C) A configuration file
- D) DNS

### Part 2 — Topics & Pub/Sub

**Q5.** Topics use which communication pattern?
- A) Request/Response
- B) Publish/Subscribe (asynchronous, one-way)
- C) RPC (Remote Procedure Call)
- D) Polling

**Q6.** What does a QoS queue depth of 10 mean?
- A) Maximum 10 subscribers allowed
- B) The publisher/subscriber buffers up to 10 messages — oldest dropped if full
- C) Messages are sent 10 times for reliability
- D) The topic name must be 10 characters

**Q7.** What serialization format do ROS2 messages use on the wire?
- A) JSON
- B) CDR (Common Data Representation)
- C) MessagePack
- D) XML

**Q8.** A C++ publisher and a Python subscriber on the same topic:
- A) Cannot communicate
- B) Communicate perfectly — both use CDR serialization over DDS
- C) Need a bridge node
- D) Only work on different machines

### Part 3 — Services

**Q9.** What is the key difference between topics and services?
- A) Topics are faster
- B) Topics are one-way async (pub/sub), services are two-way sync (request/response)
- C) Services can only use Python
- D) Topics can only use C++

**Q10.** Why should you use `async_send_request` instead of a blocking call in ROS2?
- A) Because blocking is not implemented
- B) Because a blocking call inside `spin()` would deadlock — the thread can't process the response while blocked
- C) Because async is always preferred
- D) Because the server requires it

**Q11.** Under the hood, a ROS2 service uses:
- A) One DDS topic
- B) Two hidden DDS topics (request + response)
- C) HTTP
- D) Shared files

**Q12.** What is a Future?
- A) A time prediction
- B) A container for a value that will be available later when the async response arrives
- C) A thread
- D) A timer

### Part 4 — Custom Interfaces

**Q13.** What does `rosidl_generate_interfaces` do in CMakeLists.txt?
- A) Installs the package
- B) Triggers code generation from .msg/.srv files into C, C++, Python code and DDS type support
- C) Compiles a node
- D) Runs tests

**Q14.** Why are there padding bytes in a serialized message between a `bool` (1 byte) and a `float64` (8 bytes)?
- A) For security
- B) Memory alignment — CPUs read faster when data is at its natural boundary
- C) To add checksums
- D) It's a bug

**Q15.** How many bytes does `float64` use in memory?
- A) 4
- B) 8
- C) 16
- D) 64

**Q16.** What standard defines how `float64` is stored in binary?
- A) ASCII
- B) IEEE 754
- C) Unicode
- D) CDR

### Part 5 — Parameters

**Q17.** What happens if a YAML file sets `message: "Hello World"` but the code declares `declare_parameter("message", "Default")`?
- A) The code default wins: "Default"
- B) The YAML value wins: "Hello World"
- C) An error occurs
- D) Both values are stored

**Q18.** What data structure stores parameters inside a node?
- A) Array
- B) Hash map (unordered_map) — O(1) average lookup
- C) Linked list
- D) Binary tree

**Q19.** Can parameters be changed while a node is running?
- A) No, they're fixed at startup
- B) Yes, via `ros2 param set` or the node's built-in parameter services
- C) Only if the node restarts
- D) Only in debug mode

### Part 6 — Launch Files

**Q20.** What does `$(find-pkg-share template_bringup)` resolve to?
- A) The source code path
- B) The installed share directory of the package
- C) The build directory
- D) The home directory

**Q21.** What does `$(command 'xacro file.xacro')` do in a launch file?
- A) Compiles xacro
- B) Runs the xacro tool at launch time and substitutes its stdout output
- C) Downloads the file
- D) Creates a node

**Q22.** What does `<include file="...">` do in an XML launch file?
- A) Imports a C++ header
- B) Nests another launch file, executing its nodes as part of this launch
- C) Includes a YAML config
- D) Creates a symbolic link

### Part 7 — URDF & Visualization

**Q23.** What joint type allows infinite rotation like a wheel?
- A) fixed
- B) revolute
- C) continuous
- D) prismatic

**Q24.** What is the inertia formula Ixx for a box of mass m and dimensions x, y, z?
- A) `(m/12) * (x^2 + y^2)`
- B) `(m/12) * (y^2 + z^2)`
- C) `m * x * y * z`
- D) `(2/5) * m * r^2`

**Q25.** Why do URDF wheels need `rpy="${pi/2} 0 0"` in their origin?
- A) To make them invisible
- B) URDF cylinders are vertical by default; the rotation makes them horizontal for rolling
- C) To increase their size
- D) To change their color

**Q26.** What does robot_state_publisher do?
- A) Controls the motors
- B) Reads the URDF and broadcasts TF transforms so all nodes know where each link is
- C) Publishes camera images
- D) Compiles the URDF

**Q27.** In TF2, how do you get frame C's position relative to frame A through frame B?
- A) Add: pos_A + pos_B
- B) Multiply transformation matrices: T(A->B) * T(B->C)
- C) Subtract: pos_C - pos_A
- D) Average the positions

**Q28.** What are the three sub-elements of a URDF link?
- A) Node, topic, service
- B) Visual (appearance), collision (physics contacts), inertial (mass + inertia)
- C) Header, body, footer
- D) X, Y, Z

---

## Block B: ros2_control (Questions 29-50)

### Part 8 — ros2_control Architecture

**Q29.** What are the three layers of ros2_control?
- A) DDS, Node, Topic
- B) Application, Controller, Hardware Interface
- C) Linux, ROS2, Python
- D) URDF, Launch, Config

**Q30.** In each control loop cycle, the order of operations is:
- A) write -> read -> update
- B) update -> write -> read
- C) read -> update (controllers) -> write
- D) read -> write -> update

**Q31.** If update_rate is 50, the loop runs every:
- A) 50ms
- B) 20ms (1/50 = 0.02 seconds)
- C) 50 seconds
- D) 2ms

**Q32.** How do controllers and hardware exchange data?
- A) Via DDS topics with serialization
- B) Via shared memory — plain double values, no serialization, no network
- C) Via HTTP REST API
- D) Via files on disk

### Part 9 — Hardware Interface

**Q33.** What is the correct lifecycle order for a hardware interface?
- A) read -> write -> init
- B) on_init -> on_configure -> on_activate -> read/write loop -> on_deactivate
- C) activate -> configure -> init
- D) write -> read -> deactivate

**Q34.** In the `read()` function, `position += velocity * dt` is called:
- A) Derivative
- B) Euler integration — estimating position by integrating velocity over time
- C) Fourier transform
- D) Matrix multiplication

**Q35.** Why check `std::isnan(cmd)` in the `write()` function?
- A) To detect servo overheating
- B) Because uninitialized command interfaces may be NaN — sending NaN to hardware is dangerous
- C) To count the commands
- D) For performance measurement

**Q36.** What does `PLUGINLIB_EXPORT_CLASS` do?
- A) Creates a new ROS2 node
- B) Registers the class for dynamic loading — the Controller Manager can find it at runtime
- C) Compiles the URDF
- D) Publishes a topic

### Part 10 — ros2_control URDF

**Q37.** In `<ros2_control type="system">`, what does "system" mean?
- A) Operating system
- B) The hardware manages multiple joints as one unit (maps to SystemInterface)
- C) A sensor
- D) A single motor

**Q38.** What does `<command_interface name="velocity" />` create?
- A) A ROS2 topic
- B) A double value in shared memory that controllers write to (desired wheel speed)
- C) A service
- D) A parameter

**Q39.** Where do URDF `<param>` values (servo_id, baudrate, port) end up in C++?
- A) In environment variables
- B) In `info_.hardware_parameters` map, available in `on_init()`
- C) In a YAML file
- D) In the ROS2 parameter server

**Q40.** What is `mock_components/GenericSystem`?
- A) A real servo driver
- B) A fake hardware interface for testing — no real hardware needed
- C) A Gazebo world
- D) A launch file

### Part 11 — DiffDrive Controllers

**Q41.** The inverse kinematics formula for the left wheel is:
- A) `w_L = v * r`
- B) `w_L = (v - omega * L/2) / r`
- C) `w_L = omega / L`
- D) `w_L = v + omega * r`

**Q42.** If both wheels spin at the same speed, the robot:
- A) Turns left
- B) Turns right
- C) Goes straight
- D) Spins in place

**Q43.** If the right wheel is faster than the left wheel, the robot turns:
- A) Right
- B) Left
- C) It goes straight
- D) It stops

**Q44.** With r=0.1, L=0.45, v=0, omega=1.0 — what is w_R?
- A) 0.0 rad/s
- B) 2.25 rad/s
- C) 10.0 rad/s
- D) -2.25 rad/s

**Q45.** What is odometry?
- A) A type of motor
- B) Estimating the robot's position over time by integrating wheel velocities
- C) A networking protocol
- D) A URDF tag

**Q46.** Why does odometry drift over time?
- A) Because the URDF is wrong
- B) Because Euler integration accumulates small errors, and real wheels slip
- C) Because DDS loses packets
- D) Because the timer is inaccurate

### Part 12 — LX-225 Driver

**Q47.** What does "8N1" mean in UART?
- A) 8 servos, No errors, 1 bus
- B) 8 data bits, No parity, 1 stop bit
- C) 8 bytes per packet, No checksum, 1 wire
- D) 8 MHz clock, No buffer, 1 channel

**Q48.** At 115200 baud (8N1), how many bytes per second can be transferred?
- A) 115200
- B) 11520 (each byte needs 10 bits: start + 8 data + stop)
- C) 1152
- D) 57600

**Q49.** What does `tcflush(handle, TCIFLUSH)` do?
- A) Sends buffered data
- B) Clears the receive buffer, discarding stale unread bytes
- C) Closes the port
- D) Resets the servo

**Q50.** In `set_position(800, 1000)`, what do the two arguments mean?
- A) speed=800, torque=1000
- B) target_position=800 (range 0-1000), duration=1000ms (time to reach position)
- C) x=800, y=1000
- D) servo_id=800, baudrate=1000

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
| 11 |       |
| 12 |       |
| 13 |       |
| 14 |       |
| 15 |       |
| 16 |       |
| 17 |       |
| 18 |       |
| 19 |       |
| 20 |       |
| 21 |       |
| 22 |       |
| 23 |       |
| 24 |       |
| 25 |       |
| 26 |       |
| 27 |       |
| 28 |       |
| 29 |       |
| 30 |       |
| 31 |       |
| 32 |       |
| 33 |       |
| 34 |       |
| 35 |       |
| 36 |       |
| 37 |       |
| 38 |       |
| 39 |       |
| 40 |       |
| 41 |       |
| 42 |       |
| 43 |       |
| 44 |       |
| 45 |       |
| 46 |       |
| 47 |       |
| 48 |       |
| 49 |       |
| 50 |       |

---

## Corrections

_To be filled after answering._

---

## Score: ___ / 50
