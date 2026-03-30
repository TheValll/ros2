# ROS2 Learning Wiki

A structured deep-dive into ROS2, from basic concepts to ros2_control hardware interfaces.
Each part includes detailed explanations (hardware, memory, math when relevant) and a quiz.

---

## Syllabus

### Block A: ROS2 Basics

| # | Part | Key Concepts | Files |
|---|------|-------------|-------|
| 1 | [Nodes, DDS & the Graph](parts/01-nodes-dds-graph.md) | Processes, DDS discovery, spin loop, memory layout | `minimal_node.cpp`, `minimal_node.py` |
| 2 | [Topics & Pub/Sub](parts/02-topics-pub-sub.md) | Async messaging, queues, QoS, serialization | `publisher_node`, `subscriber_node` |
| 3 | [Services: Request/Reply](parts/03-services.md) | Sync communication, client-server, executors | `server_node`, `client_node` |
| 4 | [Custom Interfaces](parts/04-custom-interfaces.md) | .msg/.srv files, IDL, CDR serialization, codegen | `MinimalInterface.msg`, `MinimalService.srv` |
| 5 | [Parameters](parts/05-parameters.md) | Parameter server, YAML config, dynamic reconfigure | `parameters_node`, `minimal_params.yaml` |
| 6 | [Launch Files](parts/06-launch-files.md) | Process orchestration, arguments, composition | `simple_app.launch.xml`, `display.launch.xml` |
| 7 | [URDF & Visualization](parts/07-urdf-visualization.md) | Xacro, links, joints, inertia matrices, TF2 | `basic_urdf.urdf.xacro`, `mobile_base.xacro` |

### Block B: ROS2 Control

| # | Part | Key Concepts | Files |
|---|------|-------------|-------|
| 8 | [ros2_control Architecture](parts/08-ros2-control-architecture.md) | Controller Manager, real-time loop, read/update/write | `template_controllers.yaml` |
| 9 | [Hardware Interface](parts/09-hardware-interface.md) | SystemInterface, lifecycle, state/command interfaces | `mobile_base_hardware_interface.hpp/.cpp` |
| 10 | [ros2_control URDF](parts/10-ros2-control-urdf.md) | `<ros2_control>` tags, plugins, pluginlib | `mobile_base.ros2_control.xacro` |
| 11 | [Controllers: DiffDrive](parts/11-controllers-diffdrive.md) | Differential drive kinematics, odometry math | `template_controllers.yaml` |
| 12 | [Hardware Driver: LX-225](parts/12-lx225-driver.md) | UART, serial protocol, baudrate, servo commands | `LX225Driver.hpp`, `lx225_test.cpp` |

### Block C: Final Assessment

| # | Part | Description |
|---|------|-------------|
| F | [Final Quiz (50 questions)](quizzes/final-quiz.md) | Covers all 12 parts |

---

## Quiz Tracking

| Part | Quiz | My Answers | Score |
|------|------|-----------|-------|
| 1 | [Quiz 1](quizzes/quiz-01.md) | completed | 10/10 |
| 2 | [Quiz 2](quizzes/quiz-02.md) | _pending_ | _/10_ |
| 3 | [Quiz 3](quizzes/quiz-03.md) | _pending_ | _/10_ |
| 4 | [Quiz 4](quizzes/quiz-04.md) | _pending_ | _/10_ |
| 5 | [Quiz 5](quizzes/quiz-05.md) | _pending_ | _/10_ |
| 6 | [Quiz 6](quizzes/quiz-06.md) | _pending_ | _/10_ |
| 7 | [Quiz 7](quizzes/quiz-07.md) | _pending_ | _/10_ |
| 8 | [Quiz 8](quizzes/quiz-08.md) | _pending_ | _/10_ |
| 9 | [Quiz 9](quizzes/quiz-09.md) | _pending_ | _/10_ |
| 10 | [Quiz 10](quizzes/quiz-10.md) | _pending_ | _/10_ |
| 11 | [Quiz 11](quizzes/quiz-11.md) | _pending_ | _/10_ |
| 12 | [Quiz 12](quizzes/quiz-12.md) | _pending_ | _/10_ |
| Final | [Final Quiz](quizzes/final-quiz.md) | _pending_ | _/50_ |
