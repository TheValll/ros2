# Quiz 5 — Parameters

> **Related part:** [Part 5](../parts/05-parameters.md)

---

## Questions

**Q1.** What is a ROS2 parameter?
- A) A topic that sends configuration data
- B) A command-line argument that only works at startup
- C) A type of service
- D) A named configuration value stored inside a node, readable and writable at runtime

**Q2.** What does `declare_parameter("message", "Simple publisher")` do?
- A) Registers the parameter name in the node and sets a default value
- B) Publishes "Simple publisher" to a topic
- C) Creates a service called "message"
- D) Writes to a YAML file

**Q3.** In the YAML file, what does `ros__parameters` mean?
- A) A parameter named "ros__parameters"
- B) A Python import
- C) A reserved key that ROS2 uses to identify the parameter section for a node
- D) A DDS namespace

**Q4.** What is the priority order when a parameter has both a default value in code and a value in a YAML file?
- A) Code default always wins
- B) YAML file value overrides the code default
- C) It depends on which was set first
- D) Both values are kept as a list

**Q5.** How does `get_parameter("message")` find the value internally?
- A) Looks up the key in a hash map stored in the node's heap memory — O(1) average
- B) Reads from disk every time
- C) Sends a network request to the parameter server
- D) Searches through all nodes

**Q6.** Can you change a parameter while a node is running?
- A) No, parameters are fixed at startup
- B) Only in Python, not C++
- C) Only by restarting the node
- D) Yes, via `ros2 param set` or by calling the node's built-in parameter service

**Q7.** What hidden services does every node automatically create for parameters?
- A) None — you must create them manually
- B) Only get_parameters
- C) set_parameters, get_parameters, list_parameters, describe_parameters
- D) A single "parameters" topic

**Q8.** In the launch file, what does `<param from="path/to/file.yaml" />` do?
- A) Creates the YAML file
- B) Loads parameter values from the YAML file and passes them to the node at startup
- C) Publishes the YAML content to a topic
- D) Compiles the YAML into C++

**Q9.** What data structure stores parameters inside a node?
- A) A hash map (unordered_map)
- B) A linked list
- C) A binary tree
- D) A stack

**Q10.** In Python, what is the difference when reading a parameter compared to C++?
- A) Python can't read parameters
- B) C++ returns a string, Python returns bytes
- C) There is no difference
- D) Python uses `.value`, C++ uses `.as_string()` (or `.as_int()`, etc.)

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
