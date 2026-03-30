# Quiz 4 — Custom Interfaces

> **Related part:** [Part 4](../parts/04-custom-interfaces.md)

---

## Questions

**Q1.** What is the purpose of a `.msg` file?
- A) It contains the C++ code for a node
- B) It defines the structure (fields and types) of a custom message, used by code generators
- C) It is a configuration file for launch
- D) It stores log messages

**Q2.** In a `.msg` file, `float64 a` means:
- A) A variable named "a" of type 64-byte float
- B) A field named "a" of type double-precision floating point (8 bytes, IEEE 754)
- C) A function called "a" that returns a float
- D) A constant with value 64

**Q3.** What does `rosidl_generate_interfaces` in CMakeLists.txt do?
- A) Creates the package.xml file
- B) Triggers code generation: converts .msg/.srv files into C, C++, Python code and DDS type support
- C) Compiles the node executable
- D) Installs the package

**Q4.** Why does the compiler add padding bytes between a `bool` (1 byte) and a `float64` (8 bytes)?
- A) To make the message bigger for DDS
- B) For memory alignment — CPUs read faster when data starts at addresses divisible by its size
- C) To add error-checking bytes
- D) It's a bug in the compiler

**Q5.** What intermediate language is a `.msg` file converted to before code generation?
- A) JSON
- B) YAML
- C) IDL (Interface Definition Language) — the DDS standard
- D) Protocol Buffers

**Q6.** After building `custom_interfaces`, where does the generated C++ header end up?
- A) In the source directory next to the .msg file
- B) In `install/custom_interfaces/include/custom_interfaces/msg/`
- C) In `/usr/include/ros2/`
- D) It doesn't generate headers, only Python files

**Q7.** Can a C++ node use a message type defined in a Python package?
- A) No, C++ and Python are incompatible
- B) No, message definitions must be in their own dedicated interface package (like `custom_interfaces`)
- C) Yes, directly
- D) Only with a converter tool

**Q8.** How does `colcon build` know to build `custom_interfaces` before `cpp_pkg`?
- A) Alphabetical order
- B) By reading `<depend>custom_interfaces</depend>` in `cpp_pkg/package.xml`
- C) By file modification date
- D) The user must specify the order manually

**Q9.** What is the CDR encoding used for?
- A) Compressing messages for storage
- B) Encrypting messages for security
- C) Serializing messages into a language-independent binary format for transport over DDS
- D) Converting messages to human-readable text

**Q10.** How many bytes does a `float64` occupy in memory?
- A) 4 bytes
- B) 8 bytes
- C) 16 bytes
- D) 64 bytes

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
