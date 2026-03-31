# Quiz 4 — Custom Interfaces

> **Related part:** [Part 4](../parts/04-custom-interfaces.md)

---

## Questions

**Q1.** What is the purpose of a `.msg` file?

- A) It contains the C++ code for a node
- B) It stores log messages
- **C) It defines the structure (fields and types) of a custom message, used by code generators** :white_check_mark:
- D) It is a configuration file for launch

**Q2.** In a `.msg` file, `float64 a` means:

- **A) A field named "a" of type double-precision floating point (8 bytes, IEEE 754)** :white_check_mark:
- B) A variable named "a" of type 64-byte float
- C) A function called "a" that returns a float
- D) A constant with value 64

**Q3.** What does `rosidl_generate_interfaces` in CMakeLists.txt do?

- A) Creates the package.xml file
- B) Compiles the node executable
- C) Installs the package
- **D) Triggers code generation: converts .msg/.srv files into C, C++, Python code and DDS type support** :white_check_mark:

**Q4.** Why does the compiler add padding bytes between a `bool` (1 byte) and a `float64` (8 bytes)?

- A) To make the message bigger for DDS
- **B) For memory alignment — CPUs read faster when data starts at addresses divisible by its size** :white_check_mark:
- C) To add error-checking bytes
- D) It's a bug in the compiler

**Q5.** What intermediate language is a `.msg` file converted to before code generation?

- **A) IDL (Interface Definition Language) — the DDS standard** :white_check_mark:
- B) JSON
- C) YAML
- D) Protocol Buffers

**Q6.** After building `custom_interfaces`, where does the generated C++ header end up?

- A) In the source directory next to the .msg file
- B) It doesn't generate headers, only Python files
- **C) In `install/custom_interfaces/include/custom_interfaces/msg/`** :white_check_mark:
- D) In `/usr/include/ros2/`

**Q7.** Can a C++ node use a message type defined in a Python package?

- A) No, C++ and Python are incompatible
- B) Yes, directly
- C) Only with a converter tool
- **D) No, message definitions must be in their own dedicated interface package (like `custom_interfaces`)** :white_check_mark:

**Q8.** How does `colcon build` know to build `custom_interfaces` before `cpp_pkg`?

- **A) By reading `<depend>custom_interfaces</depend>` in `cpp_pkg/package.xml`** :white_check_mark:
- B) Alphabetical order
- C) By file modification date
- D) The user must specify the order manually

**Q9.** What is the CDR encoding used for?

- A) Compressing messages for storage
- **B) Serializing messages into a language-independent binary format for transport over DDS** :white_check_mark:
- C) Encrypting messages for security
- D) Converting messages to human-readable text

**Q10.** How many bytes does a `float64` occupy in memory?

- A) 4 bytes
- B) 64 bytes
- **C) 8 bytes** :white_check_mark:
- D) 16 bytes

---

## My Answers

| Q | Answer | Result |
|---|--------|--------|
| 1 | C | :white_check_mark: |
| 2 | A | :white_check_mark: |
| 3 | D | :white_check_mark: |
| 4 | B | :white_check_mark: |
| 5 | A | :white_check_mark: |
| 6 | C | :white_check_mark: |
| 7 | B | :x: → D |
| 8 | A | :white_check_mark: |
| 9 | B | :white_check_mark: |
| 10 | C | :white_check_mark: |

---

## Score: 9 / 10
