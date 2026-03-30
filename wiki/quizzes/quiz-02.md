# Quiz 2 — Topics & Pub/Sub

> **Related part:** [Part 2](../parts/02-topics-pub-sub.md)

---

## Questions

**Q1.** What communication pattern do topics use?

- A) Request/Response (synchronous)
- **B) Publish/Subscribe (asynchronous)** :white_check_mark:
- C) Peer-to-peer (direct connection)
- D) Polling (subscriber asks repeatedly)

**Q2.** What does the `10` in `create_publisher<String>("simple_topic", 10)` represent?

- A) The message size in bytes
- B) The number of subscribers allowed
- **C) The queue depth — how many messages to buffer before dropping** :white_check_mark:
- D) The publish rate in Hz

**Q3.** What happens when the subscriber's queue is full and a new message arrives (default QoS)?

- A) The publisher blocks and waits
- B) The new message is dropped
- **C) The oldest message is dropped to make room** :white_check_mark:
- D) The program crashes

**Q4.** What serialization format does ROS2 use to convert messages to bytes?

- A) JSON
- B) XML
- **C) CDR (Common Data Representation)** :white_check_mark:
- D) Protocol Buffers

**Q5.** Can a C++ publisher communicate with a Python subscriber on the same topic?

- A) No, they use different protocols
- **B) Yes, because they both serialize to the same CDR binary format** :white_check_mark:
- C) Only if they are on different machines
- D) Only with a bridge node in between

**Q6.** What is the difference between "Reliable" and "Best Effort" QoS?

- A) Reliable is faster, Best Effort guarantees delivery
- **B) Reliable guarantees delivery (re-sends if lost), Best Effort is fire-and-forget (faster)** :white_check_mark:
- C) They are the same thing
- D) Reliable only works in C++, Best Effort only in Python

**Q7.** In the subscriber callback `void callback_topic(const String::SharedPtr msg)`, what is `SharedPtr`?

- A) A raw C pointer to the message
- **B) A smart pointer that automatically frees memory when no longer used** :white_check_mark:
- C) A copy of the message
- D) A reference to the publisher

**Q8.** What does `std::bind(&SubscriberNode::callback_topic, this, _1)` do?

- A) Compiles the callback function
- **B) Creates a function object that binds `this` (the node) and leaves `_1` (the message) as a placeholder** :white_check_mark:
- C) Sends a message to the topic
- D) Destroys the subscription

**Q9.** How many publishers and subscribers can a single topic have?

- A) Exactly 1 publisher and 1 subscriber
- B) 1 publisher and many subscribers
- C) Many publishers and 1 subscriber
- **D) Many publishers and many subscribers (many-to-many)** :white_check_mark:

**Q10.** On the same machine, what transport does DDS prefer for performance?

- A) TCP
- B) UDP multicast
- **C) Shared memory (zero-copy or minimal copy)** :white_check_mark:
- D) HTTP

---

## My Answers

| Q | Answer | Result |
|---|--------|--------|
| 1 | B | :white_check_mark: |
| 2 | C | :white_check_mark: |
| 3 | C | :white_check_mark: |
| 4 | C | :white_check_mark: |
| 5 | B | :white_check_mark: |
| 6 | B | :white_check_mark: |
| 7 | B | :white_check_mark: |
| 8 | B | :white_check_mark: |
| 9 | D | :white_check_mark: |
| 10 | C | :white_check_mark: |

---

## Score: 10 / 10
