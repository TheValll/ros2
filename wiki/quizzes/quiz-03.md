# Quiz 3 — Services: Request/Reply

> **Related part:** [Part 3](../parts/03-services.md)

---

## Questions

**Q1.** What communication pattern do services use?

- **A) Request/Response (two-way)** :white_check_mark:
- B) Publish/Subscribe (one-way)
- C) Broadcast (many-to-many)
- D) Streaming (continuous)

**Q2.** In a `.srv` file, what does the `---` separator mean?

- A) End of file
- B) A comment
- **C) It separates the Request fields (above) from the Response fields (below)** :white_check_mark:
- D) It separates two different services

**Q3.** How many servers can exist for a single service name?

- A) Unlimited
- B) Exactly two (primary and backup)
- C) As many as there are clients
- **D) Exactly one — multiple servers on the same name would cause conflicts** :white_check_mark:

**Q4.** Why does the client use `async_send_request` instead of a blocking call?

- A) Because blocking calls are not implemented in ROS2
- B) Because async is always faster
- **C) Because a blocking call inside `spin()` would cause a deadlock — the thread can't process the response while it's blocked waiting for it** :white_check_mark:
- D) Because the server requires async requests

**Q5.** What is a `Future` in the context of the client?

- A) A prediction of what the server will return
- **B) A container for a value that will be available later, when the response arrives** :white_check_mark:
- C) A timeout value
- D) The server's callback function

**Q6.** Under the hood, a ROS2 service uses:

- **A) Two hidden DDS topics (one for request, one for response)** :white_check_mark:
- B) A single DDS topic
- C) HTTP REST calls
- D) Unix sockets

**Q7.** In the server callback `callback_server(req, res)`, who allocates the Response object?

- A) The programmer manually
- B) The client creates it and sends it empty
- C) DDS creates it on the network
- **D) ROS2/rclcpp creates it before calling the callback** :white_check_mark:

**Q8.** What does `client_->wait_for_service(1s)` do?

- A) Sends a request and waits 1 second for the response
- **B) Blocks until the service server is discovered on the network, checking every 1 second** :white_check_mark:
- C) Creates the service with a 1-second timeout
- D) Waits 1 second then crashes if no server found

**Q9.** In the Python client, what does `partial(self.callback_response, req)` achieve?

- A) It sends the request immediately
- B) It blocks until the response arrives
- **C) It creates a new function that "remembers" the original request, so the callback can log what was asked** :white_check_mark:
- D) It cancels the request

**Q10.** When should you use a Service instead of a Topic?

- **A) For one-time computations or queries where you need a specific answer back** :white_check_mark:
- B) For continuous high-frequency data streams
- C) For sensor data like camera or lidar
- D) For broadcasting to multiple receivers

---

## My Answers

| Q | Answer | Result |
|---|--------|--------|
| 1 | A | :white_check_mark: |
| 2 | C | :white_check_mark: |
| 3 | D | :white_check_mark: |
| 4 | C | :white_check_mark: |
| 5 | B | :white_check_mark: |
| 6 | A | :white_check_mark: |
| 7 | B | :x: → D |
| 8 | B | :white_check_mark: |
| 9 | C | :white_check_mark: |
| 10 | A | :white_check_mark: |

---

## Score: 9 / 10
