# Quiz 3 — Services: Request/Reply

> **Related part:** [Part 3](../parts/03-services.md)

---

## Questions

**Q1.** What communication pattern do services use?
- A) Publish/Subscribe (one-way)
- B) Request/Response (two-way)
- C) Broadcast (many-to-many)
- D) Streaming (continuous)

**Q2.** In a `.srv` file, what does the `---` separator mean?
- A) End of file
- B) A comment
- C) It separates the Request fields (above) from the Response fields (below)
- D) It separates two different services

**Q3.** How many servers can exist for a single service name?
- A) Unlimited
- B) Exactly one — multiple servers on the same name would cause conflicts
- C) Exactly two (primary and backup)
- D) As many as there are clients

**Q4.** Why does the client use `async_send_request` instead of a blocking call?
- A) Because blocking calls are not implemented in ROS2
- B) Because a blocking call inside `spin()` would cause a deadlock — the thread can't process the response while it's blocked waiting for it
- C) Because async is always faster
- D) Because the server requires async requests

**Q5.** What is a `Future` in the context of the client?
- A) A prediction of what the server will return
- B) A container for a value that will be available later, when the response arrives
- C) A timeout value
- D) The server's callback function

**Q6.** Under the hood, a ROS2 service uses:
- A) A single DDS topic
- B) Two hidden DDS topics (one for request, one for response)
- C) HTTP REST calls
- D) Unix sockets

**Q7.** In the server callback `callback_server(req, res)`, who allocates the Response object?
- A) The programmer manually
- B) ROS2/rclcpp creates it before calling the callback
- C) The client creates it and sends it empty
- D) DDS creates it on the network

**Q8.** What does `client_->wait_for_service(1s)` do?
- A) Sends a request and waits 1 second for the response
- B) Blocks until the service server is discovered on the network, checking every 1 second
- C) Creates the service with a 1-second timeout
- D) Waits 1 second then crashes if no server found

**Q9.** In the Python client, what does `partial(self.callback_response, req)` achieve?
- A) It sends the request immediately
- B) It creates a new function that "remembers" the original request, so the callback can log what was asked
- C) It blocks until the response arrives
- D) It cancels the request

**Q10.** When should you use a Service instead of a Topic?
- A) For continuous high-frequency data streams
- B) For one-time computations or queries where you need a specific answer back
- C) For sensor data like camera or lidar
- D) For broadcasting to multiple receivers

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
