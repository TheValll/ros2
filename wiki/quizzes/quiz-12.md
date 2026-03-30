# Quiz 12 — Hardware Driver: LX-225

> **Related part:** [Part 12](../parts/12-lx225-driver.md)

---

## Questions

**Q1.** What does UART stand for?
- A) Universal Automatic Robot Transfer
- B) Unified Application Runtime Toolkit
- C) USB Async Read/Transmit
- D) Universal Asynchronous Receiver/Transmitter

**Q2.** What does "8N1" mean in UART configuration?
- A) 8 data bits, No parity, 1 stop bit
- B) 8 servos, No errors, 1 bus
- C) 8 baudrate, No flow control, 1 port
- D) 8 bytes, New line, 1 second

**Q3.** At 115200 baud with 8N1, how long does it take to send one byte?
- A) 1 microsecond
- B) 1 millisecond
- C) About 86.8 microseconds (10 bits / 115200 bits per second)
- D) 1 second

**Q4.** What does `tcflush(handle, TCIFLUSH)` do?
- A) Sends all buffered data
- B) Clears the receive (RX) buffer, discarding any stale unread bytes
- C) Closes the serial port
- D) Changes the baudrate

**Q5.** What is the position range of the LX-225 servo?
- A) -180 to +180
- B) 0 to 360
- C) -1000 to +1000
- D) 0 to 1000 (where 500 is the center/default position)

**Q6.** What does `boost::asio::serial_port` manage?
- A) A file descriptor for the serial port, with OS-level TX/RX buffers
- B) A network TCP socket
- C) A USB device directly
- D) A Bluetooth connection

**Q7.** Why does `init()` sleep for 3 seconds?
- A) To save power
- B) Because UART requires a delay
- C) To wait for the servo's internal microcontroller to boot up after power-on
- D) To synchronize clocks

**Q8.** In the command `"bus_servo.run(6,800,1000)\r\n"`, what do the three numbers mean?
- A) servo_id=6, target_position=800, duration_ms=1000
- B) baudrate, port, timeout
- C) x, y, z coordinates
- D) speed, acceleration, torque

**Q9.** When reading the servo position, why does the driver skip lines containing "bus_servo"?
- A) Because those lines are errors
- B) Because those lines are encrypted
- C) Because they contain binary data
- D) Because the servo echoes back the command before sending the response — the echo must be skipped

**Q10.** What hardware chip converts between USB and UART signals?
- A) A GPU
- B) A CH340 or CP2102 USB-to-UART converter chip
- C) An ARM processor
- D) A WiFi module

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
