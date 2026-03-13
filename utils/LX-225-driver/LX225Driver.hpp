#ifndef LX225DRIVER_HPP
#define LX225DRIVER_HPP
#include <iostream>
#include <string>
#include <boost/asio.hpp>
#include <thread>
#include <chrono>

class LX225Driver {
private:
    std::string port_name;
    int baudrate;
    int servo_id;
    boost::asio::io_context io;
    boost::asio::serial_port serial_;

    void send_command(const std::string& command) {
        if (!serial_.is_open()) {
            std::cerr << "Error: serial port not open." << std::endl;
            return;
        }
        boost::system::error_code ec;
        boost::asio::write(serial_, boost::asio::buffer(command), ec);
        if (ec) {
            std::cerr << "Error writing to serial port: " << ec.message() << std::endl;
        }
    }

public:
    LX225Driver(const std::string& port, int baudrate, int id = 6)
        : port_name(port), baudrate(baudrate), servo_id(id), serial_(io)
    {}

    ~LX225Driver() {
        close_LX225();
    }

    int init() {
        if (serial_.is_open()) {
            serial_.close();
        }
        try {
            serial_.open(port_name);
            serial_.set_option(boost::asio::serial_port_base::baud_rate(baudrate));
            serial_.set_option(boost::asio::serial_port_base::character_size(8));
            serial_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
            serial_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
            serial_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
            std::cout << "Connection opened : " << port_name << " at " << baudrate << " bauds." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(3));
            default_position();
            return 0;
        } catch (const boost::system::system_error& e) {
            std::cerr << "Error can't open the port : " << port_name << " : " << e.what() << std::endl;
            return -1;
        }
    }

    void close_LX225() {
        if (serial_.is_open()) {
            default_position();
            serial_.close();
            std::cout << "Serial port closed" << std::endl;
        }
    }

    int get_command_servo_position() 
    {
        boost::asio::serial_port::native_handle_type handle = serial_.native_handle();
        tcflush(handle, TCIFLUSH);

        std::string cmd = "bus_servo.get_position(" + std::to_string(servo_id) + ")\r\n";
        send_command(cmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        try {
            boost::asio::streambuf buf;
            boost::asio::read_until(serial_, buf, "\n");

            std::istream is(&buf);
            std::string line;

            while (std::getline(is, line)) {
                if (!line.empty() && line.back() == '\r') {
                    line.pop_back();
                }
                if (line.find("bus_servo") == std::string::npos) {
                    break;
                }
            }

            std::string digits;
            for (char c : line) {
                if (std::isdigit(c)) {
                    digits += c;
                }
            }

            if (digits.empty()) {
                std::cerr << "No digits found in response." << std::endl;
                return -1;
            }

            return std::stoi(digits);

        } catch (const boost::system::system_error& e) {
            std::cerr << "Timeout or error reading servo position: " << e.what() << std::endl;
            return -1;
        } catch (const std::invalid_argument& e) {
            std::cerr << "Invalid response from servo: " << e.what() << std::endl;
            return -1;
        }
    }

    void set_position(int position, int time_ms = 1000) {
        if (position < 0) position = 0;
        if (position > 1000) position = 1000;
        std::string cmd = "bus_servo.run(" + std::to_string(servo_id) + "," +
                          std::to_string(position) + "," + std::to_string(time_ms) + ")\r\n";
        send_command(cmd);
    }

    void default_position() {
        set_position(500, 1000);
    }

    void setPort(const std::string& port) { port_name = port; }
    void setBaudrate(int baud) { baudrate = baud; }
    void setServoId(int id) { servo_id = id; }
    std::string getPort() const { return port_name; }
    int getBaudrate() const { return baudrate; }
    int getServoId() const { return servo_id; }
};

#endif // LX225DRIVER_HPP
