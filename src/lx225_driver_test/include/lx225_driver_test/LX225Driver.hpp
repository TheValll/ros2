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
    LX225Driver(const std::string& port, int baudrate, int id = 6) //In my case my servo has the id 6 !
        : port_name(port), baudrate(baudrate), servo_id(id), serial_(io) 
    {
        try {
            open_LX225();
        }
        catch(const std::exception& e) {
            std::cerr << "Error configuring serial port: " << e.what() << '\n';
        }

        std::this_thread::sleep_for(std::chrono::seconds(3));
    }

    ~LX225Driver() {
        close_LX225();
    }

    void open_LX225() {
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

            std::cout << "Connection opened : " << port_name << " a " << baudrate << " bauds." << std::endl;
            default_position();
        } catch (const boost::system::system_error& e) {
            std::cerr << "Error can't open the port : " << port_name << " : " << e.what() << std::endl;
            throw; 
        }
    }

    void close_LX225() {
        if (serial_.is_open()) {
            default_position();
            serial_.close();
            std::cout << "Serial port closed" << std::endl;
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
    void setBaudrate(int baudrate) { this->baudrate = baudrate; }
    void setServoId(int id) { servo_id = id; }
    std::string getPort() const { return port_name; }
    int getBaudrate() const { return baudrate; }
    int getServoId() const { return servo_id; }
};

#endif // LX225DRIVER_HPP
