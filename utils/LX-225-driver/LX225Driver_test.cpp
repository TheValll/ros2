/*
Installation and compilation instructions:
sudo apt-get update
sudo apt-get install libboost-all-dev
sudo chmod 666 /dev/ttyUSB0
g++ LX225Driver_test.cpp -o LX225Driver_test -lboost_system -lpthread
./LX225Driver_test
*/

#include <iostream>
#include <thread>
#include <chrono>
#include "LX225Driver.hpp"

int main() {
    std::string port = "/dev/ttyUSB0";
    int baudrate = 115200;
    int servo_id = 6;

    std::cout << "Initializing on port " << port << "..." << std::endl;

    LX225Driver servo(port, baudrate, servo_id);

    if (servo.init() != 0) {
        std::cerr << "Fatal error: failed to initialize the driver." << std::endl;
        return 1;
    }

    int target_position = 800;
    int duration_ms = 1000;

    std::cout << "Moving servo ID " << servo.getServoId()
              << " to position " << target_position
              << " in " << duration_ms << " ms..." << std::endl;

    servo.set_position(target_position, duration_ms);
    std::this_thread::sleep_for(std::chrono::milliseconds(duration_ms + 500));

    int current_pos = servo.get_command_servo_position();
    if (current_pos >= 0) {
        std::cout << "Current servo position: " << current_pos << std::endl;
    }

    std::cout << "Closing port. The servo should return to its default position." << std::endl;
    servo.close_LX225();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::cout << "--- End of test ---" << std::endl;
    return 0;
}
