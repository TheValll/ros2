#include "rclcpp/rclcpp.hpp"
#include "lx225_driver_test/LX225Driver.hpp"
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    std::string port = "/dev/ttyUSB0"; // In my case 
    int baudrate = 115200; // In my case
    int servo_id = 6; // In my case

    try {
        std::cout << "Initializing on port " << port << "..." << std::endl;
        LX225Driver servo(port, baudrate, servo_id);

        int target_position = 800;
        int duration_ms = 1000;
        
        std::cout << "Moving servo ID " << servo.getServoId() 
                  << " to position " << target_position 
                  << " in " << duration_ms << " ms..." << std::endl;
                  
        servo.set_position(target_position, duration_ms);

        std::this_thread::sleep_for(std::chrono::milliseconds(duration_ms + 500));

        std::cout << "Closing port. The servo should return to its default position." << std::endl;
        servo.close_LX225();

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    } catch (const std::exception& e) {
        std::cerr << "Fatal error during execution: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "--- End of test ---" << std::endl;
    return 0;
}
