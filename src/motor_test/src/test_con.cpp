#include "rclcpp/rclcpp.hpp"
#include "motor_test/motor_driver.hpp"
#include <thread>
#include <iostream>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    MotorDriver m;
    m.setup("/dev/ttyUSB0", 57600, 1000);

    if (m.connected())
        std::cout << "port is opened" << std::endl;
    else
        std::cerr << "failed to open port" << std::endl;
    
    m.setMotorValues(100,100);

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
