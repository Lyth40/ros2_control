#ifndef MOBILE_BASE_HARDWARE_INTERFACE_HPP
#define MOBILE_BASE_HARDWARE_INTERFACE_HPP

#include "rclcpp/rclcpp.hpp"
#include"hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include"my_robot_hardware/motor_driver.hpp"
#include "my_robot_hardware/config.hpp"
#include "my_robot_hardware/wheel.hpp"

namespace mobile_base_hardware{

    class MobileBaseHardwareInterface : public hardware_interface::SystemInterface
    {
        public:
            hardware_interface::CallbackReturn
                on_configure(const rclcpp_lifecycle::State & previous_state) override;

            hardware_interface::CallbackReturn
                on_activate(const rclcpp_lifecycle::State & previous_state) override;

            hardware_interface::CallbackReturn
                on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
                
            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
            

            hardware_interface::CallbackReturn
                on_init(const hardware_interface::HardwareInfo & info) override;

            hardware_interface::return_type
                read(const rclcpp::Time & time , const rclcpp::Duration & period) override;
            
            hardware_interface::return_type
                write(const rclcpp::Time & time , const rclcpp::Duration & period) override;

        private:
            MotorDriver driver_ ; 
            Config cfg_;

            Wheel l_wheel_;
            Wheel r_wheel_;

          
            std::chrono::time_point<std::chrono::system_clock> time_;

    } ;
}

#endif