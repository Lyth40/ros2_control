#include"my_robot_hardware/mobile_base_hardware_interface.hpp"


namespace mobile_base_hardware{


    hardware_interface::CallbackReturn
                MobileBaseHardwareInterface::on_init
                (const hardware_interface::HardwareInfo & info)
                {
                    if(hardware_interface::SystemInterface::on_init(info) !=
                        hardware_interface::CallbackReturn::SUCCESS)
                        {
                            return hardware_interface::CallbackReturn::ERROR;
                        }
                    info_=info;

                    try {
                            cfg_.left_wheel_name = info_.hardware_parameters.at("left_wheel_name");
                            cfg_.right_wheel_name = info_.hardware_parameters.at("right_wheel_name");
                            cfg_.loop_rate = std::stof(info_.hardware_parameters.at("loop_rate"));
                            cfg_.device = info_.hardware_parameters.at("device");
                            cfg_.baud_rate = std::stoi(info_.hardware_parameters.at("baud_rate"));
                            cfg_.timeout = std::stoi(info_.hardware_parameters.at("timeout"));
                            cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters.at("enc_counts_per_rev"));
                        }
                        catch (const std::out_of_range& e) {
                            RCLCPP_ERROR(rclcpp::get_logger("MobileBaseHardwareInterface"),
                                        "Missing hardware parameter: %s", e.what());
                            return hardware_interface::CallbackReturn::ERROR;
                        }
                        catch (const std::invalid_argument& e) {
                            RCLCPP_ERROR(rclcpp::get_logger("MobileBaseHardwareInterface"),
                                        "Invalid parameter format: %s", e.what());
                            return hardware_interface::CallbackReturn::ERROR;
                        }
                        catch (const std::exception& e) {
                            RCLCPP_ERROR(rclcpp::get_logger("MobileBaseHardwareInterface"),
                                        "Unexpected exception during on_init: %s", e.what());
                            return hardware_interface::CallbackReturn::ERROR;
                        }
                        
                    l_wheel_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
                    r_wheel_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);

                    driver_.setup(cfg_.device, cfg_.baud_rate, cfg_.timeout); 

                    return hardware_interface::CallbackReturn::SUCCESS;
                }

    hardware_interface::CallbackReturn 
               MobileBaseHardwareInterface::on_configure
               (const rclcpp_lifecycle::State & previous_state)
               {
                    if(!driver_.connected()){
                        return hardware_interface::CallbackReturn::ERROR;
                    }
                    return hardware_interface::CallbackReturn::SUCCESS;
               }

    hardware_interface::CallbackReturn
                MobileBaseHardwareInterface::on_activate
                (const rclcpp_lifecycle::State & previous_state)
                {   
                    (void) previous_state ;
                    
                    driver_.sendEmptyMsg();
                    
                    return hardware_interface::CallbackReturn::SUCCESS;
                }

    hardware_interface::CallbackReturn
                MobileBaseHardwareInterface::on_deactivate
                (const rclcpp_lifecycle::State & previous_state)
                {
                    
                    return hardware_interface::CallbackReturn::SUCCESS;

                }                

    hardware_interface::return_type
                MobileBaseHardwareInterface::read
                (const rclcpp::Time & time , const rclcpp::Duration & period)
                {
                    (void)time;
                    auto new_time = std::chrono::system_clock::now();
                    std::chrono::duration<double> diff = new_time - time_;
                    double deltaSeconds = diff.count();
                    time_ = new_time;
                    if (!driver_.connected())
                    {
                        return hardware_interface::return_type::ERROR;
                    }

                    driver_.readEncoderValues(l_wheel_.enc, r_wheel_.enc);

                    double pos_prev = l_wheel_.pos;
                    l_wheel_.pos = l_wheel_.calcEncAngle();
                    l_wheel_.vel = (l_wheel_.pos - pos_prev) / deltaSeconds;

                    pos_prev = r_wheel_.pos;
                    r_wheel_.pos = r_wheel_.calcEncAngle();
                    r_wheel_.vel = (r_wheel_.pos - pos_prev) / deltaSeconds;
                    

                    return hardware_interface::return_type::OK;
                }
                
    hardware_interface::return_type
                MobileBaseHardwareInterface::write(const rclcpp::Time & time , const rclcpp::Duration & period) 
                {   (void)time;
                    if (!driver_.connected())
                    {
                        return hardware_interface::return_type::ERROR;
                    }

                    driver_.setMotorValues(l_wheel_.cmd / l_wheel_.rads_per_count / cfg_.loop_rate,
                                         r_wheel_.cmd / r_wheel_.rads_per_count / cfg_.loop_rate);
                                        
                    RCLCPP_INFO(rclcpp::get_logger("MobileBaseHardwareInterface"), 
                        "Cmd L: %.3f, Cmd R: %.3f", 
                        l_wheel_.cmd, r_wheel_.cmd);

                    return hardware_interface::return_type::OK;

                }


    std::vector<hardware_interface::StateInterface> MobileBaseHardwareInterface::export_state_interfaces()
    {
        // We need to set up a position and a velocity interface for each wheel

        std::vector<hardware_interface::StateInterface> state_interfaces;
        state_interfaces.emplace_back(hardware_interface::StateInterface(l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.vel));
        state_interfaces.emplace_back(hardware_interface::StateInterface(l_wheel_.name, hardware_interface::HW_IF_POSITION, &l_wheel_.pos));
        state_interfaces.emplace_back(hardware_interface::StateInterface(r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.vel));
        state_interfaces.emplace_back(hardware_interface::StateInterface(r_wheel_.name, hardware_interface::HW_IF_POSITION, &r_wheel_.pos));

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> MobileBaseHardwareInterface::export_command_interfaces()
    {
        // We need to set up a velocity command interface for each wheel

        std::vector<hardware_interface::CommandInterface> command_interfaces;

        command_interfaces.emplace_back(hardware_interface::CommandInterface(l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.cmd));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.cmd));

        return command_interfaces;
    }
}

#include"pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(mobile_base_hardware::MobileBaseHardwareInterface,hardware_interface::SystemInterface)