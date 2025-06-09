#include "protobot_controller/protobot_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>


namespace protobot_controller
{

hardware_interface::CallbackReturn ProtobotInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{

    if (
        hardware_interface::SystemInterface::on_init(hardware_info) !=
        hardware_interface::CallbackReturn::SUCCESS)
      {
        return hardware_interface::CallbackReturn::ERROR;
      }

    for (const hardware_interface::ComponentInfo & joint : info_.joints)
    {
        // Checks for exactly one state and command interface on each joint
        if (joint.command_interfaces.size() != 1)
        {
            RCLCPP_FATAL(
            get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
            joint.name.c_str(), joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_FATAL(
            get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
            joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
            hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 1)
        {
            RCLCPP_FATAL(
            get_logger(), "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_FATAL(
            get_logger(), "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    try
    {
        sensor_map_["shoulder/position"] = std::make_unique<Sensor>(0x36);
        motor_map_["shoulder/position"] = std::make_unique<Motor>(0, 0);
    }
    catch(const std::out_of_range &e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("Protobot Interface"), "Unable to create Servo class instances, Aborting");
        return hardware_interface::CallbackReturn::FAILURE;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ProtobotInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
    // reset values always when configuring hardware
  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    set_state(name, 0.0);
  }
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, 0.0);
  }
  RCLCPP_INFO(rclcpp::get_logger("ProtobotInterface"), "Sucessfully configured command and state interfaces!");

  //open up i2c bus
  bus = open_bus(3);  // Opens I2C bus number 3

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ProtobotInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{    
    RCLCPP_INFO(rclcpp::get_logger("ProtobotInterface"), "Reading joint position values...");

    for (const auto & [name, descr] : joint_state_interfaces_)
    {
        auto sensor = sensor_map_.find(name);
        if (sensor != sensor_map_.end())
        {
            set_command(name, sensor->second->get_angle_degrees(bus));
        }
        else
        {
            RCLCPP_FATAL(rclcpp::get_logger("Protobot Interface"), "Unable to find %s sensor in sensor map", name.c_str());
            return hardware_interface::CallbackReturn::FAILURE;
        }
        auto motor = motor_map_.find(name);
        if (motor != motor_map_.end()) 
        {
            motor->second->activate();
        }
        else
        {
            RCLCPP_FATAL(rclcpp::get_logger("Protobot Interface"), "Unable to find %s motor in motor map", name.c_str());
            return hardware_interface::CallbackReturn::FAILURE;
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("ProtobotInterface"), "Hardware started, ready to take commands");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ProtobotInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{    
    RCLCPP_INFO(rclcpp::get_logger("ProtobotInterface"), "Stopping the robot hardware...");

    for (const auto & [name, descr] : joint_state_interfaces_)
    {
        auto motor = motor_map_.find(name);
        if (motor != motor_map_.end()) 
        {
            motor->second->deactivate();
        }
        else
        {
            RCLCPP_FATAL(rclcpp::get_logger("Protobot Interface"), "Unable to find %s motor in motor map", name.c_str());
            return hardware_interface::CallbackReturn::FAILURE;
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("ProtobotInterface"), "Hardware stopped");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ProtobotInterface::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{   
    for (const auto & [name, descr] : joint_state_interfaces_)
    {
        auto sensor = sensor_map_.find(name);
        if (sensor != sensor_map_.end())
        {
            auto new_value = sensor->second->get_angle_radians(bus);
            set_state(name, new_value);
            // RCLCPP_INFO(rclcpp::get_logger("ProtobotInterface"), "Joint '%s' angle: %f radians", name.c_str(), new_value);
        }
        else
        {
            RCLCPP_FATAL(rclcpp::get_logger("Protobot Interface"), "Unable to find %s motor in sensor map", name.c_str());
            return hardware_interface::return_type::ERROR;
        }
        
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ProtobotInterface::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{   

    // Check if all current commands match the previous ones
    bool all_match = true;
    for (const auto & [name, descr] : joint_command_interfaces_) {
        double current = get_command(name);
        // If the values differ or joint prev_position_commands_ hasn't been set yet (first run)
        if (current != prev_position_commands_[name] || prev_position_commands_.find(name) == prev_position_commands_.end()) {
            all_match = false;
            break;
        }
    }
    if (all_match) {
        return hardware_interface::return_type::OK;
    }

    // Update previous commands
    for (const auto & [name, descr] : joint_command_interfaces_) {
         auto motor = motor_map_.find(name);
        if (motor != motor_map_.end())
        {
            double new_pos = static_cast<int>(get_command(name) * (180 / M_PI));

            motor->second->set_speed(new_pos);
            prev_position_commands_[name] = new_pos;
        }
        else
        {
            RCLCPP_FATAL(rclcpp::get_logger("Protobot Interface"), "Unable to find %s motor in motor map", name.c_str());
            return hardware_interface::return_type::ERROR;
        }
    }

    return hardware_interface::return_type::OK;
}


} // namespace protobot_controller

PLUGINLIB_EXPORT_CLASS(protobot_controller::ProtobotInterface, hardware_interface::SystemInterface);