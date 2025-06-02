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
        ShoulderMotor = std::make_unique<Motor>(0, 0);
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

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ProtobotInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{    
    RCLCPP_INFO(rclcpp::get_logger("ProtobotInterface"), "Starting the robot hardware...");
    // command and state should be equal when starting
    for (const auto & [name, descr] : joint_state_interfaces_)
    {
        set_command(name, get_state(name));
    }

    ShoulderMotor->activate();

    RCLCPP_INFO(rclcpp::get_logger("ProtobotInterface"), "Hardware started, ready to take commands");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ProtobotInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{    
    RCLCPP_INFO(rclcpp::get_logger("ProtobotInterface"), "Stopping the robot hardware...");
    
    ShoulderMotor->deactivate();

    RCLCPP_INFO(rclcpp::get_logger("ProtobotInterface"), "Hardware stopped");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ProtobotInterface::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{   
    for (const auto & [name, descr] : joint_state_interfaces_)
    {
        auto new_value = get_command(name);
        set_state(name, new_value);
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ProtobotInterface::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{   

    // Check if all current commands match the previous ones
    bool all_match = true;
    // for (const auto& name : position_interfaces_) {
    for (const auto & [name, descr] : joint_command_interfaces_) {
        double current = get_command(name);
        // If the interface isn't in prev_position_commands_ yet (first run) or values differ
        if (prev_position_commands_.find(name) == prev_position_commands_.end() || current != prev_position_commands_[name]) {
            all_match = false;
            break;
        }
    }
    if (all_match) {
        return hardware_interface::return_type::OK;
    }

    // Set the new command
    // Shoulder joint
    double shoulder_pos = get_command(position_interfaces_.at(0));
    int shoulder = static_cast<int>(180 - (shoulder_pos + (M_PI/2)) * (180 / M_PI));

    ShoulderMotor->set_angle(shoulder);

    // Update previous commands
    // for (const auto& name : position_interfaces_) {
    for (const auto & [name, descr] : joint_command_interfaces_) {
        prev_position_commands_[name] = get_command(name);
    }

    return hardware_interface::return_type::OK;
}


} // namespace protobot_controller

PLUGINLIB_EXPORT_CLASS(protobot_controller::ProtobotInterface, hardware_interface::SystemInterface);