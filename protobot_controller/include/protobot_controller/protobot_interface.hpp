#ifndef PROTOBOT_INTERFACE_H
#define PROTOBOT_INTERFACE_H

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include "protobot_controller/pwm_comms.hpp"

#include <memory>
#include <string>
#include <vector>

namespace protobot_controller 
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ProtobotInterface : public hardware_interface::SystemInterface 
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(ProtobotInterface)

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
    std::vector<std::string> position_interfaces_ = {
        "shoulder_joint/position",
    };
    std::unordered_map<std::string, double> prev_position_commands_;
    std::unique_ptr<Motor> ShoulderMotor;
};
}
#endif