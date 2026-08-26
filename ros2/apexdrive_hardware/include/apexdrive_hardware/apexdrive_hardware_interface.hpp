#pragma once

#include <vector>
#include <string>
#include <memory>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/macros.hpp"

#include "apexdrive/host/actuator.hpp"

namespace apexdrive_hardware {

class ApexDriveHardware : public hardware_interface::SystemInterface {
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(ApexDriveHardware)

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
    
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    std::vector<std::unique_ptr<apexdrive::Actuator>> actuators_;
    
    // Command Buffers
    std::vector<double> hw_commands_position_;
    std::vector<double> hw_commands_velocity_;
    std::vector<double> hw_commands_effort_;
    std::vector<double> hw_commands_kp_;
    std::vector<double> hw_commands_kd_;

    // State Buffers
    std::vector<double> hw_states_position_;
    std::vector<double> hw_states_velocity_;
    std::vector<double> hw_states_effort_;
};

} // namespace apexdrive_hardware
