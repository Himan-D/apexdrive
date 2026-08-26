#include "apexdrive_hardware/apexdrive_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace apexdrive_hardware {

hardware_interface::CallbackReturn ApexDriveHardware::on_init(const hardware_interface::HardwareInfo& info) {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    std::string can_interface = info_.hardware_parameters.count("can_interface") ? 
                                info_.hardware_parameters.at("can_interface") : "can0";

    const size_t num_joints = info_.joints.size();
    hw_commands_position_.resize(num_joints, 0.0);
    hw_commands_velocity_.resize(num_joints, 0.0);
    hw_commands_effort_.resize(num_joints, 0.0);
    hw_commands_kp_.resize(num_joints, 40.0);
    hw_commands_kd_.resize(num_joints, 2.5);

    hw_states_position_.resize(num_joints, 0.0);
    hw_states_velocity_.resize(num_joints, 0.0);
    hw_states_effort_.resize(num_joints, 0.0);

    actuators_.reserve(num_joints);
    for (size_t i = 0; i < num_joints; ++i) {
        uint8_t node_id = static_cast<uint8_t>(0x10 + i);
        if (info_.joints[i].parameters.count("node_id")) {
            node_id = static_cast<uint8_t>(std::stoul(info_.joints[i].parameters.at("node_id"), nullptr, 0));
        }
        actuators_.push_back(std::make_unique<apexdrive::Actuator>(can_interface, node_id, /*mock_mode=*/false));
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ApexDriveHardware::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]);
        state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]);
        state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_states_effort_[i]);
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ApexDriveHardware::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_position_[i]);
        command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocity_[i]);
        command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_effort_[i]);
        command_interfaces.emplace_back(info_.joints[i].name, "stiffness_kp", &hw_commands_kp_[i]);
        command_interfaces.emplace_back(info_.joints[i].name, "damping_kd", &hw_commands_kd_[i]);
    }
    return command_interfaces;
}

hardware_interface::CallbackReturn ApexDriveHardware::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
    for (auto& actuator : actuators_) {
        actuator->Arm();
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ApexDriveHardware::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
    for (auto& actuator : actuators_) {
        actuator->Disarm();
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ApexDriveHardware::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
    for (size_t i = 0; i < actuators_.size(); ++i) {
        auto telemetry = actuators_[i]->GetState();
        hw_states_position_[i] = telemetry.position_rad;
        hw_states_velocity_[i] = telemetry.velocity_rad_s;
        hw_states_effort_[i] = telemetry.torque_nm;
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ApexDriveHardware::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
    for (size_t i = 0; i < actuators_.size(); ++i) {
        actuators_[i]->SetImpedance(
            static_cast<float>(hw_commands_position_[i]),
            static_cast<float>(hw_commands_velocity_[i]),
            static_cast<float>(hw_commands_kp_[i]),
            static_cast<float>(hw_commands_kd_[i]),
            static_cast<float>(hw_commands_effort_[i])
        );
    }
    return hardware_interface::return_type::OK;
}

} // namespace apexdrive_hardware

PLUGINLIB_EXPORT_CLASS(apexdrive_hardware::ApexDriveHardware, hardware_interface::SystemInterface)
