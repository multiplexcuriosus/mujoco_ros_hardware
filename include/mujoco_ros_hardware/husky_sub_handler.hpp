#pragma once

#include <string>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"

#include "mujoco_ros_hardware/sub_handler_base.hpp"

namespace mujoco_ros_hardware
{

/**
 * Sub-handler for the Clearpath Husky mobile base.
 *
 * Key responsibilities:
 *  - Reads husky-specific hw_params (wheel_diameter, max_speed) from HardwareInfo.
 *  - Joint mapping is done lazily on first read()/write() once the scene is loaded
 *    (the scene is loaded by FrankaSubHandler at controller activation time).
 *  - Applies left/right velocity commands; clamps to max_speed.
 */
class HuskySubHandler : public SubHandlerBase
{
public:
    hardware_interface::CallbackReturn onInit(const hardware_interface::HardwareInfo & info) override;

    std::vector<hardware_interface::StateInterface>   exportStateInterfaces()   override;
    std::vector<hardware_interface::CommandInterface> exportCommandInterfaces() override;

    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    // Lazy joint mapping: called on first read/write after scene is loaded
    void mapJoints();

    // ---- Per-joint state/command buffers ----
    struct JointData {
        std::string name;
        int qpos_idx = -1;
        int qvel_idx = -1;
        int ctrl_idx = -1;
        double pos_state = 0.0, vel_state = 0.0;
        double vel_cmd   = 0.0;
    };
    std::vector<JointData> joints_;
    bool joints_mapped_ = false;

    // ---- Husky hw_params ----
    double wheel_radius_    = 0.1651;
    double max_speed_radps_ = 6.06;

    // ---- Stored for export methods ----
    hardware_interface::HardwareInfo hw_info_;
};

}  // namespace mujoco_ros_hardware
