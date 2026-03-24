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
 * Sub-handler for the Franka FR3 arm.
 *
 * hw_params:
 *  - arm_id       : arm identifier, e.g. "fr3"
 *  - prefix       : arm prefix, e.g. "left" (optional)
 *  - load_gripper : "true" | "false"
 *
 * State interfaces exported:
 *  - {joint_name}/position, velocity, effort
 */
class FrankaSubHandler : public SubHandlerBase
{
public:
    hardware_interface::CallbackReturn onInit(const hardware_interface::HardwareInfo & info) override;

    std::vector<hardware_interface::StateInterface>   exportStateInterfaces()   override;
    std::vector<hardware_interface::CommandInterface> exportCommandInterfaces() override;

    hardware_interface::return_type performCommandModeSwitch(const std::vector<std::string> & start_interfaces, const std::vector<std::string> & stop_interfaces) override;

    // Franka is the primary scene parameteriser (supplies control_mode), so it
    // must load the scene before other handlers.
    int scenePriority() const override { return 10; }

    // Not ready until a command-mode controller has activated and set control_mode_.
    bool isReadyToLoadScene() const override { return !control_mode_.empty(); }

    // Returns "arm_id:=<> hand:=<> control_mode:=<>" for MujocoHardwareInterface to run xacro.
    std::string getXacroArgs() const override;

    // Called after scene is loaded — maps joint names to mjModel indices.
    void onSceneLoaded() override;

    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    void mapJoints();

    static std::string interfaceTypeFromName(const std::string & full_name);
    static std::string normalizeBool(const std::string & value, const std::string & default_val);

    // ---- Real hardware params ----
    std::string arm_id_;
    std::string arm_prefix_;
    std::string name_stem_;  // arm_prefix_ + "_" + arm_id_  (or arm_id_ if prefix empty)
    std::string hand_;   // "true" | "false"

    std::string control_mode_;  // "position" | "velocity" | "effort"

    // ---- Per-joint state/command buffers ----
    struct JointData {
        std::string name;
        int qpos_idx = -1;
        int qvel_idx = -1;
        int ctrl_idx = -1;
        double initial_pos = 0.0;
        double pos_state = 0.0, vel_state = 0.0, effort_state = 0.0;
        double pos_cmd   = 0.0, vel_cmd   = 0.0, effort_cmd   = 0.0;
    };
    std::vector<JointData> joints_;
    bool joints_mapped_ = false;

    // ---- Stored for export methods ----
    hardware_interface::HardwareInfo hw_info_;
};

}  // namespace mujoco_ros_hardware
