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
 * Sub-handler for multiple Franka FR3 arms (mirrors franka_multi_hardware_interface).
 *
 * hw_params:
 *  - robot_count      : number of robots (e.g. "2")
 *  - arm_id_<i>       : arm identifier for robot i (default: "fr3")
 *  - prefix_<i>       : arm prefix for robot i (e.g. "left", "right")
 *  - load_gripper_<i> : "true" | "false"
 *
 * State interfaces per robot:
 *  - {joint_name}/position, velocity, effort
 *
 * Xacro args:
 *  robot_count:=N arm_id_0:=... hand_0:=... control_mode_0:=... prefix_0:=... ...
 */
class FrankaMultiSubHandler : public SubHandlerBase
{
public:
    hardware_interface::CallbackReturn onInit(const hardware_interface::HardwareInfo & info) override;

    std::vector<hardware_interface::StateInterface>   exportStateInterfaces()   override;
    std::vector<hardware_interface::CommandInterface> exportCommandInterfaces() override;

    hardware_interface::return_type performCommandModeSwitch(
        const std::vector<std::string> & start_interfaces,
        const std::vector<std::string> & stop_interfaces) override;

    // Franka is the primary scene parameteriser — must load scene before other handlers.
    int scenePriority() const override { return 10; }

    // Not ready until ALL robots have had a command-mode controller activate.
    bool isReadyToLoadScene() const override;

    // Returns indexed xacro args for all robots.
    std::string getXacroArgs() const override;

    // Called after scene is loaded — maps joint names to mjModel indices for all robots.
    void onSceneLoaded() override;

    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    struct JointData {
        std::string name;
        int qpos_idx = -1, qvel_idx = -1, ctrl_idx = -1;
        double initial_pos = 0.0;
        double pos_state = 0.0, vel_state = 0.0, effort_state = 0.0;
        double pos_cmd   = 0.0, vel_cmd   = 0.0, effort_cmd   = 0.0;
        // Interface availability flags (populated in onInit)
        bool has_pos_state = false, has_vel_state = false, has_eff_state = false;
        bool has_pos_cmd   = false, has_vel_cmd   = false, has_eff_cmd   = false;
    };

    struct RobotContext {
        std::string arm_id;
        std::string arm_prefix;
        std::string name_stem;  // arm_prefix + "_" + arm_id, or arm_id if prefix empty
        std::string hand;       // "true" | "false"
        std::string control_mode;

        std::vector<JointData> joints;
        bool joints_mapped = false;

    };

    // Returns the index into robots_ for the given joint name, or robots_.size() if unmatched.
    size_t findRobotForJoint(const std::string & joint_name) const;

    static std::string interfaceTypeFromName(const std::string & full_name);
    static std::string normalizeBool(const std::string & value, const std::string & default_val);

    std::vector<RobotContext> robots_;
    hardware_interface::HardwareInfo hw_info_;
};

}  // namespace mujoco_ros_hardware
