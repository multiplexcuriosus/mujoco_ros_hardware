#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>
#include <unistd.h>
#include <fstream>
#include <sstream>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "mujoco_ros_hardware/sub_handler_base.hpp"

namespace mujoco_ros_hardware
{

/**
 * Single ros2_control hardware plugin used for all robot types.
 *
 * URDF usage:
 *   <hardware>
 *     <plugin>mujoco_ros_hardware/MujocoHardwareInterface</plugin>
 *     <param name="robot_type">franka</param>   <!-- or "husky" -->
 *     <!-- robot-specific params follow (arm_id, load_gripper, wheel_diameter, ...) -->
 *   </hardware>
 *
 * The MuJoCo scene xacro path and base args are read from the controller_manager
 *
 * Scene loading flow (perform_command_mode_switch, first plugin to run when !isSceneLoaded):
 *   1. sub_handler->performCommandModeSwitch()       — determines runtime state (e.g. control_mode)
 *   2. sub_handler->getXacroArgs()                   — returns handler-specific xacro args
 *   3. Combined args = Singleton::xacroBaseArgs() + handler args
 *   4. runXacro(Singleton::xacroPath(), combined_args) — produces XML string
 *   5. Singleton::loadScene(xml)                     — loads MuJoCo world (once)
 *   6. sub_handler->onSceneLoaded()                  — maps joints against loaded mjModel
 *
 * NOTE: In a multi-robot setup the plugin that must parameterise the scene xacro
 * (e.g. Franka, which provides control_mode) should be listed first in the URDF
 * so its perform_command_mode_switch runs before the others.
 */
class MujocoHardwareInterface : public hardware_interface::SystemInterface
{
public:
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    std::vector<hardware_interface::StateInterface>   export_state_interfaces()   override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string> & start_interfaces, const std::vector<std::string> & stop_interfaces) override;
    hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string> & start_interfaces, const std::vector<std::string> & stop_interfaces) override;
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    bool buildAndLoadScene(const std::string & xacro_args);

    static std::string shellEscape(const std::string & value);
    static std::string runCommand(const std::string & cmd, int * exit_code);

    std::unique_ptr<SubHandlerBase> sub_handler_;
};

}  // namespace mujoco_ros_hardware
