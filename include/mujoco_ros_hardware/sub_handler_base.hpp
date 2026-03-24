#pragma once

#include <string>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"

namespace mujoco_ros_hardware
{

/**
 * Abstract base for robot-specific sub-handlers.
 * MujocoHardwareInterface delegates all logic here based on robot_type param.
 */
class SubHandlerBase
{
public:
    virtual ~SubHandlerBase() = default;

    virtual hardware_interface::CallbackReturn onInit(const hardware_interface::HardwareInfo & info) = 0;

    virtual std::vector<hardware_interface::StateInterface>   exportStateInterfaces()   = 0;
    virtual std::vector<hardware_interface::CommandInterface> exportCommandInterfaces() = 0;

    virtual hardware_interface::return_type prepareCommandModeSwitch(const std::vector<std::string> &, const std::vector<std::string> &)
    { return hardware_interface::return_type::OK; }

    // Determines any runtime state (e.g. control_mode) from start_interfaces.
    // Does NOT load the scene — scene loading is done by MujocoHardwareInterface.
    virtual hardware_interface::return_type performCommandModeSwitch(const std::vector<std::string> &, const std::vector<std::string> &)
    { return hardware_interface::return_type::OK; }

    // Scene-loading priority. The plugin with the highest registered priority loads
    // the scene. Among equal-priority plugins, the first to arrive wins.
    virtual int scenePriority() const { return 0; }

    // Returns true when the handler has enough runtime state to generate xacro args.
    // Override in handlers that need a command-mode switch before the scene can be built
    // (e.g. FrankaSubHandler must know control_mode before calling getXacroArgs).
    virtual bool isReadyToLoadScene() const { return true; }

    // Returns xacro args string (e.g. "arm_id:=fr3 hand:=true control_mode:=position").
    // Called by MujocoHardwareInterface after performCommandModeSwitch to build the scene.
    virtual std::string getXacroArgs() const { return ""; }

    // Called by MujocoHardwareInterface after the scene is successfully loaded.
    // Use this to map joints against the newly loaded mjModel.
    virtual void onSceneLoaded() {}

    virtual hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) = 0;

    virtual hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) = 0;
};

}  // namespace mujoco_ros_hardware
