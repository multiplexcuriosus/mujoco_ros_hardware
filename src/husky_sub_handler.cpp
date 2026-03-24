#include "mujoco_ros_hardware/husky_sub_handler.hpp"
#include "mujoco_ros_hardware/sub_handler_registry.hpp"

#include <algorithm>
#include <cmath>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <mujoco/mujoco.h>

#include "mujoco_ros_hardware/mujoco_world_singleton.hpp"

namespace mujoco_ros_hardware
{

namespace
{

std::string getParam(
    const hardware_interface::HardwareInfo & info,
    const std::string & key,
    const std::string & default_val = "")
{
    auto it = info.hardware_parameters.find(key);
    return it != info.hardware_parameters.end() ? it->second : default_val;
}

}  // namespace

hardware_interface::CallbackReturn HuskySubHandler::onInit(const hardware_interface::HardwareInfo & info)
{
    hw_info_ = info;

    try
    {
        const double wheel_diameter = std::stod(getParam(info, "wheel_diameter", "0.3302"));
        const double max_speed_ms   = std::stod(getParam(info, "max_speed",      "1.0"));
        wheel_radius_    = wheel_diameter / 2.0;
        max_speed_radps_ = max_speed_ms / wheel_radius_;
    }
    catch (const std::exception & e)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("HuskySubHandler"),
            "\033[31mParameter parsing failed: %s\033[0m", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    joints_.resize(info.joints.size());
    for (size_t i = 0; i < info.joints.size(); ++i)
        joints_[i].name = info.joints[i].name;

    RCLCPP_INFO(
        rclcpp::get_logger("HuskySubHandler"),
        "\033[34mwheel_radius=%.4f  max_speed=%.2f rad/s\033[0m",
        wheel_radius_, max_speed_radps_);

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> HuskySubHandler::exportStateInterfaces()
{
    std::vector<hardware_interface::StateInterface> si;
    for (size_t i = 0; i < hw_info_.joints.size(); ++i)
    {
        auto & j = joints_[i];
        for (const auto & iface : hw_info_.joints[i].state_interfaces)
        {
            if      (iface.name == hardware_interface::HW_IF_POSITION)
                si.emplace_back(j.name, hardware_interface::HW_IF_POSITION, &j.pos_state);
            else if (iface.name == hardware_interface::HW_IF_VELOCITY)
                si.emplace_back(j.name, hardware_interface::HW_IF_VELOCITY, &j.vel_state);
        }
    }
    return si;
}

std::vector<hardware_interface::CommandInterface> HuskySubHandler::exportCommandInterfaces()
{
    std::vector<hardware_interface::CommandInterface> ci;
    for (size_t i = 0; i < hw_info_.joints.size(); ++i)
    {
        auto & j = joints_[i];
        for (const auto & iface : hw_info_.joints[i].command_interfaces)
        {
            if (iface.name == hardware_interface::HW_IF_VELOCITY)
                ci.emplace_back(j.name, hardware_interface::HW_IF_VELOCITY, &j.vel_cmd);
        }
    }
    return ci;
}

hardware_interface::return_type HuskySubHandler::read(const rclcpp::Time &, const rclcpp::Duration &)
{
    auto & world = MujocoWorldSingleton::get();
    if (!world.isSceneLoaded()) return hardware_interface::return_type::OK;
    if (!joints_mapped_) mapJoints();

    std::lock_guard<std::mutex> lock(world.dataMutex());
    const auto * d = world.data();
    if (!d) return hardware_interface::return_type::OK;

    for (auto & j : joints_)
    {
        if (j.qpos_idx < 0) continue;
        j.pos_state = d->qpos[j.qpos_idx];
        j.vel_state = d->qvel[j.qvel_idx];
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type HuskySubHandler::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    auto & world = MujocoWorldSingleton::get();
    if (!world.isSceneLoaded()) return hardware_interface::return_type::OK;
    if (!joints_mapped_) mapJoints();

    std::lock_guard<std::mutex> lock(world.dataMutex());
    auto * d = world.data();
    if (!d) return hardware_interface::return_type::OK;

    // Average left/right wheel commands (all wheels on same side get same cmd)
    double left_cmd = 0.0, right_cmd = 0.0;
    for (const auto & j : joints_)
    {
        if      (j.name.find("left")  != std::string::npos) left_cmd  = j.vel_cmd;
        else if (j.name.find("right") != std::string::npos) right_cmd = j.vel_cmd;
    }
    left_cmd  = std::clamp(left_cmd,  -max_speed_radps_, max_speed_radps_);
    right_cmd = std::clamp(right_cmd, -max_speed_radps_, max_speed_radps_);

    for (const auto & j : joints_)
    {
        if (j.ctrl_idx < 0) continue;
        if      (j.name.find("left")  != std::string::npos) d->ctrl[j.ctrl_idx] = left_cmd;
        else if (j.name.find("right") != std::string::npos) d->ctrl[j.ctrl_idx] = right_cmd;
    }
    return hardware_interface::return_type::OK;
}

void HuskySubHandler::mapJoints()
{
    auto & world = MujocoWorldSingleton::get();
    const auto * m = world.model();
    const auto * d = world.data();
    if (!m || !d) return;

    for (auto & j : joints_)
    {
        const int jid = mj_name2id(m, mjOBJ_JOINT, j.name.c_str());
        if (jid < 0)
        {
            RCLCPP_WARN(
                rclcpp::get_logger("HuskySubHandler"),
                "\033[33mJoint '%s' not found in MuJoCo model\033[0m", j.name.c_str());
            continue;
        }
        j.qpos_idx  = m->jnt_qposadr[jid];
        j.qvel_idx  = m->jnt_dofadr[jid];
        j.pos_state = d->qpos[j.qpos_idx];
        j.vel_state = d->qvel[j.qvel_idx];

        j.ctrl_idx = mj_name2id(m, mjOBJ_ACTUATOR, j.name.c_str());

        RCLCPP_INFO(
            rclcpp::get_logger("HuskySubHandler"),
            "\033[34m[%s] qpos=%d qvel=%d ctrl=%d\033[0m",
            j.name.c_str(), j.qpos_idx, j.qvel_idx, j.ctrl_idx);
    }
    joints_mapped_ = true;
}

REGISTER_SUB_HANDLER("husky", HuskySubHandler)

}  // namespace mujoco_ros_hardware
