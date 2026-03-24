#include "mujoco_ros_hardware/franka_sub_handler.hpp"
#include "mujoco_ros_hardware/sub_handler_registry.hpp"

#include <algorithm>
#include <cctype>
#include <sstream>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <mujoco/mujoco.h>

#include "mujoco_ros_hardware/mujoco_world_singleton.hpp"

namespace mujoco_ros_hardware
{

namespace
{

std::string getParam(const hardware_interface::HardwareInfo & info, const std::string & key, const std::string & default_val = "")
{
    auto it = info.hardware_parameters.find(key);
    return it != info.hardware_parameters.end() ? it->second : default_val;
}

}  // namespace

hardware_interface::CallbackReturn FrankaSubHandler::onInit(const hardware_interface::HardwareInfo & info)
{
    hw_info_ = info;

    arm_id_     = getParam(info, "arm_id", "fr3");
    arm_prefix_ = getParam(info, "prefix", "");
    name_stem_  = arm_prefix_.empty() ? arm_id_ : (arm_prefix_ + "_" + arm_id_);
    hand_       = normalizeBool(getParam(info, "load_gripper", "true"), "true");

    joints_.resize(info.joints.size());
    for (size_t i = 0; i < info.joints.size(); ++i)
    {
        joints_[i].name = info.joints[i].name;
        for (const auto & si : info.joints[i].state_interfaces)
        {
            if (si.name == hardware_interface::HW_IF_POSITION && !si.initial_value.empty())
            {
                joints_[i].initial_pos = std::stod(si.initial_value);
                break;
            }
        }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> FrankaSubHandler::exportStateInterfaces()
{
    std::vector<hardware_interface::StateInterface> si;
    for (size_t i = 0; i < hw_info_.joints.size(); ++i)
    {
        auto & j = joints_[i];
        for (const auto & iface : hw_info_.joints[i].state_interfaces)
        {
            if      (iface.name == hardware_interface::HW_IF_POSITION) si.emplace_back(j.name, hardware_interface::HW_IF_POSITION, &j.pos_state);
            else if (iface.name == hardware_interface::HW_IF_VELOCITY) si.emplace_back(j.name, hardware_interface::HW_IF_VELOCITY, &j.vel_state);
            else if (iface.name == hardware_interface::HW_IF_EFFORT)   si.emplace_back(j.name, hardware_interface::HW_IF_EFFORT,   &j.effort_state);
        }
    }

    return si;
}

std::vector<hardware_interface::CommandInterface> FrankaSubHandler::exportCommandInterfaces()
{
    std::vector<hardware_interface::CommandInterface> ci;
    for (size_t i = 0; i < hw_info_.joints.size(); ++i)
    {
        auto & j = joints_[i];
        for (const auto & iface : hw_info_.joints[i].command_interfaces)
        {
            if      (iface.name == hardware_interface::HW_IF_POSITION) ci.emplace_back(j.name, hardware_interface::HW_IF_POSITION, &j.pos_cmd);
            else if (iface.name == hardware_interface::HW_IF_VELOCITY) ci.emplace_back(j.name, hardware_interface::HW_IF_VELOCITY, &j.vel_cmd);
            else if (iface.name == hardware_interface::HW_IF_EFFORT)   ci.emplace_back(j.name, hardware_interface::HW_IF_EFFORT,   &j.effort_cmd);
        }
    }
    return ci;
}

hardware_interface::return_type FrankaSubHandler::performCommandModeSwitch(const std::vector<std::string> & start_interfaces, const std::vector<std::string> &)
{
    if (!control_mode_.empty()) return hardware_interface::return_type::OK;

    // Filter: only consider interfaces belonging to this arm (containing arm_id_)
    std::vector<std::string> my_interfaces;
    for (const auto & iface : start_interfaces)
    {
        if (iface.find(arm_id_) != std::string::npos)
            my_interfaces.push_back(iface);
    }
    if (my_interfaces.empty()) return hardware_interface::return_type::OK;

    std::string mode;
    for (const auto & iface : my_interfaces)
    {
        const auto type = interfaceTypeFromName(iface);
        if (type != "position" && type != "velocity" && type != "effort")
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("FrankaSubHandler"),
                "\033[31mUnsupported command interface: '%s'\033[0m", iface.c_str());
            return hardware_interface::return_type::ERROR;
        }
        if (mode.empty()) mode = type;
        else if (mode != type)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("FrankaSubHandler"),
                "\033[31mMixed command interfaces: %s vs %s\033[0m",
                mode.c_str(), type.c_str());
            return hardware_interface::return_type::ERROR;
        }
    }

    control_mode_ = mode;
    return hardware_interface::return_type::OK;
}

std::string FrankaSubHandler::getXacroArgs() const
{
    std::ostringstream args;
    args << "arm_id:="       << arm_id_
         << " hand:="        << hand_
         << " control_mode:=" << control_mode_;
    return args.str();
}

void FrankaSubHandler::onSceneLoaded()
{
    mapJoints();
}

hardware_interface::return_type FrankaSubHandler::read(const rclcpp::Time &, const rclcpp::Duration &)
{
    auto & world = MujocoWorldSingleton::get();
    if (!world.isSceneLoaded() || !joints_mapped_) return hardware_interface::return_type::OK;

    std::lock_guard<std::mutex> lock(world.dataMutex());
    const auto * d = world.data();
    if (!d) return hardware_interface::return_type::OK;

    for (size_t i = 0; i < joints_.size(); ++i)
    {
        auto & j = joints_[i];
        if (j.qpos_idx < 0) continue;
        j.pos_state    = d->qpos[j.qpos_idx];
        j.vel_state    = d->qvel[j.qvel_idx];
        j.effort_state = (j.ctrl_idx >= 0) ? d->actuator_force[j.ctrl_idx] : 0.0;
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type FrankaSubHandler::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    auto & world = MujocoWorldSingleton::get();
    if (!world.isSceneLoaded() || !joints_mapped_) return hardware_interface::return_type::OK;

    std::lock_guard<std::mutex> lock(world.dataMutex());
    auto * d = world.data();
    if (!d) return hardware_interface::return_type::OK;

    for (const auto & j : joints_)
    {
        if (j.ctrl_idx < 0) continue;
        if      (control_mode_ == "position") d->ctrl[j.ctrl_idx] = j.pos_cmd;
        else if (control_mode_ == "velocity") d->ctrl[j.ctrl_idx] = j.vel_cmd;
        else                                   d->ctrl[j.ctrl_idx] = j.effort_cmd;
    }

    return hardware_interface::return_type::OK;
}

void FrankaSubHandler::mapJoints()
{
    auto & world = MujocoWorldSingleton::get();
    const auto * m = world.model();
    if (!m) return;

    std::lock_guard<std::mutex> lock(world.dataMutex());
    auto * d = world.data();
    if (!d) return;

    for (auto & j : joints_)
    {
        const int jid = mj_name2id(m, mjOBJ_JOINT, j.name.c_str());
        if (jid < 0)
        {
            RCLCPP_WARN(
                rclcpp::get_logger("FrankaSubHandler"),
                "\033[33mJoint '%s' not found in MuJoCo model\033[0m", j.name.c_str());
            continue;
        }
        j.qpos_idx = m->jnt_qposadr[jid];
        j.qvel_idx = m->jnt_dofadr[jid];
        j.ctrl_idx = mj_name2id(m, mjOBJ_ACTUATOR, j.name.c_str());

        // Apply initial position from URDF hw_interface initial_value
        d->qpos[j.qpos_idx] = j.initial_pos;
        d->qvel[j.qvel_idx] = 0.0;

        j.pos_state = j.initial_pos;
        j.vel_state = 0.0;
        j.pos_cmd   = j.initial_pos;

        RCLCPP_INFO(
            rclcpp::get_logger("FrankaSubHandler"),
            "\033[34m[%s] qpos=%d qvel=%d ctrl=%d init_pos=%.4f\033[0m",
            j.name.c_str(), j.qpos_idx, j.qvel_idx, j.ctrl_idx, j.initial_pos);
    }

    // Propagate physics state after setting all initial positions
    mj_forward(m, d);

    joints_mapped_ = true;
}

std::string FrankaSubHandler::interfaceTypeFromName(const std::string & full_name)
{
    const auto slash = full_name.rfind('/');
    if (slash == std::string::npos || slash + 1 >= full_name.size()) return "";
    return full_name.substr(slash + 1);
}

std::string FrankaSubHandler::normalizeBool(const std::string & value, const std::string & default_val)
{
    if (value.empty()) return default_val;
    std::string v = value;
    std::transform(v.begin(), v.end(), v.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (v == "1" || v == "true"  || v == "yes" || v == "on")  return "true";
    if (v == "0" || v == "false" || v == "no"  || v == "off") return "false";
    return value;
}

REGISTER_SUB_HANDLER("franka", FrankaSubHandler)

}  // namespace mujoco_ros_hardware
