#include "mujoco_ros_hardware/franka_multi_sub_handler.hpp"
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

// ---------------------------------------------------------------------------
// onInit
// ---------------------------------------------------------------------------

hardware_interface::CallbackReturn FrankaMultiSubHandler::onInit(const hardware_interface::HardwareInfo & info)
{
    hw_info_ = info;

    const size_t robot_count = static_cast<size_t>(std::stoi(getParam(info, "robot_count", "1")));

    // Reserve to prevent reallocation after self-pointer fix-up below.
    robots_.reserve(robot_count);

    for (size_t i = 0; i < robot_count; ++i)
    {
        const std::string idx = std::to_string(i);
        RobotContext ctx;
        ctx.arm_id     = getParam(info, "arm_id_"      + idx, "fr3");
        ctx.arm_prefix = getParam(info, "prefix_"      + idx, "");
        ctx.hand       = normalizeBool(getParam(info, "load_gripper_" + idx, "true"), "true");
        ctx.name_stem  = ctx.arm_prefix.empty() ? ctx.arm_id : (ctx.arm_prefix + "_" + ctx.arm_id);
        robots_.push_back(std::move(ctx));
    }


    // Assign joints to robots by matching joint name against each robot's name_stem.
    for (const auto & joint_info : info.joints)
    {
        const size_t ridx = findRobotForJoint(joint_info.name);
        if (ridx >= robots_.size())
        {
            RCLCPP_WARN(
                rclcpp::get_logger("FrankaMultiSubHandler"),
                "\033[33mJoint '%s' could not be assigned to any robot\033[0m",
                joint_info.name.c_str());
            continue;
        }

        JointData jd;
        jd.name = joint_info.name;

        for (const auto & si : joint_info.state_interfaces)
        {
            if      (si.name == hardware_interface::HW_IF_POSITION)
            {
                jd.has_pos_state = true;
                if (!si.initial_value.empty())
                    jd.initial_pos = std::stod(si.initial_value);
            }
            else if (si.name == hardware_interface::HW_IF_VELOCITY) jd.has_vel_state = true;
            else if (si.name == hardware_interface::HW_IF_EFFORT)   jd.has_eff_state = true;
        }
        for (const auto & ci : joint_info.command_interfaces)
        {
            if      (ci.name == hardware_interface::HW_IF_POSITION) jd.has_pos_cmd = true;
            else if (ci.name == hardware_interface::HW_IF_VELOCITY) jd.has_vel_cmd = true;
            else if (ci.name == hardware_interface::HW_IF_EFFORT)   jd.has_eff_cmd = true;
        }

        robots_[ridx].joints.push_back(std::move(jd));
    }

    // Log assignment summary
    for (size_t i = 0; i < robots_.size(); ++i)
    {
        RCLCPP_INFO(
            rclcpp::get_logger("FrankaMultiSubHandler"),
            "\033[34mRobot[%zu] name_stem='%s' hand=%s joints=%zu\033[0m",
            i, robots_[i].name_stem.c_str(), robots_[i].hand.c_str(),
            robots_[i].joints.size());
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// exportStateInterfaces / exportCommandInterfaces
// ---------------------------------------------------------------------------

std::vector<hardware_interface::StateInterface> FrankaMultiSubHandler::exportStateInterfaces()
{
    std::vector<hardware_interface::StateInterface> si;
    for (auto & robot : robots_)
    {
        for (auto & j : robot.joints)
        {
            if (j.has_pos_state) si.emplace_back(j.name, hardware_interface::HW_IF_POSITION, &j.pos_state);
            if (j.has_vel_state) si.emplace_back(j.name, hardware_interface::HW_IF_VELOCITY, &j.vel_state);
            if (j.has_eff_state) si.emplace_back(j.name, hardware_interface::HW_IF_EFFORT,   &j.effort_state);
        }

    }
    return si;
}

std::vector<hardware_interface::CommandInterface> FrankaMultiSubHandler::exportCommandInterfaces()
{
    std::vector<hardware_interface::CommandInterface> ci;
    for (auto & robot : robots_)
    {
        for (auto & j : robot.joints)
        {
            if (j.has_pos_cmd) ci.emplace_back(j.name, hardware_interface::HW_IF_POSITION, &j.pos_cmd);
            if (j.has_vel_cmd) ci.emplace_back(j.name, hardware_interface::HW_IF_VELOCITY, &j.vel_cmd);
            if (j.has_eff_cmd) ci.emplace_back(j.name, hardware_interface::HW_IF_EFFORT,   &j.effort_cmd);
        }
    }
    return ci;
}

// ---------------------------------------------------------------------------
// performCommandModeSwitch
// ---------------------------------------------------------------------------

hardware_interface::return_type FrankaMultiSubHandler::performCommandModeSwitch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> &)
{
    for (auto & robot : robots_)
    {
        if (!robot.control_mode.empty()) continue;

        // Filter interfaces belonging to this robot (matched by name_stem)
        std::vector<std::string> my_interfaces;
        for (const auto & iface : start_interfaces)
        {
            if (iface.find(robot.name_stem) != std::string::npos)
                my_interfaces.push_back(iface);
        }
        if (my_interfaces.empty()) continue;

        std::string mode;
        for (const auto & iface : my_interfaces)
        {
            const auto type = interfaceTypeFromName(iface);
            if (type != "position" && type != "velocity" && type != "effort")
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("FrankaMultiSubHandler"),
                    "\033[31mUnsupported command interface for robot '%s': '%s'\033[0m",
                    robot.name_stem.c_str(), iface.c_str());
                return hardware_interface::return_type::ERROR;
            }
            if (mode.empty()) mode = type;
            else if (mode != type)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("FrankaMultiSubHandler"),
                    "\033[31mMixed command interfaces for robot '%s': %s vs %s\033[0m",
                    robot.name_stem.c_str(), mode.c_str(), type.c_str());
                return hardware_interface::return_type::ERROR;
            }
        }

        robot.control_mode = mode;
        RCLCPP_INFO(
            rclcpp::get_logger("FrankaMultiSubHandler"),
            "\033[32mRobot '%s' control mode set to: %s\033[0m",
            robot.name_stem.c_str(), mode.c_str());
    }

    return hardware_interface::return_type::OK;
}

// ---------------------------------------------------------------------------
// isReadyToLoadScene / getXacroArgs
// ---------------------------------------------------------------------------

bool FrankaMultiSubHandler::isReadyToLoadScene() const
{
    for (const auto & r : robots_)
        if (r.control_mode.empty()) return false;
    return true;
}

std::string FrankaMultiSubHandler::getXacroArgs() const
{
    std::ostringstream args;
    args << "robot_count:=" << robots_.size();
    for (size_t i = 0; i < robots_.size(); ++i)
    {
        const auto & r = robots_[i];
        args << " arm_id_"      << i << ":=" << r.arm_id
             << " hand_"        << i << ":=" << r.hand
             << " control_mode_"<< i << ":=" << r.control_mode
             << " prefix_"      << i << ":=" << r.arm_prefix;
    }
    return args.str();
}

// ---------------------------------------------------------------------------
// onSceneLoaded
// ---------------------------------------------------------------------------

void FrankaMultiSubHandler::onSceneLoaded()
{
    auto & world = MujocoWorldSingleton::get();
    const auto * m = world.model();
    if (!m) return;

    {
        std::lock_guard<std::mutex> lock(world.dataMutex());
        auto * d = world.data();
        if (!d) return;

        for (auto & robot : robots_)
        {
            for (auto & j : robot.joints)
            {
                const int jid = mj_name2id(m, mjOBJ_JOINT, j.name.c_str());
                if (jid < 0)
                {
                    RCLCPP_WARN(
                        rclcpp::get_logger("FrankaMultiSubHandler"),
                        "\033[33mJoint '%s' not found in MuJoCo model\033[0m",
                        j.name.c_str());
                    continue;
                }
                j.qpos_idx = m->jnt_qposadr[jid];
                j.qvel_idx = m->jnt_dofadr[jid];
                j.ctrl_idx = mj_name2id(m, mjOBJ_ACTUATOR, j.name.c_str());

                d->qpos[j.qpos_idx] = j.initial_pos;
                d->qvel[j.qvel_idx] = 0.0;
                j.pos_state = j.initial_pos;
                j.vel_state = 0.0;
                j.pos_cmd   = j.initial_pos;

                RCLCPP_INFO(
                    rclcpp::get_logger("FrankaMultiSubHandler"),
                    "\033[34m[%s] qpos=%d qvel=%d ctrl=%d init_pos=%.4f\033[0m",
                    j.name.c_str(), j.qpos_idx, j.qvel_idx, j.ctrl_idx, j.initial_pos);
            }
            robot.joints_mapped = true;
        }

        // Single mj_forward propagates all robots' initial positions at once.
        mj_forward(m, d);
    }

}

// ---------------------------------------------------------------------------
// read / write
// ---------------------------------------------------------------------------

hardware_interface::return_type FrankaMultiSubHandler::read(const rclcpp::Time &, const rclcpp::Duration &)
{
    auto & world = MujocoWorldSingleton::get();
    if (!world.isSceneLoaded()) return hardware_interface::return_type::OK;

    std::lock_guard<std::mutex> lock(world.dataMutex());
    const auto * d = world.data();
    if (!d) return hardware_interface::return_type::OK;

    for (auto & robot : robots_)
    {
        if (!robot.joints_mapped) continue;
        for (size_t i = 0; i < robot.joints.size(); ++i)
        {
            auto & j = robot.joints[i];
            if (j.qpos_idx < 0) continue;
            j.pos_state    = d->qpos[j.qpos_idx];
            j.vel_state    = d->qvel[j.qvel_idx];
            j.effort_state = (j.ctrl_idx >= 0) ? d->actuator_force[j.ctrl_idx] : 0.0;
        }
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type FrankaMultiSubHandler::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    auto & world = MujocoWorldSingleton::get();
    if (!world.isSceneLoaded()) return hardware_interface::return_type::OK;

    std::lock_guard<std::mutex> lock(world.dataMutex());
    auto * d = world.data();
    if (!d) return hardware_interface::return_type::OK;

    for (const auto & robot : robots_)
    {
        if (!robot.joints_mapped) continue;
        for (const auto & j : robot.joints)
        {
            if (j.ctrl_idx < 0) continue;
            if      (robot.control_mode == "position") d->ctrl[j.ctrl_idx] = j.pos_cmd;
            else if (robot.control_mode == "velocity") d->ctrl[j.ctrl_idx] = j.vel_cmd;
            else                                       d->ctrl[j.ctrl_idx] = j.effort_cmd;
        }
    }

    return hardware_interface::return_type::OK;
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

size_t FrankaMultiSubHandler::findRobotForJoint(const std::string & joint_name) const
{
    for (size_t i = 0; i < robots_.size(); ++i)
    {
        if (joint_name.find(robots_[i].name_stem) != std::string::npos)
            return i;
    }
    return robots_.size();  // not found
}

std::string FrankaMultiSubHandler::interfaceTypeFromName(const std::string & full_name)
{
    const auto slash = full_name.rfind('/');
    if (slash == std::string::npos || slash + 1 >= full_name.size()) return "";
    return full_name.substr(slash + 1);
}

std::string FrankaMultiSubHandler::normalizeBool(const std::string & value, const std::string & default_val)
{
    if (value.empty()) return default_val;
    std::string v = value;
    std::transform(v.begin(), v.end(), v.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (v == "1" || v == "true"  || v == "yes" || v == "on")  return "true";
    if (v == "0" || v == "false" || v == "no"  || v == "off") return "false";
    return value;
}

REGISTER_SUB_HANDLER("franka_multi", FrankaMultiSubHandler)

}  // namespace mujoco_ros_hardware
