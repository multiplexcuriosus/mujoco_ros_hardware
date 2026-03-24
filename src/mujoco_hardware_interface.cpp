#include "mujoco_ros_hardware/mujoco_hardware_interface.hpp"

#include <array>
#include <sstream>

#include "mujoco_ros_hardware/mujoco_world_singleton.hpp"
#include "mujoco_ros_hardware/sub_handler_registry.hpp"

namespace mujoco_ros_hardware
{

hardware_interface::CallbackReturn MujocoHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) return hardware_interface::CallbackReturn::ERROR;

    const auto it = info_.hardware_parameters.find("robot_type");
    if (it == info_.hardware_parameters.end())
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("MujocoHardwareInterface"),
            "\033[31mMissing required hw_param: robot_type\033[0m");
        return hardware_interface::CallbackReturn::ERROR;
    }

    const std::string & robot_type = it->second;
    sub_handler_ = SubHandlerRegistry::get().create(robot_type);
    if (!sub_handler_)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("MujocoHardwareInterface"),
            "\033[31mNo SubHandler registered for robot_type: '%s'\033[0m",
            robot_type.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }

    MujocoWorldSingleton::get().registerPlugin(sub_handler_->scenePriority());

    return sub_handler_->onInit(info);
}

hardware_interface::CallbackReturn MujocoHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MujocoHardwareInterface::export_state_interfaces()
{
    return sub_handler_->exportStateInterfaces();
}

std::vector<hardware_interface::CommandInterface> MujocoHardwareInterface::export_command_interfaces()
{
    return sub_handler_->exportCommandInterfaces();
}

hardware_interface::return_type MujocoHardwareInterface::prepare_command_mode_switch(const std::vector<std::string> & start_interfaces, const std::vector<std::string> & stop_interfaces)
{
    return sub_handler_->prepareCommandModeSwitch(start_interfaces, stop_interfaces);
}

hardware_interface::return_type MujocoHardwareInterface::perform_command_mode_switch(const std::vector<std::string> & start_interfaces, const std::vector<std::string> & stop_interfaces)
{
    // Step 1: let sub-handler determine runtime state (e.g. control_mode)
    const auto ret = sub_handler_->performCommandModeSwitch(start_interfaces, stop_interfaces);
    if (ret != hardware_interface::return_type::OK) return ret;

    // Step 2-5: load scene once.
    // Skip if already loaded, or if another plugin has higher priority, or if not ready.
    if (MujocoWorldSingleton::get().isSceneLoaded()) return hardware_interface::return_type::OK;
    if (sub_handler_->scenePriority() < MujocoWorldSingleton::get().maxPriority()) return hardware_interface::return_type::OK;
    if (!sub_handler_->isReadyToLoadScene()) return hardware_interface::return_type::OK;

    // Read xacro path + base args from controller_manager ROS params.
    // Done here (not in on_init) because the executor must be running for SyncParametersClient.
    if (!MujocoWorldSingleton::get().init())
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("MujocoHardwareInterface"),
            "\033[31mMujocoWorldSingleton::init() failed\033[0m");
        return hardware_interface::return_type::ERROR;
    }

    // Combine controller_manager base args + handler-specific args
    std::string xacro_args = MujocoWorldSingleton::get().xacroBaseArgs();
    const std::string handler_args = sub_handler_->getXacroArgs();
    if (!handler_args.empty())
    {
        if (!xacro_args.empty()) xacro_args += ' ';
        xacro_args += handler_args;
    }

    if (!buildAndLoadScene(xacro_args)) return hardware_interface::return_type::ERROR;

    sub_handler_->onSceneLoaded();
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MujocoHardwareInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    return sub_handler_->read(time, period);
}

hardware_interface::return_type MujocoHardwareInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    return sub_handler_->write(time, period);
}

bool MujocoHardwareInterface::buildAndLoadScene(const std::string & xacro_args)
{
    const std::string & xml_path   = MujocoWorldSingleton::get().xmlPath();
    const std::string & xacro_path = MujocoWorldSingleton::get().xacroPath();

    if (!xml_path.empty())
    {
        return MujocoWorldSingleton::get().loadSceneFromPath(xml_path);
    }
    
    std::ostringstream cmd;
    cmd << "xacro --inorder " << shellEscape(xacro_path);
    if (!xacro_args.empty()) cmd << " " << xacro_args;

    RCLCPP_INFO(
        rclcpp::get_logger("MujocoHardwareInterface"),
        "\033[34mRunning: %s\033[0m", cmd.str().c_str());

    int exit_code = 0;
    const std::string xml = runCommand(cmd.str(), &exit_code);
    if (exit_code != 0 || xml.empty())
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("MujocoHardwareInterface"),
            "\033[31mxacro failed (exit=%d)\033[0m", exit_code);
        return false;
    }

    return MujocoWorldSingleton::get().loadSceneFromXML(xml);
}

std::string MujocoHardwareInterface::shellEscape(const std::string & value)
{
    std::string out;
    out.reserve(value.size() + 2);
    out.push_back('\'');
    for (char c : value)
    {
        if (c == '\'') out.append("'\\''");
        else           out.push_back(c);
    }
    out.push_back('\'');
    return out;
}

std::string MujocoHardwareInterface::runCommand(const std::string & cmd, int * exit_code)
{
    std::array<char, 4096> buffer{};
    std::string result;
    FILE * pipe = popen(cmd.c_str(), "r");
    if (!pipe) { if (exit_code) *exit_code = -1; return ""; }
    while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) != nullptr)
        result.append(buffer.data());
    const int rc = pclose(pipe);
    if (exit_code) *exit_code = rc;
    return result;
}

}  // namespace mujoco_ros_hardware

PLUGINLIB_EXPORT_CLASS(
    mujoco_ros_hardware::MujocoHardwareInterface,
    hardware_interface::SystemInterface)
