#include "mujoco_ros_hardware/sub_handler_registry.hpp"

#include "rclcpp/rclcpp.hpp"

namespace mujoco_ros_hardware
{

SubHandlerRegistry & SubHandlerRegistry::get()
{
    static SubHandlerRegistry instance;
    return instance;
}

void SubHandlerRegistry::registerHandler(const std::string & robot_type, Factory factory)
{
    factories_[robot_type] = std::move(factory);
    RCLCPP_DEBUG(
        rclcpp::get_logger("SubHandlerRegistry"),
        "Registered SubHandler for robot_type: '%s'", robot_type.c_str());
}

std::unique_ptr<SubHandlerBase> SubHandlerRegistry::create(const std::string & robot_type) const
{
    const auto it = factories_.find(robot_type);
    if (it == factories_.end()) return nullptr;
    return it->second();
}

}  // namespace mujoco_ros_hardware
