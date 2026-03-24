#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "mujoco_ros_hardware/sub_handler_base.hpp"

namespace mujoco_ros_hardware
{

/**
 * Maps robot_type strings to SubHandler factory functions.
 *
 * MujocoHardwareInterface only knows this registry — it has no dependency
 * on concrete handler types. Each handler registers itself via
 * REGISTER_SUB_HANDLER() at static initialization time.
 */
class SubHandlerRegistry
{
public:
    using Factory = std::function<std::unique_ptr<SubHandlerBase>()>;

    static SubHandlerRegistry & get();

    void registerHandler(const std::string & robot_type, Factory factory);

    // Returns nullptr if robot_type is unknown.
    std::unique_ptr<SubHandlerBase> create(const std::string & robot_type) const;

private:
    SubHandlerRegistry() = default;
    std::map<std::string, Factory> factories_;
};

}  // namespace mujoco_ros_hardware

/**
 * Place this macro at namespace scope in a handler's .cpp to register it.
 *
 *   REGISTER_SUB_HANDLER("franka", mujoco_ros_hardware::FrankaSubHandler)
 */
#define REGISTER_SUB_HANDLER(robot_type_str, ClassName)                       \
    static const bool ClassName##_registered = []() {                         \
        mujoco_ros_hardware::SubHandlerRegistry::get().registerHandler(  \
            robot_type_str,                                                   \
            []() { return std::make_unique<ClassName>(); });                  \
        return true;                                                          \
    }();
