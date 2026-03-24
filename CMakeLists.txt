cmake_minimum_required(VERSION 3.16)
project(mujoco_ros_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mujoco REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glfw3 QUIET)
if(NOT glfw3_FOUND)
    include(FetchContent)
    FetchContent_Declare(
        glfw
        GIT_REPOSITORY https://github.com/glfw/glfw.git
        GIT_TAG        3.4
    )
    set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_DOCS     OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(glfw)
endif()

include(FetchContent)
FetchContent_Declare(
    lodepng
    GIT_REPOSITORY https://github.com/lvandeve/lodepng.git
    GIT_TAG        b4ed2cd7ecf61d29076169b49199371456d4f90b
)
FetchContent_GetProperties(lodepng)
if(NOT lodepng_POPULATED)
    FetchContent_Populate(lodepng)
    add_library(lodepng OBJECT
        ${lodepng_SOURCE_DIR}/lodepng.cpp
        ${lodepng_SOURCE_DIR}/lodepng.h
    )
    target_include_directories(lodepng PUBLIC ${lodepng_SOURCE_DIR})
    set_target_properties(lodepng PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

set(SIMULATE_SRCDIR "${CMAKE_CURRENT_SOURCE_DIR}/src/simulate")

add_library(mujoco_simulate STATIC
    ${SIMULATE_SRCDIR}/simulate.cc
    ${SIMULATE_SRCDIR}/platform_ui_adapter.cc
    ${SIMULATE_SRCDIR}/glfw_dispatch.cc
    ${SIMULATE_SRCDIR}/glfw_adapter.cc
)
set_target_properties(mujoco_simulate PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(mujoco_simulate PUBLIC
    $<BUILD_INTERFACE:${SIMULATE_SRCDIR}>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(mujoco_simulate PUBLIC
    mujoco::mujoco
    glfw
    OpenGL::GL
)
target_sources(mujoco_simulate PRIVATE $<TARGET_OBJECTS:lodepng>)
target_include_directories(mujoco_simulate PRIVATE ${lodepng_SOURCE_DIR})

add_library(${PROJECT_NAME} SHARED
    src/sub_handler_registry.cpp
    src/mujoco_world_singleton.cpp
    src/mujoco_hardware_interface.cpp
    src/franka_sub_handler.cpp
    src/franka_multi_sub_handler.cpp
    src/husky_sub_handler.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    mujoco::mujoco
    mujoco_simulate
    dl
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
    rclcpp
    hardware_interface
    pluginlib
    sensor_msgs
)

pluginlib_export_plugin_description_file(hardware_interface mujoco_hardware_interface_plugin.xml)

install(TARGETS 
    mujoco_simulate
    ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}_targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
install(DIRECTORY include/
    DESTINATION include
)

ament_export_targets(${PROJECT_NAME}_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp hardware_interface pluginlib sensor_msgs)
ament_export_include_directories(include)

ament_package()
