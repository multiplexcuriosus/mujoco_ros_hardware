# Architecture

## Table of Contents
- [MujocoHardwareInterface](#mujocohardwareinterface)
- [MujocoWorldSingleton](#mujocoworldsingleton)
- [SubHandlerBase & SubHandlerRegistry](#subhandlerbase--subhandlerregistry)
- [FrankaSubHandler](#frankasubhandler)
- [FrankaMultiSubHandler](#frankamultisubhandler)
- [HuskySubHandler](#huskysubhandler)
- [MujocoFrankaModel](#mujocofrankmodel)
- [Adding a New Robot Type](#adding-a-new-robot-type)

---

## MujocoHardwareInterface

Single `hardware_interface::SystemInterface` used for all robot types. On `on_init()` it reads `robot_type`, creates the matching sub-handler via `SubHandlerRegistry`, and registers itself with `MujocoWorldSingleton`.

Scene loading happens in `perform_command_mode_switch()` — not in `on_init()` — because the ROS 2 executor must already be running for `MujocoWorldSingleton::init()` to call the `controller_manager` parameter service without deadlocking.

**Scene loading flow (per `perform_command_mode_switch` call until scene is loaded):**

1. `sub_handler->performCommandModeSwitch()` — determines runtime state (e.g. `control_mode`)
2. `sub_handler->isReadyToLoadScene()` — waits until xacro args are available
3. `sub_handler->getXacroArgs()` — returns handler-specific xacro args
4. `runXacro(xacro_path, base_args + handler_args)` → XML string
5. `MujocoWorldSingleton::loadScene(xml)` — loads MuJoCo world (once, thread-safe)
6. `sub_handler->onSceneLoaded()` — maps joints against loaded `mjModel`

---

## MujocoWorldSingleton

Process-level singleton owning the single `mjModel*`/`mjData*` shared across all plugin instances in the same controller_manager.

**Key responsibilities:**
- Reads `mujoco_scene_xacro_path` and `mujoco_scene_xacro_args` from `controller_manager` ROS parameters via `SyncParametersClient`
- Loads the compiled XML scene via `mj_loadXML()`
- Runs a real-time simulation thread calling `mj_step()` at `model->opt.timestep`
- Opens a passive MuJoCo viewer window (GLFW)
- Exposes `dataMutex()` — sub-handlers must hold this while accessing `model()`/`data()`

**Priority system:** Multiple plugins register with different `scenePriority()` values. Only the plugin whose priority equals `maxPriority()` loads the scene. This prevents race conditions in multi-robot setups (e.g. `FrankaSubHandler` at priority 10 always loads before `HuskySubHandler` at priority 0).

**controller_manager parameters (set from launch file):**

| Parameter | Description |
|-----------|-------------|
| `mujoco_scene_xacro_path` | Path to the MJCF xacro scene file |
| `mujoco_scene_xacro_args` | Base xacro args (e.g. `"as_two_wheels:=false"`) |

---

## SubHandlerBase & SubHandlerRegistry

`SubHandlerBase` is the abstract interface for all robot-specific logic:

| Method | Description |
|--------|-------------|
| `onInit()` | Read hw_params, allocate state/command buffers |
| `exportStateInterfaces()` | Return state interface list |
| `exportCommandInterfaces()` | Return command interface list |
| `scenePriority()` | Priority for scene loading (default: 0) |
| `isReadyToLoadScene()` | True when xacro args are available (default: true) |
| `getXacroArgs()` | Xacro argument string for scene parameterisation |
| `onSceneLoaded()` | Map joints against loaded `mjModel` |
| `read()` / `write()` | Read states from / write commands to `mjData` |

`SubHandlerRegistry` maps `robot_type` strings to factory functions. Handlers self-register via the `REGISTER_SUB_HANDLER` macro at static initialization time. `MujocoHardwareInterface` has no compile-time dependency on any concrete handler.

---

## FrankaSubHandler

**robot_type:** `"franka"`

Sub-handler for a single Franka FR3 arm.

**hw_params:**

| Parameter | Description |
|-----------|-------------|
| `arm_id` | Arm model identifier (e.g. `"fr3"`) |
| `prefix` | Arm prefix for multi-arm setups (e.g. `"left"`) |
| `load_gripper` | `"true"` \| `"false"` |

**State interfaces exported** (`name_stem = prefix + "_" + arm_id`):

| Interface | Description |
|-----------|-------------|
| `{joint_name}/position` | Joint position (rad) |
| `{joint_name}/velocity` | Joint velocity (rad/s) |
| `{joint_name}/effort` | Joint effort (Nm) |
| `{name_stem}/robot_time` | MuJoCo simulation time |
| `{name_stem}/robot_state` | Bit-cast `franka::RobotState*` as `double` |
| `{name_stem}/robot_model` | Bit-cast `franka_hardware::Model*` as `double` |

**Scene loading:** `scenePriority() = 10`. `isReadyToLoadScene()` returns false until `control_mode_` is set by `performCommandModeSwitch()`, preventing the scene from being built with an empty `control_mode` xacro arg.

**Xacro args:** `"arm_id:=<> hand:=<> control_mode:=<>"`

---

## FrankaMultiSubHandler

**robot_type:** `"franka_multi"`

Sub-handler for multiple Franka FR3 arms sharing a single URDF plugin block.

**hw_params:**

| Parameter | Description |
|-----------|-------------|
| `robot_count` | Number of arms (e.g. `"2"`) |
| `arm_id_<i>` | Arm identifier for robot `i` |
| `prefix_<i>` | Arm prefix for robot `i` |
| `load_gripper_<i>` | `"true"` \| `"false"` for robot `i` |

**Xacro args:** `"robot_count:=N arm_id_0:=... hand_0:=... control_mode_0:=... prefix_0:=... ..."`

`isReadyToLoadScene()` returns false until **all** robots have a `control_mode` set.

---

## HuskySubHandler

**robot_type:** `"husky"`

Sub-handler for the Clearpath Husky mobile base.

**hw_params:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `wheel_radius` | `0.1651` | Wheel radius (m) |
| `max_speed` | `6.06` | Max wheel speed (rad/s) |

`scenePriority() = 0` — scene is always loaded by `FrankaSubHandler` first. Joint mapping is done lazily on the first `read()`/`write()` call after the scene is loaded.

---

## MujocoFrankaModel

Implements `franka_hardware::Model` backed by the live MuJoCo simulation, allowing controllers that depend on dynamics (gravity compensation, impedance control) to run without real hardware.

| Method | Implementation |
|--------|----------------|
| `mass()` | 7×7 submatrix from `mj_fullM()` (column-major) |
| `gravity()` | `qfrc_bias[qvel_idx]` per joint (gravity + Coriolis combined) |
| `coriolis()` | Zeros (bias already included in `gravity()`) |
| `pose()` / `bodyJacobian()` / `zeroJacobian()` | Zeros / Identity (stub) |

---

## Adding a New Robot Type

1. Create `my_sub_handler.hpp` / `my_sub_handler.cpp` inheriting `SubHandlerBase`.
2. Implement at minimum: `onInit()`, `exportStateInterfaces()`, `exportCommandInterfaces()`, `read()`, `write()`.
3. Override `scenePriority()`, `isReadyToLoadScene()`, `getXacroArgs()`, `onSceneLoaded()` as needed.
4. Register using the macro **after** the closing namespace brace in the `.cpp` file:

```cpp
} // namespace mujoco_ros_hardware

REGISTER_SUB_HANDLER("my_robot", mujoco_ros_hardware::MySubHandler)
```

5. Set `robot_type:=my_robot` in the URDF `<hardware>` block.
6. Add the `.cpp` to `CMakeLists.txt` alongside the existing handlers.
