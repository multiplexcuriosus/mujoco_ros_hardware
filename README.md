# mujoco_ros_hardware

Unified `ros2_control` hardware plugin for MuJoCo simulation. A single plugin (`MujocoHardwareInterface`) handles any robot type via pluggable sub-handlers, sharing one MuJoCo world across all plugin instances.

See [ARCHITECTURE.md](ARCHITECTURE.md) for design details.

---

## Dependencies

- ROS 2 Humble, `ros2_control` / `hardware_interface`, `pluginlib`
- MuJoCo (>= 3.x)
- `franka_hardware`, `libfranka` (for Franka sub-handlers)

---

## Build

```bash
colcon build --packages-select mujoco_ros_hardware
```

---

## Usage

Set `robot_type` in the URDF `<hardware>` block and pass the scene xacro path as `controller_manager` parameters in the launch file.

### URDF

```xml
<hardware>
  <plugin>mujoco_ros_hardware/MujocoHardwareInterface</plugin>
  <param name="robot_type">franka</param>   <!-- or "husky", "franka_multi" -->
  <param name="arm_id">fr3</param>
  <param name="prefix">left</param>
  <param name="load_gripper">true</param>
</hardware>
```

### Launch file

```python
cm_params = [
    controllers_yaml,
    {'robot_description': robot_description},
    {'mujoco_scene_xacro_path': '/path/to/scene.xml.xacro'},
    {'mujoco_scene_xacro_args': 'side:=left hand:=true'},
]
```

### Example launch commands

```bash
# FR3 single arm
ros2 launch franka_bringup example.launch.py \
  controller_name:=joint_position_example_controller use_mujoco:=true

# FR3 + Husky
ros2 launch fr3_husky_controller fr3_husky_controller.launch.py \
  robot_side:=left load_gripper:=true use_mujoco:=true

# FR3 only (no mobile base)
ros2 launch fr3_husky_controller fr3_controller.launch.py \
  robot_side:=left load_gripper:=true load_mobile:=false use_mujoco:=true
```

### Camera parameters (controller_manager)

| Parameter | Type | Default | Description |
|---|---|---|---|
| `mujoco_camera_publish_rate` | `double` | `30.0` | Camera image publish rate (Hz) |
| `mujoco_camera_publish_pointcloud` | `bool` | `false` | Also publish `PointCloud2` per camera |

```python
cm_params = [
    ...
    {'mujoco_camera_publish_rate': 30.0},
    {'mujoco_camera_publish_pointcloud': False},
]
```

---

## Camera RGBD Publishing

Cameras declared in the MJCF are automatically discovered and published as ROS 2 topics in a separate thread (no physics slowdown). Resolution is read from the MJCF `<sensor type="camera" width="..." height="..."/>` element; cameras without a sensor element default to 640×480.

### Topic layout (per camera)

```
/mujoco_ros_hardware/<camera_name>/color/image_raw          sensor_msgs/Image       (rgb8)
/mujoco_ros_hardware/<camera_name>/color/camera_info        sensor_msgs/CameraInfo
/mujoco_ros_hardware/<camera_name>/depth/image_rect_raw     sensor_msgs/Image       (32FC1, metres)
/mujoco_ros_hardware/<camera_name>/depth/camera_info        sensor_msgs/CameraInfo
/mujoco_ros_hardware/<camera_name>/depth/points             sensor_msgs/PointCloud2  (optional)
```

### MJCF camera definition

Declare a fixed camera inside any body in the MJCF scene:

```xml
<body name="hand">
  <camera name="hand_eye" pos="0.1 0.0 0.0" euler="3.14159 0.218 0"
          mode="fixed" fovy="58"/>
</body>
```

To specify resolution, add a sensor element (must share the same name):

```xml
<sensor name="hand_eye" type="camera" camera="hand_eye" width="640" height="480"/>
```

### Depth image

Depth is metric distance in metres (`32FC1`). Pixels at the far clip plane (no geometry hit) are `NaN`.

The intrinsic matrix assumes MuJoCo's pinhole model (square pixels):

```
fx = fy = (height / 2) / tan(fovy_rad / 2)
cx = width / 2,  cy = height / 2
```

---

## Supported Robot Types

| `robot_type` | Handler | Description |
|---|---|---|
| `franka` | `FrankaSubHandler` | Single FR3 arm |
| `franka_multi` | `FrankaMultiSubHandler` | Multiple FR3 arms (one plugin block) |
| `husky` | `HuskySubHandler` | Clearpath Husky mobile base |
