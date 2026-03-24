#include "mujoco_ros_hardware/mujoco_world_singleton.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <dlfcn.h>
#include <limits>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_client.hpp"

namespace mujoco_ros_hardware
{

MujocoWorldSingleton & MujocoWorldSingleton::get()
{
    static MujocoWorldSingleton instance;
    return instance;
}

MujocoWorldSingleton::~MujocoWorldSingleton()
{
    stopSimulation();
    stopCameraThread();   // must stop before stopViewer destroys offscreen_window_
    stopViewer();
    if (data_)  { mj_deleteData(data_);   data_  = nullptr; }
    if (model_) { mj_deleteModel(model_); model_ = nullptr; }
}

void MujocoWorldSingleton::registerPlugin(int priority)
{
    ++total_plugins_;
    if (priority > max_priority_) max_priority_ = priority;
    RCLCPP_INFO(
        rclcpp::get_logger("MujocoWorldSingleton"),
        "Plugin registered (total: %d, priority: %d, max: %d)",
        total_plugins_, priority, max_priority_);
}

bool MujocoWorldSingleton::init()
{
    if (initialized_) return true;

    // Load MuJoCo decoder plugins (e.g. obj_decoder for .obj mesh support in MuJoCo 3.5+).
    {
        Dl_info dl_info;
        if (dladdr(reinterpret_cast<void*>(mj_version), &dl_info) && dl_info.dli_fname) {
            std::string mujoco_lib_dir(dl_info.dli_fname);
            mujoco_lib_dir = mujoco_lib_dir.substr(0, mujoco_lib_dir.find_last_of('/'));
            const std::string plugin_dir = mujoco_lib_dir + "/../bin/mujoco_plugin";
            mj_loadAllPluginLibraries(plugin_dir.c_str(), nullptr);
        }
    }

    // Create a temporary node to read parameters from controller_manager
    rclcpp::NodeOptions opts;
    opts.allow_undeclared_parameters(true);
    auto tmp = rclcpp::Node::make_shared("_mujoco_singleton_init", opts);
    rclcpp::SyncParametersClient cli(tmp, "controller_manager");

    if (!cli.wait_for_service(std::chrono::seconds(5)))
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("MujocoWorldSingleton"),
            "\033[31mcontroller_manager parameter service not available within 5s\033[0m");
        return false;
    }

    const auto params = cli.get_parameters({
        "mujoco_scene_path",
        "mujoco_scene_xacro_path",
        "mujoco_scene_xacro_args",
        "mujoco_camera_publish_rate",
        "mujoco_camera_publish_pointcloud"
    });
    for (const auto & p : params)
    {
        if (p.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) continue;
        if      (p.get_name() == "mujoco_scene_path")              xml_path_                = p.as_string();
        else if (p.get_name() == "mujoco_scene_xacro_path")        xacro_path_              = p.as_string();
        else if (p.get_name() == "mujoco_scene_xacro_args")        xacro_base_args_         = p.as_string();
        else if (p.get_name() == "mujoco_camera_publish_rate")     camera_publish_rate_hz_  = p.as_double();
        else if (p.get_name() == "mujoco_camera_publish_pointcloud") publish_pointcloud_    = p.as_bool();
    }

    if (xml_path_.empty() && xacro_path_.empty())
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("MujocoWorldSingleton"),
            "\033[31mROS param 'mujoco_scene_path' and 'mujoco_scene_xacro_path' not set on controller_manager\033[0m");
        return false;
    }

    if (!xml_path_.empty() && (access(xml_path_.c_str(), F_OK) == -1))
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("MujocoWorldSingleton"),
            "\033[31mXML file (mujoco_scene_path) not found: '%s'\033[0m", xml_path_.c_str());
        return false;
    }

    if (!xacro_path_.empty() && (access(xacro_path_.c_str(), F_OK) == -1))
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("MujocoWorldSingleton"),
            "\033[31mXML.xacro file (mujoco_scene_xacro_path) not found: '%s'\033[0m", xacro_path_.c_str());
        return false;
    }

    initialized_ = true;
    if (!xml_path_.empty())
    {
        RCLCPP_INFO(
            rclcpp::get_logger("MujocoWorldSingleton"),
            "\033[34mInit: xml_path=%s\033[0m", xml_path_.c_str());
        return true;
    }
    RCLCPP_INFO(
        rclcpp::get_logger("MujocoWorldSingleton"),
        "\033[34mInit: xacro_path=%s  base_args=[%s]\033[0m",
        xacro_path_.c_str(), xacro_base_args_.c_str());
    return true;
}

bool MujocoWorldSingleton::loadSceneFromPath(const std::string & xml_path)
{
    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        if (data_)  { mj_deleteData(data_);   data_  = nullptr; }
        if (model_) { mj_deleteModel(model_); model_ = nullptr; }

        char err[1000] = {};
        model_ = mj_loadXML(xml_path.c_str(), nullptr, err, sizeof(err));
        if (!model_)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("MujocoWorldSingleton"),
                "\033[31mmj_loadXML failed:  %s\033[0m", err);
            return false;
        }

        data_ = mj_makeData(model_);
        mj_forward(model_, data_);
        scene_loaded_ = true;
    }

    RCLCPP_INFO(
        rclcpp::get_logger("MujocoWorldSingleton"),
        "\033[34mScene loaded: nq=%d nv=%d nu=%d njnt=%d ncam=%d\033[0m",
        model_->nq, model_->nv, model_->nu, model_->njnt, model_->ncam);

    discoverCameras();
    startViewer();
    startCameraThread();
    startSimulation();
    return true;
}

bool MujocoWorldSingleton::loadSceneFromXML(const std::string & xml_string)
{
    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        if (data_)  { mj_deleteData(data_);   data_  = nullptr; }
        if (model_) { mj_deleteModel(model_); model_ = nullptr; }

        char err[1000] = {};
        mjSpec * spec = mj_parseXMLString(xml_string.c_str(), nullptr, err, sizeof(err));
        if (!spec)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("MujocoWorldSingleton"),
                "\033[31mmj_parseXMLString failed: %s\033[0m", err);
            return false;
        }

        model_ = mj_compile(spec, nullptr);
        mj_deleteSpec(spec);
        if (!model_)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("MujocoWorldSingleton"),
                "\033[31mmj_compile failed\033[0m");
            return false;
        }

        data_ = mj_makeData(model_);
        mj_forward(model_, data_);
        scene_loaded_ = true;
    }

    RCLCPP_INFO(
        rclcpp::get_logger("MujocoWorldSingleton"),
        "\033[34mScene loaded: nq=%d nv=%d nu=%d njnt=%d ncam=%d\033[0m",
        model_->nq, model_->nv, model_->nu, model_->njnt, model_->ncam);

    discoverCameras();
    startViewer();
    startCameraThread();
    startSimulation();
    return true;
}

// ---------------------------------------------------------------------------
// Camera RGBD publishing
// ---------------------------------------------------------------------------

void MujocoWorldSingleton::discoverCameras()
{
    cameras_.clear();

    if (!model_ || model_->ncam == 0)
    {
        RCLCPP_INFO(rclcpp::get_logger("MujocoWorldSingleton"), "No cameras in MJCF.");
        return;
    }

    cam_node_ = rclcpp::Node::make_shared("mujoco_camera_publisher");

    int max_w = 0, max_h = 0;

    for (int i = 0; i < model_->ncam; ++i)
    {
        const char * raw_name = mj_id2name(model_, mjOBJ_CAMERA, i);
        if (!raw_name) continue;

        CameraDesc cam;
        cam.id       = i;
        cam.name     = raw_name;
        // cam_resolution[2*i] / [2*i+1]: set by <sensor type="camera" width/height>;
        // default to 640×480 when not specified (value is 0).
        cam.width    = (model_->cam_resolution[2 * i]     > 0) ? model_->cam_resolution[2 * i]     : 640;
        cam.height   = (model_->cam_resolution[2 * i + 1] > 0) ? model_->cam_resolution[2 * i + 1] : 480;
        cam.fovy_deg = model_->cam_fovy[i];

        max_w = std::max(max_w, cam.width);
        max_h = std::max(max_h, cam.height);

        // Topic layout mirrors RealSense / standard ROS2 camera drivers:
        //   /mujoco_ros_hardware/<name>/color/image_raw
        //   /mujoco_ros_hardware/<name>/color/camera_info
        //   /mujoco_ros_hardware/<name>/depth/image_rect_raw   (32FC1, meters)
        //   /mujoco_ros_hardware/<name>/depth/camera_info
        //   /mujoco_ros_hardware/<name>/depth/points           (optional)
        const std::string base = "/mujoco_ros_hardware/" + cam.name;
        const auto qos = rclcpp::QoS(1);

        cam.rgb_pub        = cam_node_->create_publisher<sensor_msgs::msg::Image>     (base + "/color/image_raw",       qos);
        cam.depth_pub      = cam_node_->create_publisher<sensor_msgs::msg::Image>     (base + "/depth/image_rect_raw",  qos);
        cam.rgb_info_pub   = cam_node_->create_publisher<sensor_msgs::msg::CameraInfo>(base + "/color/camera_info",     qos);
        cam.depth_info_pub = cam_node_->create_publisher<sensor_msgs::msg::CameraInfo>(base + "/depth/camera_info",     qos);

        if (publish_pointcloud_)
            cam.cloud_pub = cam_node_->create_publisher<sensor_msgs::msg::PointCloud2>(base + "/depth/points", qos);

        cam.camera_info_msg = buildCameraInfoMsg(cam);

        RCLCPP_INFO(rclcpp::get_logger("MujocoWorldSingleton"),
            "\033[34mCamera '%s' (id=%d): %dx%d  fovy=%.1f°\033[0m",
            cam.name.c_str(), cam.id, cam.width, cam.height, cam.fovy_deg);

        cameras_.push_back(std::move(cam));
    }

    if (!cameras_.empty())
    {
        offscreen_width_  = max_w;
        offscreen_height_ = max_h;

        // Tell MuJoCo the required offscreen FBO dimensions BEFORE mjr_makeContext
        // is called in the camera thread.  mjr_makeContext creates the offscreen FBO
        // sized to model->vis.global.offwidth × offheight; if these are 0 (or the
        // compiled-in default 640×480 happens to be too small), the FBO will not
        // fit our viewport and mjr_setBuffer(mjFB_OFFSCREEN) falls back to the
        // window framebuffer (causing solid-colour or black images).
        model_->vis.global.offwidth  = max_w;
        model_->vis.global.offheight = max_h;

        RCLCPP_INFO(rclcpp::get_logger("MujocoWorldSingleton"),
            "\033[34mCamera offscreen FBO: %dx%d for %zu camera(s)  |  rate=%.1f Hz  pointcloud=%s\033[0m",
            max_w, max_h, cameras_.size(),
            camera_publish_rate_hz_, publish_pointcloud_ ? "yes" : "no");
    }
}

sensor_msgs::msg::CameraInfo
MujocoWorldSingleton::buildCameraInfoMsg(const CameraDesc & cam) const
{
    // MuJoCo cameras are pinhole with square pixels.
    // fy = (height/2) / tan(fovy/2),  fx = fy
    const double fovy_rad = cam.fovy_deg * M_PI / 180.0;
    const double fy       = (cam.height / 2.0) / std::tan(fovy_rad / 2.0);
    const double fx       = fy;
    const double cx       = cam.width  / 2.0;
    const double cy       = cam.height / 2.0;

    sensor_msgs::msg::CameraInfo info;
    info.width  = static_cast<uint32_t>(cam.width);
    info.height = static_cast<uint32_t>(cam.height);
    info.distortion_model = "plumb_bob";
    info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
    info.k = {fx,  0.0, cx,
              0.0, fy,  cy,
              0.0, 0.0, 1.0};
    info.r = {1.0, 0.0, 0.0,
              0.0, 1.0, 0.0,
              0.0, 0.0, 1.0};
    info.p = {fx,  0.0, cx,  0.0,
              0.0, fy,  cy,  0.0,
              0.0, 0.0, 1.0, 0.0};
    info.header.frame_id = cam.name + "_optical_frame";
    return info;
}

void MujocoWorldSingleton::startCameraThread()
{
    if (cameras_.empty())
    {
        RCLCPP_INFO(rclcpp::get_logger("MujocoWorldSingleton"),
            "No cameras found – camera thread not started.");
        return;
    }
    if (cam_running_.load()) return;

    if (!offscreen_window_)
    {
        RCLCPP_WARN(rclcpp::get_logger("MujocoWorldSingleton"),
            "\033[33mOffscreen GLFW window not available – camera thread not started.\033[0m");
        return;
    }

    cam_running_.store(true);

    camera_thread_ = std::thread([this]()
    {
        // Claim the offscreen OpenGL context on this thread.
        glfwMakeContextCurrent(offscreen_window_);

        // ---- MuJoCo rendering objects ----
        mjvScene   scn;
        mjvOption  vopt;
        mjvPerturb pert;
        mjrContext con;

        mjv_defaultScene(&scn);
        mjv_makeScene(model_, &scn, 20000);
        mjv_defaultOption(&vopt);
        mjv_defaultPerturb(&pert);
        mjr_defaultContext(&con);
        mjr_makeContext(model_, &con, mjFONTSCALE_100);

        // Switch to MuJoCo's offscreen FBO.  discoverCameras() already set
        // model->vis.global.offwidth/offheight = max camera resolution, so
        // mjr_makeContext created the FBO at the right size (con.offFBO != 0).
        mjr_setBuffer(mjFB_OFFSCREEN, &con);

        RCLCPP_INFO(rclcpp::get_logger("MujocoWorldSingleton"),
            "\033[34mCamera thread ready (offscreen FBO %dx%d, offFBO=%u)\033[0m",
            con.offWidth, con.offHeight, con.offFBO);

        // ---- Depth buffer → metric distance ----
        // MuJoCo depth values are normalised [0,1] in the hyperbolic z-buffer.
        // Formula: dist = znear*zfar / (zfar - buf*(zfar-znear))
        const double znear = model_->vis.map.znear * model_->stat.extent;
        const double zfar  = model_->vis.map.zfar  * model_->stat.extent;

        const auto period = std::chrono::duration<double>(1.0 / camera_publish_rate_hz_);
        auto next = std::chrono::steady_clock::now();

        while (cam_running_.load())
        {
            next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);

            // ---- Snapshot mjData (brief lock; rendering happens without lock) ----
            mjData * d_snap = nullptr;
            {
                std::lock_guard<std::mutex> lk(data_mutex_);
                d_snap = mj_copyData(nullptr, model_, data_);
            }

            const auto stamp = cam_node_->now();

            for (auto & cam : cameras_)
            {
                const int npixels = cam.width * cam.height;

                // ---- Render ----
                mjvCamera mjcam;
                mjv_defaultCamera(&mjcam);
                mjcam.type       = mjCAMERA_FIXED;
                mjcam.fixedcamid = cam.id;

                const mjrRect vp = {0, 0, cam.width, cam.height};
                mjv_updateScene(model_, d_snap, &vopt, &pert, &mjcam, mjCAT_ALL, &scn);
                mjr_render(vp, &scn, &con);

                // ---- Read pixels (OpenGL: origin at bottom-left) ----
                std::vector<uint8_t> rgb_buf(npixels * 3);
                std::vector<float>   depth_buf(npixels);
                mjr_readPixels(rgb_buf.data(), depth_buf.data(), vp, &con);

                // ---- Flip vertically: OpenGL bottom-up → ROS top-down ----
                for (int row = 0; row < cam.height / 2; ++row)
                {
                    uint8_t * top_rgb = rgb_buf.data()   + row                       * cam.width * 3;
                    uint8_t * bot_rgb = rgb_buf.data()   + (cam.height - 1 - row)    * cam.width * 3;
                    std::swap_ranges(top_rgb, top_rgb + cam.width * 3, bot_rgb);

                    float * top_d = depth_buf.data() + row                    * cam.width;
                    float * bot_d = depth_buf.data() + (cam.height - 1 - row) * cam.width;
                    std::swap_ranges(top_d, top_d + cam.width, bot_d);
                }

                // ---- Convert depth buffer [0,1] → metric distance (meters, float32) ----
                std::vector<float> depth_m(npixels);
                for (int k = 0; k < npixels; ++k)
                {
                    const float buf = depth_buf[k];
                    depth_m[k] = (buf >= 1.0f - 1e-6f)
                        ? std::numeric_limits<float>::quiet_NaN()
                        : static_cast<float>(znear * zfar / (zfar - buf * (zfar - znear)));
                }

                // ---- Build header ----
                std_msgs::msg::Header hdr;
                hdr.stamp    = stamp;
                hdr.frame_id = cam.name + "_optical_frame";

                // ---- PointCloud2 (optional, before moving rgb_buf) ----
                if (publish_pointcloud_ && cam.cloud_pub)
                {
                    const double fovy_rad = cam.fovy_deg * M_PI / 180.0;
                    const double fy_px    = (cam.height / 2.0) / std::tan(fovy_rad / 2.0);
                    const double fx_px    = fy_px;
                    const double cx_px    = cam.width  / 2.0;
                    const double cy_px    = cam.height / 2.0;

                    // Organised cloud (height × width) with NaN for invalid pixels.
                    sensor_msgs::msg::PointCloud2 cloud;
                    cloud.header     = hdr;
                    cloud.height     = static_cast<uint32_t>(cam.height);
                    cloud.width      = static_cast<uint32_t>(cam.width);
                    cloud.is_dense   = false;
                    cloud.is_bigendian = false;

                    // Fields: x(4) y(4) z(4) rgb(4)  → 16 bytes/point
                    auto make_field = [](const std::string & name, uint32_t offset, uint8_t type) {
                        sensor_msgs::msg::PointField f;
                        f.name     = name;
                        f.offset   = offset;
                        f.datatype = type;
                        f.count    = 1;
                        return f;
                    };
                    cloud.fields = {
                        make_field("x",   0,  sensor_msgs::msg::PointField::FLOAT32),
                        make_field("y",   4,  sensor_msgs::msg::PointField::FLOAT32),
                        make_field("z",   8,  sensor_msgs::msg::PointField::FLOAT32),
                        make_field("rgb", 12, sensor_msgs::msg::PointField::FLOAT32),
                    };
                    cloud.point_step = 16;
                    cloud.row_step   = cloud.point_step * cloud.width;
                    cloud.data.resize(static_cast<std::size_t>(cloud.row_step) * cloud.height);

                    for (int v = 0; v < cam.height; ++v)
                    {
                        for (int u = 0; u < cam.width; ++u)
                        {
                            float * pt = reinterpret_cast<float *>(
                                cloud.data.data() + (v * cam.width + u) * 16);

                            const float z = depth_m[v * cam.width + u];
                            if (!std::isfinite(z) || z <= 0.0f)
                            {
                                const float nan = std::numeric_limits<float>::quiet_NaN();
                                pt[0] = pt[1] = pt[2] = nan;
                                pt[3] = 0.0f;
                            }
                            else
                            {
                                pt[0] = static_cast<float>((u - cx_px) * z / fx_px);
                                pt[1] = static_cast<float>((v - cy_px) * z / fy_px);
                                pt[2] = z;

                                // Pack R,G,B into the float's bits (ROS convention)
                                const uint8_t r = rgb_buf[(v * cam.width + u) * 3 + 0];
                                const uint8_t g = rgb_buf[(v * cam.width + u) * 3 + 1];
                                const uint8_t b = rgb_buf[(v * cam.width + u) * 3 + 2];
                                const uint32_t rgb_packed =
                                    (static_cast<uint32_t>(r) << 16) |
                                    (static_cast<uint32_t>(g) <<  8) |
                                     static_cast<uint32_t>(b);
                                std::memcpy(&pt[3], &rgb_packed, sizeof(float));
                            }
                        }
                    }
                    cam.cloud_pub->publish(cloud);
                }

                // ---- Publish RGB image (move buffer – do after point cloud use) ----
                {
                    sensor_msgs::msg::Image img;
                    img.header   = hdr;
                    img.width    = static_cast<uint32_t>(cam.width);
                    img.height   = static_cast<uint32_t>(cam.height);
                    img.encoding = "rgb8";
                    img.step     = static_cast<uint32_t>(cam.width * 3);
                    img.data     = std::move(rgb_buf);
                    cam.rgb_pub->publish(img);
                }

                // ---- Publish Depth image (32FC1, meters) ----
                {
                    sensor_msgs::msg::Image img;
                    img.header   = hdr;
                    img.width    = static_cast<uint32_t>(cam.width);
                    img.height   = static_cast<uint32_t>(cam.height);
                    img.encoding = "32FC1";
                    img.step     = static_cast<uint32_t>(cam.width * sizeof(float));
                    img.data.resize(static_cast<std::size_t>(npixels) * sizeof(float));
                    std::memcpy(img.data.data(), depth_m.data(), img.data.size());
                    cam.depth_pub->publish(img);
                }

                // ---- Publish CameraInfo (same intrinsics for colour and depth) ----
                {
                    auto info    = cam.camera_info_msg;
                    info.header  = hdr;
                    cam.rgb_info_pub->publish(info);
                    cam.depth_info_pub->publish(info);
                }
            }  // for cameras_

            mj_deleteData(d_snap);

            std::this_thread::sleep_until(next);
        }  // while cam_running_

        // ---- Cleanup ----
        mjv_freeScene(&scn);
        mjr_freeContext(&con);
        glfwMakeContextCurrent(nullptr);  // release context so render_thread_ can destroy it
    });

    RCLCPP_INFO(rclcpp::get_logger("MujocoWorldSingleton"),
        "\033[34mCamera thread started (%.1f Hz, %zu camera(s))\033[0m",
        camera_publish_rate_hz_, cameras_.size());
}

void MujocoWorldSingleton::stopCameraThread()
{
    if (!cam_running_.load()) return;
    cam_running_.store(false);
    if (camera_thread_.joinable()) camera_thread_.join();
}

// ---------------------------------------------------------------------------
// Simulation thread
// ---------------------------------------------------------------------------

void MujocoWorldSingleton::startSimulation()
{
    if (sim_running_.load() || !model_ || !data_) return;
    sim_running_.store(true);

    sim_thread_ = std::thread([this]()
    {
        const double step_sec =
            (model_->opt.timestep > 0.0) ? model_->opt.timestep : 0.001;

        auto next      = std::chrono::steady_clock::now();
        auto last_sync = next;
        const auto sync_period = std::chrono::duration<double>(1.0 / 60.0);

        while (sim_running_.load())
        {
            next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(step_sec));

            if (sim_)
            {
                const std::unique_lock<std::recursive_mutex> sim_lk(sim_->mtx);
                {
                    std::lock_guard<std::mutex> data_lk(data_mutex_);
                    mj_step(model_, data_);
                }
                const auto now = std::chrono::steady_clock::now();
                if (now - last_sync >= sync_period)
                {
                    sim_->Sync(false);
                    last_sync = now;
                }
            }
            else
            {
                std::lock_guard<std::mutex> lk(data_mutex_);
                mj_step(model_, data_);
            }

            std::this_thread::sleep_until(next);
        }
    });

    RCLCPP_INFO(rclcpp::get_logger("MujocoWorldSingleton"),
        "\033[34mSimulation thread started (dt=%.4f s)\033[0m", model_->opt.timestep);
}

void MujocoWorldSingleton::stopSimulation()
{
    if (!sim_running_.load()) return;
    sim_running_.store(false);
    if (sim_thread_.joinable()) sim_thread_.join();
}

// ---------------------------------------------------------------------------
// Viewer (passive GLFW window)
// ---------------------------------------------------------------------------

void MujocoWorldSingleton::startViewer()
{
    mjv_defaultCamera(&cam_);
    mjv_defaultOption(&opt_);
    mjv_defaultPerturb(&pert_);

    {
        std::lock_guard<std::mutex> lk(sim_ready_mtx_);
        sim_ready_ = false;
    }

    render_thread_ = std::thread([this]()
    {
        auto sim = std::make_unique<mujoco::Simulate>(
            std::make_unique<mujoco::GlfwAdapter>(),
            &cam_, &opt_, &pert_, /*is_passive=*/true);

        // Create a tiny hidden window just to get an OpenGL context for the
        // camera thread.  The actual render target is MuJoCo's offscreen FBO
        // (mjFB_OFFSCREEN), whose size is set via model->vis.global.offwidth/
        // offheight in discoverCameras(), so the window dimensions don't matter.
        glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
        offscreen_window_ = glfwCreateWindow(1, 1, "mujoco_offscreen", nullptr, nullptr);
        glfwDefaultWindowHints();  // restore defaults

        if (!offscreen_window_)
        {
            RCLCPP_WARN(rclcpp::get_logger("MujocoWorldSingleton"),
                "\033[33mglfwCreateWindow (offscreen) failed – camera publishing disabled\033[0m");
        }

        {
            std::lock_guard<std::mutex> lk(sim_ready_mtx_);
            sim_ = std::move(sim);
            sim_ready_ = true;
        }
        sim_ready_cv_.notify_one();

        sim_->RenderLoop();

        // Camera thread is always stopped before this point (destructor order),
        // so it is safe to destroy the offscreen window here.
        if (offscreen_window_)
        {
            glfwDestroyWindow(offscreen_window_);
            offscreen_window_ = nullptr;
        }
    });

    {
        std::unique_lock<std::mutex> lk(sim_ready_mtx_);
        sim_ready_cv_.wait(lk, [this]() { return sim_ready_; });
    }

    sim_->Load(model_, data_, "mujoco_scene");
    RCLCPP_INFO(rclcpp::get_logger("MujocoWorldSingleton"), "\033[34mViewer started\033[0m");
}

void MujocoWorldSingleton::stopViewer()
{
    if (!sim_) return;
    sim_->exitrequest.store(1);
    if (render_thread_.joinable()) render_thread_.join();
    sim_.reset();
}

}  // namespace mujoco_ros_hardware
