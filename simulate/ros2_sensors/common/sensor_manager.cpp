#include "common/sensor_manager.hpp"
#include "camera/camera_sensor.hpp"
#include "lidar/lidar_sensor.hpp"
#include "state/odometry_publisher.hpp"
#include "common/offscreen_context.hpp"

#include <cmath>
#include <cstdlib>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <yaml-cpp/yaml.h>

namespace ros2_sensors {

struct SensorManager::Impl {
  bool enabled = false;
  double clock_rate = 100.0;
  double last_clock = -std::numeric_limits<double>::infinity();
  double last_time = -std::numeric_limits<double>::infinity();
  YAML::Node config;
  rclcpp::Context::SharedPtr context;
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf;
  std::vector<std::unique_ptr<Sensor>> sensors;
};

SensorManager::SensorManager(const std::string& config_file, int unitree_domain_id)
    : impl_(std::make_unique<Impl>()) {
  auto& state = *impl_;
  state.config = YAML::LoadFile(config_file);
  state.enabled = state.config["enabled"].as<bool>(true);
  if (!state.enabled) return;
  const int ros_domain = state.config["ros_domain_id"].as<int>(10);
  if (unitree_domain_id != 0 || ros_domain != 10) {
    throw std::runtime_error("Sensor integration requires Unitree DDS domain 0 and ROS 2 domain 10");
  }
  const char* rmw = std::getenv("RMW_IMPLEMENTATION");
  if (rmw && std::string(rmw) != "rmw_fastrtps_cpp") {
    throw std::runtime_error("Use RMW_IMPLEMENTATION=rmw_fastrtps_cpp for the embedded ROS sensors");
  }
  setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp", 1);
  state.clock_rate = state.config["clock_publish_rate"].as<double>(100.0);
  if (!std::isfinite(state.clock_rate) || state.clock_rate <= 0.0) {
    throw std::runtime_error("clock_publish_rate must be positive and finite");
  }
  state.context = std::make_shared<rclcpp::Context>();
  rclcpp::InitOptions init_options;
  init_options.set_domain_id(10);  // Explicitly independent of ROS_DOMAIN_ID and SDK2.
  state.context->init(0, nullptr, init_options);
  rclcpp::NodeOptions options;
  options.context(state.context).use_global_arguments(false)
      .start_parameter_services(false).start_parameter_event_publisher(false);
  state.node = std::make_shared<rclcpp::Node>("unitree_sim_sensors", options);
  state.clock_pub = state.node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
  RCLCPP_INFO(state.node->get_logger(), "Unitree DDS domain=0; ROS 2 domain=10; RMW=rmw_fastrtps_cpp");
}

SensorManager::~SensorManager() {
  unload_model();
  // Keep the shared EGL display initialized across model reloads. Some Mesa
  // drivers cannot safely unload/reinitialize their LLVM backend in-process.
  OffscreenContext::release_displays();
  impl_->clock_pub.reset();
  impl_->node.reset();
  if (impl_->context && impl_->context->is_valid()) impl_->context->shutdown("Simulator stopped");
}

void SensorManager::load_model(const mjModel* model, const mjData* data) {
  unload_model();
  auto& state = *impl_;
  if (!state.enabled) return;
  state.tf = std::make_unique<tf2_ros::StaticTransformBroadcaster>(state.node);
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  for (const auto& entry : state.config["cameras"]) {
    if (!entry["enabled"].as<bool>(true)) continue;
    CameraConfig camera;
    camera.name = entry["name"].as<std::string>();
    if (mj_name2id(model, mjOBJ_CAMERA, camera.name.c_str()) < 0) {
      RCLCPP_WARN(state.node->get_logger(), "Model has no camera '%s'; skipping it", camera.name.c_str());
      continue;
    }
    camera.frame_id = entry["frame_id"].as<std::string>(camera.name + "_optical_frame");
    camera.publish_rate = entry["publish_rate"].as<double>(15.0);
    camera.color_topic = entry["color_topic"].as<std::string>("/camera/color/image_raw");
    camera.depth_topic = entry["depth_topic"].as<std::string>("/camera/depth/image_raw");
    camera.info_topic = entry["info_topic"].as<std::string>("/camera/camera_info");
    auto sensor = std::make_unique<CameraSensor>(state.node, model, std::move(camera));
    transforms.push_back(sensor->static_transform());
    state.sensors.push_back(std::move(sensor));
  }
  for (const auto& entry : state.config["lidars"]) {
    if (!entry["enabled"].as<bool>(true)) continue;
    LidarConfig lidar;
    lidar.name = entry["name"].as<std::string>();
    lidar.site = entry["site"].as<std::string>(lidar.name);
    if (mj_name2id(model, mjOBJ_SITE, lidar.site.c_str()) < 0) {
      RCLCPP_WARN(state.node->get_logger(), "Model has no lidar site '%s'; skipping it", lidar.site.c_str());
      continue;
    }
    lidar.frame_id = entry["frame_id"].as<std::string>(lidar.name + "_frame");
    lidar.topic = entry["topic"].as<std::string>(lidar.topic);
    lidar.publish_rate = entry["publish_rate"].as<double>(lidar.publish_rate);
    lidar.range_min = entry["range_min"].as<double>(lidar.range_min);
    lidar.range_max = entry["range_max"].as<double>(lidar.range_max);
    if (entry["resolution"]) lidar.resolution = entry["resolution"].as<std::array<int, 2>>();
    if (entry["azimuth_range"]) lidar.azimuth_range = entry["azimuth_range"].as<std::array<double, 2>>();
    if (entry["elevation_range"]) lidar.elevation_range = entry["elevation_range"].as<std::array<double, 2>>();
    if (entry["geom_groups"]) lidar.geom_groups = entry["geom_groups"].as<std::vector<int>>();
    auto sensor = std::make_unique<LidarSensor>(state.node, model, std::move(lidar));
    transforms.push_back(sensor->static_transform());
    state.sensors.push_back(std::move(sensor));
  }
  if (!transforms.empty()) state.tf->sendTransform(transforms);
  const auto odometry = state.config["odometry"];
  if (odometry && odometry["enabled"].as<bool>(true)) {
    OdometryConfig config;
    config.free_joint = odometry["free_joint"].as<std::string>(config.free_joint);
    config.topic = odometry["topic"].as<std::string>(config.topic);
    config.frame_id = odometry["frame_id"].as<std::string>(config.frame_id);
    config.publish_rate = odometry["publish_rate"].as<double>(config.publish_rate);
    config.publish_tf = odometry["publish_tf"].as<bool>(config.publish_tf);
    if (mj_name2id(model, mjOBJ_JOINT, config.free_joint.c_str()) < 0) {
      RCLCPP_WARN(state.node->get_logger(), "Model has no odometry joint '%s'; skipping it",
          config.free_joint.c_str());
    } else {
      state.sensors.push_back(std::make_unique<OdometryPublisher>(state.node, model, std::move(config)));
    }
  }
  update(model, data);
}

void SensorManager::update(const mjModel* model, const mjData* data) {
  auto& state = *impl_;
  if (!state.enabled) return;
  if (data->time < state.last_time) reset();
  state.last_time = data->time;
  if (data->time - state.last_clock + 1e-9 >= 1.0 / state.clock_rate) {
    rosgraph_msgs::msg::Clock clock;
    clock.clock = rclcpp::Time(static_cast<int64_t>(std::llround(data->time * 1e9)), RCL_ROS_TIME);
    state.clock_pub->publish(clock);
    state.last_clock = data->time;
  }
  for (auto& sensor : state.sensors) sensor->sample(model, data);
}

void SensorManager::reset() {
  impl_->last_time = impl_->last_clock = -std::numeric_limits<double>::infinity();
  for (auto& sensor : impl_->sensors) sensor->reset();
}

void SensorManager::unload_model() {
  for (auto& sensor : impl_->sensors) sensor->stop();
  impl_->sensors.clear();
  impl_->tf.reset();
  reset();
}

}  // namespace ros2_sensors
