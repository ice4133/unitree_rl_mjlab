// Adapted from mujoco_ros2_control's 3dlidar.cpp and mujoco_3d_lidar_plugin.cpp.
// Copyright (c) 2026, United States Government, as represented by the
// Administrator of the National Aeronautics and Space Administration.
// Modified: standalone Sensor lifecycle, private worker snapshots, MuJoCo 3.3.6,
// validated configuration, ROS sensor QoS, reset handling, and full-circle seam.
// Licensed under the Apache License, Version 2.0. See ../LICENSE.
#include "lidar/lidar_sensor.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace ros2_sensors {
namespace {
constexpr double kPi = 3.14159265358979323846;

bool valid_angles(const std::array<double, 2>& angles, double limit, int count) {
  return std::isfinite(angles[0]) && std::isfinite(angles[1]) &&
         angles[0] >= -limit && angles[1] <= limit &&
         (count == 1 ? angles[0] <= angles[1] : angles[0] < angles[1]);
}
}  // namespace

LidarSensor::LidarSensor(rclcpp::Node::SharedPtr node, const mjModel* model, LidarConfig config)
    : node_(std::move(node)), model_(model), config_(std::move(config)) {
  site_id_ = mj_name2id(model_, mjOBJ_SITE, config_.site.c_str());
  if (site_id_ < 0) throw std::runtime_error("Lidar site not found in MJCF: " + config_.site);
  const auto& resolution = config_.resolution;
  if (resolution[0] < 1 || resolution[1] < 1 ||
      static_cast<int64_t>(resolution[0]) * resolution[1] > 1000000 ||
      !valid_angles(config_.azimuth_range, kPi, resolution[0]) ||
      !valid_angles(config_.elevation_range, kPi / 2, resolution[1]) ||
      !std::isfinite(config_.range_min) || !std::isfinite(config_.range_max) ||
      config_.range_min < 0 || config_.range_max <= config_.range_min ||
      !std::isfinite(config_.publish_rate) || config_.publish_rate <= 0 ||
      config_.frame_id.empty() || config_.topic.empty()) {
    throw std::runtime_error("Invalid lidar resolution/angles/range/rate/frame/topic: " + config_.name);
  }
  for (int group : config_.geom_groups) {
    if (group < 0 || group >= mjNGROUP) throw std::runtime_error("Lidar geom groups must be in [0, 5]");
    geom_mask_[group] = 1;
  }
  num_rays_ = resolution[0] * resolution[1];
  vectors_.reserve(static_cast<size_t>(num_rays_) * 3);
  // Same elevation-major unit-vector grid as upstream. Do not duplicate -pi/+pi
  // when the configured azimuth covers a complete circle.
  const bool full_circle = std::abs(config_.azimuth_range[1] - config_.azimuth_range[0] - 2 * kPi) < 1e-9;
  for (int e = 0; e < resolution[1]; ++e) {
    const double elevation = config_.elevation_range[0] +
        (resolution[1] > 1 ? e * (config_.elevation_range[1] - config_.elevation_range[0]) / (resolution[1] - 1) : 0);
    for (int a = 0; a < resolution[0]; ++a) {
      const double azimuth = config_.azimuth_range[0] +
          (resolution[0] > 1 ? a * (config_.azimuth_range[1] - config_.azimuth_range[0]) /
              (full_circle ? resolution[0] : resolution[0] - 1) : 0);
      vectors_.push_back(std::cos(azimuth) * std::cos(elevation));
      vectors_.push_back(std::sin(azimuth) * std::cos(elevation));
      vectors_.push_back(std::sin(elevation));
    }
  }
  world_vectors_.resize(vectors_.size());
  ranges_.resize(num_rays_);
  geom_ids_.resize(num_rays_);

  sensor_msgs::PointCloud2Modifier modifier(cloud_);
  modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32);
  cloud_.header.frame_id = config_.frame_id;
  cloud_.width = resolution[0];
  cloud_.height = resolution[1];
  cloud_.row_step = cloud_.point_step * cloud_.width;
  cloud_.data.resize(static_cast<size_t>(cloud_.row_step) * cloud_.height);
  const uint16_t endian = 1;
  cloud_.is_bigendian = (*reinterpret_cast<const unsigned char*>(&endian) == 0);
  cloud_.is_dense = false;
  publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      config_.topic, rclcpp::SensorDataQoS().keep_last(1));
  pending_data_.reset(mj_makeData(model_));
  scan_data_.reset(mj_makeData(model_));
  if (!pending_data_ || !scan_data_) throw std::runtime_error("Cannot allocate lidar snapshots");
  // Validate the parent frame before launching a thread.
  static_transform();
  worker_ = std::thread(&LidarSensor::scan_loop, this);
  RCLCPP_INFO(node_->get_logger(), "Lidar '%s': %dx%d rays, %.1f Hz, topic '%s', frame '%s'",
      config_.name.c_str(), resolution[0], resolution[1], config_.publish_rate,
      config_.topic.c_str(), config_.frame_id.c_str());
}

LidarSensor::~LidarSensor() { stop(); }

void LidarSensor::sample(const mjModel* model, const mjData* data) {
  if (stopping_ || data->time - last_sample_time_ + 1e-9 < 1.0 / config_.publish_rate) return;
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (!lock.owns_lock()) return;
  // Both snapshots are exclusively owned. The worker never reads live mjData.
  mj_copyData(pending_data_.get(), model, data);
  pending_generation_ = generation_;
  pending_ = true;
  last_sample_time_ = data->time;
  lock.unlock();
  cv_.notify_one();
}

void LidarSensor::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  ++generation_;
  pending_ = false;
  last_sample_time_ = -std::numeric_limits<double>::infinity();
}

void LidarSensor::stop() {
  stopping_ = true;
  cv_.notify_one();
  if (worker_.joinable()) worker_.join();
}

geometry_msgs::msg::TransformStamped LidarSensor::static_transform() const {
  const int body = model_->site_bodyid[site_id_];
  const char* parent = mj_id2name(model_, mjOBJ_BODY, body);
  if (!parent) throw std::runtime_error("Lidar parent body must have a name");
  if (config_.frame_id == parent) throw std::runtime_error("Lidar frame must differ from its parent body");
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = parent;
  transform.child_frame_id = config_.frame_id;
  transform.transform.translation.x = model_->site_pos[3 * site_id_];
  transform.transform.translation.y = model_->site_pos[3 * site_id_ + 1];
  transform.transform.translation.z = model_->site_pos[3 * site_id_ + 2];
  transform.transform.rotation.w = model_->site_quat[4 * site_id_];
  transform.transform.rotation.x = model_->site_quat[4 * site_id_ + 1];
  transform.transform.rotation.y = model_->site_quat[4 * site_id_ + 2];
  transform.transform.rotation.z = model_->site_quat[4 * site_id_ + 3];
  return transform;
}

void LidarSensor::scan_loop() {
  try {
    while (!stopping_) {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.wait(lock, [this] { return pending_ || stopping_; });
      if (stopping_) break;
      pending_data_.swap(scan_data_);
      const uint64_t generation = pending_generation_;
      pending_ = false;
      lock.unlock();
      scan_and_publish(generation);
    }
  } catch (const std::exception& error) {
    stopping_ = true;
    RCLCPP_ERROR(node_->get_logger(), "Lidar '%s' stopped: %s", config_.name.c_str(), error.what());
  }
}

void LidarSensor::scan_and_publish(uint64_t generation) {
  // mj_step integrates qpos after deriving geom/site poses. Rebuild geometry and
  // dynamic BVH on the private snapshot so ranges match its sampling timestamp.
  mj_fwdPosition(model_, scan_data_.get());
  const mjtNum* rotation = scan_data_->site_xmat + 9 * site_id_;
  for (int i = 0; i < num_rays_; ++i) {
    mju_mulMatVec3(world_vectors_.data() + 3 * i, rotation, vectors_.data() + 3 * i);
  }
  // MuJoCo 3.3.6 signature; flg_static=1 includes the floor and fixed obstacles.
  // As upstream, exclude the site's own body, retaining possible leg occlusion.
  mj_multiRay(model_, scan_data_.get(), scan_data_->site_xpos + 3 * site_id_,
      world_vectors_.data(), config_.geom_groups.empty() ? nullptr : geom_mask_.data(),
      1, model_->site_bodyid[site_id_], geom_ids_.data(), ranges_.data(), num_rays_, config_.range_max);

  sensor_msgs::PointCloud2Iterator<float> x(cloud_, "x"), y(cloud_, "y"), z(cloud_, "z");
  for (int i = 0; i < num_rays_; ++i, ++x, ++y, ++z) {
    const mjtNum range = ranges_[i];
    if (std::isfinite(range) && range >= config_.range_min && range < config_.range_max) {
      *x = static_cast<float>(vectors_[3 * i] * range);
      *y = static_cast<float>(vectors_[3 * i + 1] * range);
      *z = static_cast<float>(vectors_[3 * i + 2] * range);
    } else {
      *x = *y = *z = std::numeric_limits<float>::quiet_NaN();
    }
  }
  cloud_.header.stamp = rclcpp::Time(
      static_cast<int64_t>(std::llround(scan_data_->time * 1e9)), RCL_ROS_TIME);
  // Serialize the generation check/publication with reset, so an old scan cannot
  // be published after reset returns. Sampling uses try_lock and never waits.
  std::lock_guard<std::mutex> lock(mutex_);
  if (!stopping_ && generation == generation_) publisher_->publish(cloud_);
}

}  // namespace ros2_sensors
