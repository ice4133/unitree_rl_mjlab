// Adapted from mujoco_ros2_control_plugins/src/camera_plugin.cpp (Apache-2.0).
// Copyright (c) 2026, United States Government, as represented by the
// Administrator of the National Aeronautics and Space Administration.
// Modified: MuJoCo 3.3.6, standalone lifecycle, EGL worker, race-free snapshots,
// ROS optical frames, identity rectification matrix, and invalid-depth handling.
// Licensed under the Apache License, Version 2.0. See ../LICENSE.

#include "camera/camera_sensor.hpp"
#include "common/offscreen_context.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <utility>

namespace ros2_sensors {

CameraSensor::CameraSensor(rclcpp::Node::SharedPtr node, const mjModel* model, CameraConfig config)
    : node_(std::move(node)), model_(model), config_(std::move(config)) {
  camera_id_ = mj_name2id(model_, mjOBJ_CAMERA, config_.name.c_str());
  if (camera_id_ < 0) throw std::runtime_error("Camera not found in MJCF: " + config_.name);
  width_ = model_->cam_resolution[2 * camera_id_];
  height_ = model_->cam_resolution[2 * camera_id_ + 1];
  if (width_ < 1 || height_ < 1 || config_.frame_id.empty() ||
      !std::isfinite(config_.publish_rate) || config_.publish_rate <= 0.0) {
    throw std::runtime_error("Invalid camera resolution/frame/publish_rate: " + config_.name);
  }
  if (model_->cam_mode[camera_id_] != mjCAMLIGHT_FIXED || model_->cam_orthographic[camera_id_]) {
    throw std::runtime_error("ROS camera requires a fixed, perspective MJCF camera: " + config_.name);
  }
  mjv_defaultCamera(&camera_);
  camera_.type = mjCAMERA_FIXED;
  camera_.fixedcamid = camera_id_;
  mjv_defaultOption(&options_);
  options_.flags[mjVIS_RANGEFINDER] = 0;
  std::fill(std::begin(options_.sitegroup), std::end(options_.sitegroup), 0);
  options_.geomgroup[3] = 0;  // Robot collision shapes are separate from its visible meshes.

  const uint16_t endian = 1;
  color_.is_bigendian = depth_.is_bigendian =
      (*reinterpret_cast<const unsigned char*>(&endian) == 0);
  color_.header.frame_id = depth_.header.frame_id = info_.header.frame_id = config_.frame_id;
  color_.width = depth_.width = info_.width = width_;
  color_.height = depth_.height = info_.height = height_;
  color_.encoding = "rgb8";
  color_.step = width_ * 3;
  color_.data.resize(static_cast<size_t>(color_.step) * height_);
  depth_.encoding = "32FC1";
  depth_.step = width_ * sizeof(float);
  depth_.data.resize(static_cast<size_t>(depth_.step) * height_);
  rgb_buffer_.resize(color_.data.size());
  depth_buffer_.resize(static_cast<size_t>(width_) * height_);

  // Same pinhole/fovy intrinsics as upstream; this model has square pixels.
  constexpr double pi = 3.14159265358979323846;
  const double focal = height_ / (2.0 * std::tan(model_->cam_fovy[camera_id_] * pi / 360.0));
  info_.distortion_model = "plumb_bob";
  info_.d.assign(5, 0.0);
  info_.k = {focal, 0, width_ / 2.0, 0, focal, height_ / 2.0, 0, 0, 1};
  info_.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  info_.p = {focal, 0, width_ / 2.0, 0, 0, focal, height_ / 2.0, 0, 0, 0, 1, 0};
  const auto qos = rclcpp::SensorDataQoS().keep_last(1);
  color_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(config_.color_topic, qos);
  depth_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(config_.depth_topic, qos);
  info_pub_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>(config_.info_topic, qos);

  pending_data_.reset(mj_makeData(model_));
  render_data_.reset(mj_makeData(model_));
  if (!pending_data_ || !render_data_) throw std::runtime_error("Cannot allocate camera snapshots");
  std::promise<void> initialized;
  auto ready = initialized.get_future();
  worker_ = std::thread(&CameraSensor::render_loop, this, std::move(initialized));
  try {
    ready.get();
  } catch (...) {
    stop();
    throw;
  }
  RCLCPP_INFO(node_->get_logger(), "Camera '%s': %dx%d, %.1f Hz, frame '%s'",
              config_.name.c_str(), width_, height_, config_.publish_rate, config_.frame_id.c_str());
}

CameraSensor::~CameraSensor() { stop(); }

void CameraSensor::sample(const mjModel* model, const mjData* data) {
  if (stopping_ || data->time - last_sample_time_ + 1e-9 < 1.0 / config_.publish_rate) return;
  // Never wait for OpenGL or publication while the simulator's mutex is held.
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (!lock.owns_lock()) return;
  mjv_copyData(pending_data_.get(), model, data);
  pending_generation_ = generation_.load();
  pending_ = true;  // Keep the latest pending frame instead of growing a queue.
  last_sample_time_ = data->time;
  lock.unlock();
  cv_.notify_one();
}

void CameraSensor::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  ++generation_;
  pending_ = false;
  last_sample_time_ = -std::numeric_limits<double>::infinity();
}

void CameraSensor::stop() {
  stopping_ = true;
  cv_.notify_one();
  if (worker_.joinable()) worker_.join();
}

geometry_msgs::msg::TransformStamped CameraSensor::static_transform() const {
  geometry_msgs::msg::TransformStamped transform;
  const int body = model_->cam_bodyid[camera_id_];
  const char* body_name = mj_id2name(model_, mjOBJ_BODY, body);
  if (!body_name) throw std::runtime_error("Camera parent body must have a name");
  transform.header.frame_id = body_name;
  transform.child_frame_id = config_.frame_id;
  transform.transform.translation.x = model_->cam_pos[3 * camera_id_];
  transform.transform.translation.y = model_->cam_pos[3 * camera_id_ + 1];
  transform.transform.translation.z = model_->cam_pos[3 * camera_id_ + 2];
  // MuJoCo/OpenGL: right +X, up +Y, backward +Z.
  // ROS optical: right +X, down +Y, forward +Z (180 degrees about local X).
  const mjtNum optical_rotation[4] = {0, 1, 0, 0};
  mjtNum quaternion[4];
  mju_mulQuat(quaternion, model_->cam_quat + 4 * camera_id_, optical_rotation);
  transform.transform.rotation.w = quaternion[0];
  transform.transform.rotation.x = quaternion[1];
  transform.transform.rotation.y = quaternion[2];
  transform.transform.rotation.z = quaternion[3];
  return transform;
}

void CameraSensor::render_loop(std::promise<void> initialized) {
  bool ready = false;
  try {
    OffscreenContext gl;
    mjvScene scene;
    mjrContext context;
    mjv_defaultScene(&scene);
    mjr_defaultContext(&context);
    try {
      mjv_makeScene(model_, &scene, std::max(2000, model_->ngeom * 4));
      mjr_makeContext(model_, &context, mjFONTSCALE_100);
      mjr_resizeOffscreen(width_, height_, &context);
      context.readDepthMap = mjDEPTH_ZERONEAR;
      initialized.set_value();
      ready = true;
      while (!stopping_) {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this] { return pending_ || stopping_; });
        if (stopping_) break;
        pending_data_.swap(render_data_);
        const uint64_t generation = pending_generation_;
        pending_ = false;
        lock.unlock();
        render_and_publish(scene, context, generation);
      }
    } catch (...) {
      mjr_freeContext(&context);
      mjv_freeScene(&scene);
      throw;
    }
    mjr_freeContext(&context);
    mjv_freeScene(&scene);
  } catch (...) {
    stopping_ = true;
    if (!ready) initialized.set_exception(std::current_exception());
    else RCLCPP_ERROR(node_->get_logger(), "Camera '%s' rendering stopped after an error", config_.name.c_str());
  }
}

void CameraSensor::render_and_publish(mjvScene& scene, mjrContext& context, uint64_t generation) {
  // mj_step integrates positions after computing derived camera/body poses.
  // Recompute these on our private snapshot so pixels match its timestamp.
  mj_kinematics(model_, render_data_.get());
  mj_camlight(model_, render_data_.get());
  mjr_setBuffer(mjFB_OFFSCREEN, &context);
  mjv_updateScene(model_, render_data_.get(), &options_, nullptr, &camera_, mjCAT_ALL, &scene);
  const mjrRect viewport{0, 0, width_, height_};
  mjr_render(viewport, &scene, &context);
  mjr_readPixels(rgb_buffer_.data(), depth_buffer_.data(), viewport, &context);

  const double near = model_->vis.map.znear * model_->stat.extent;
  const double far = model_->vis.map.zfar * model_->stat.extent;
  const double scale = 1.0 - near / far;
  for (int row = 0; row < height_; ++row) {
    const size_t src = static_cast<size_t>(row) * width_;
    const size_t dst = static_cast<size_t>(height_ - 1 - row) * width_;
    std::memcpy(color_.data.data() + dst * 3, rgb_buffer_.data() + src * 3, width_ * 3);
    for (int col = 0; col < width_; ++col) {
      const float raw = depth_buffer_[src + col];
      const float meters = raw >= 1.0f ? std::numeric_limits<float>::quiet_NaN()
                                      : static_cast<float>(near / (1.0 - raw * scale));
      std::memcpy(depth_.data.data() + (dst + col) * sizeof(float), &meters, sizeof(float));
    }
  }
  if (stopping_ || generation != generation_.load()) return;
  const rclcpp::Time stamp(static_cast<int64_t>(std::llround(render_data_->time * 1e9)), RCL_ROS_TIME);
  color_.header.stamp = depth_.header.stamp = info_.header.stamp = stamp;
  color_pub_->publish(color_);
  depth_pub_->publish(depth_);
  info_pub_->publish(info_);
}

}  // namespace ros2_sensors
