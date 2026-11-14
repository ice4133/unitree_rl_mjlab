// Adapted from mujoco_ros2_control_plugins/src/camera_plugin.hpp (Apache-2.0).
// Copyright (c) 2026, United States Government, as represented by the
// Administrator of the National Aeronautics and Space Administration.
// Modified: standalone Sensor API and double-buffered visual snapshots.
// Licensed under the Apache License, Version 2.0. See ../LICENSE.
#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "common/sensor.hpp"

namespace ros2_sensors {

struct CameraConfig {
  std::string name;
  std::string frame_id;
  std::string color_topic;
  std::string depth_topic;
  std::string info_topic;
  double publish_rate = 15.0;
};

class CameraSensor final : public Sensor {
 public:
  CameraSensor(rclcpp::Node::SharedPtr node, const mjModel* model, CameraConfig config);
  ~CameraSensor() override;
  void sample(const mjModel* model, const mjData* data) override;
  void reset() override;
  void stop() override;
  geometry_msgs::msg::TransformStamped static_transform() const;

 private:
  void render_loop(std::promise<void> initialized);
  void render_and_publish(mjvScene& scene, mjrContext& context, uint64_t generation);

  rclcpp::Node::SharedPtr node_;
  const mjModel* model_;
  CameraConfig config_;
  int camera_id_;
  int width_;
  int height_;
  std::unique_ptr<mjData, decltype(&mj_deleteData)> pending_data_{nullptr, mj_deleteData};
  std::unique_ptr<mjData, decltype(&mj_deleteData)> render_data_{nullptr, mj_deleteData};
  mjvCamera camera_{};
  mjvOption options_{};
  sensor_msgs::msg::Image color_;
  sensor_msgs::msg::Image depth_;
  sensor_msgs::msg::CameraInfo info_;
  std::vector<unsigned char> rgb_buffer_;
  std::vector<float> depth_buffer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::thread worker_;
  std::atomic_bool stopping_{false};
  std::atomic<uint64_t> generation_{0};
  uint64_t pending_generation_ = 0;
  bool pending_ = false;
  double last_sample_time_ = -std::numeric_limits<double>::infinity();
};

}  // namespace ros2_sensors
