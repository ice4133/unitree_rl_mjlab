// Adapted from mujoco_ros2_control's 3dlidar and Mujoco3dLidarPlugin (Apache-2.0).
// Copyright (c) 2026, United States Government, as represented by the
// Administrator of the National Aeronautics and Space Administration.
// Modified: standalone Sensor lifecycle, private snapshots, MuJoCo 3.3.6.
// Licensed under the Apache License, Version 2.0. See ../LICENSE.
#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "common/sensor.hpp"

namespace ros2_sensors {

struct LidarConfig {
  std::string name;
  std::string site;
  std::string frame_id;
  std::string topic = "/lidar/points";
  std::array<int, 2> resolution{1000, 20};
  std::array<double, 2> azimuth_range{-3.141592653589793, 3.141592653589793};
  std::array<double, 2> elevation_range{-0.122173047639603, 0.907571211037051};
  std::vector<int> geom_groups{0, 3};
  double range_min = 0.1;
  double range_max = 40.0;
  double publish_rate = 10.0;
};

class LidarSensor final : public Sensor {
 public:
  LidarSensor(rclcpp::Node::SharedPtr node, const mjModel* model, LidarConfig config);
  ~LidarSensor() override;
  void sample(const mjModel* model, const mjData* data) override;
  void reset() override;
  void stop() override;
  geometry_msgs::msg::TransformStamped static_transform() const;

 private:
  void scan_loop();
  void scan_and_publish(uint64_t generation);

  rclcpp::Node::SharedPtr node_;
  const mjModel* model_;
  LidarConfig config_;
  int site_id_;
  int num_rays_;
  std::array<mjtByte, mjNGROUP> geom_mask_{};
  std::vector<mjtNum> vectors_;
  std::vector<mjtNum> world_vectors_;
  std::vector<mjtNum> ranges_;
  std::vector<int> geom_ids_;
  std::unique_ptr<mjData, decltype(&mj_deleteData)> pending_data_{nullptr, mj_deleteData};
  std::unique_ptr<mjData, decltype(&mj_deleteData)> scan_data_{nullptr, mj_deleteData};
  sensor_msgs::msg::PointCloud2 cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::thread worker_;
  std::atomic_bool stopping_{false};
  uint64_t generation_ = 0;
  uint64_t pending_generation_ = 0;
  bool pending_ = false;
  double last_sample_time_ = -std::numeric_limits<double>::infinity();
};

}  // namespace ros2_sensors
