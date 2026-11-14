// Adapted from mujoco_ros2_control/src/mujoco_system_interface.cpp (Apache-2.0).
// Copyright (c) 2025, United States Government, as represented by the
// Administrator of the National Aeronautics and Space Administration.
// Modified: standalone lifecycle, simulation clock, body-frame twist and TF.
// Licensed under the Apache License, Version 2.0. See ../LICENSE.
#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "common/sensor.hpp"

namespace ros2_sensors {

struct OdometryConfig {
  std::string free_joint = "floating_base_joint";
  std::string topic = "/odom";
  std::string frame_id = "odom";
  double publish_rate = 50.0;
  bool publish_tf = true;
};

// Ground-truth state of the selected root free joint, expressed in MuJoCo's
// world frame (named config.frame_id). sample/reset run under the sim mutex.
class OdometryPublisher final : public Sensor {
 public:
  OdometryPublisher(rclcpp::Node::SharedPtr node, const mjModel* model, OdometryConfig config);
  ~OdometryPublisher() override;
  void sample(const mjModel* model, const mjData* data) override;
  void reset() override;
  void stop() override;

 private:
  void publish_loop();

  rclcpp::Node::SharedPtr node_;
  OdometryConfig config_;
  int qpos_address_;
  int qvel_address_;
  nav_msgs::msg::Odometry pending_message_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::thread worker_;
  std::atomic_bool stopping_{false};
  uint64_t generation_ = 0;
  bool pending_ = false;
  double last_sample_time_ = -std::numeric_limits<double>::infinity();
};

}  // namespace ros2_sensors
