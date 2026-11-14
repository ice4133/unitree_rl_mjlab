// Adapted from mujoco_ros2_control/src/mujoco_system_interface.cpp (Apache-2.0).
// Copyright (c) 2025, United States Government, as represented by the
// Administrator of the National Aeronautics and Space Administration.
// Modified: standalone lifecycle, nonblocking state snapshots, simulation-time
// rate control, ROS child-frame twist, dynamic TF and reset handling.
// Licensed under the Apache License, Version 2.0. See ../LICENSE.
#include "state/odometry_publisher.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

#include <geometry_msgs/msg/transform_stamped.hpp>

namespace ros2_sensors {

OdometryPublisher::OdometryPublisher(
    rclcpp::Node::SharedPtr node, const mjModel* model, OdometryConfig config)
    : node_(std::move(node)), config_(std::move(config)) {
  const int joint = mj_name2id(model, mjOBJ_JOINT, config_.free_joint.c_str());
  if (joint < 0 || model->jnt_type[joint] != mjJNT_FREE) {
    throw std::runtime_error("Odometry requires a free joint: " + config_.free_joint);
  }
  const char* body = mj_id2name(model, mjOBJ_BODY, model->jnt_bodyid[joint]);
  if (!body || config_.frame_id.empty() || config_.frame_id == body || config_.topic.empty() ||
      !std::isfinite(config_.publish_rate) || config_.publish_rate <= 0) {
    throw std::runtime_error("Invalid odometry body/frame/topic/publish_rate");
  }
  qpos_address_ = model->jnt_qposadr[joint];
  qvel_address_ = model->jnt_dofadr[joint];
  pending_message_.header.frame_id = config_.frame_id;
  pending_message_.child_frame_id = body;
  // This is exact simulator state, with no added estimator uncertainty/noise.
  // Covariances remain zero; no sensor accuracy is inferred from this stream.
  publisher_ = node_->create_publisher<nav_msgs::msg::Odometry>(
      config_.topic, rclcpp::QoS(10).reliable());
  if (config_.publish_tf) tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
  worker_ = std::thread(&OdometryPublisher::publish_loop, this);
  RCLCPP_INFO(node_->get_logger(), "Ground-truth odometry: '%s', %.1f Hz, %s -> %s, TF %s",
      config_.topic.c_str(), config_.publish_rate, config_.frame_id.c_str(), body,
      config_.publish_tf ? "on" : "off");
}

OdometryPublisher::~OdometryPublisher() { stop(); }

void OdometryPublisher::sample(const mjModel* /*model*/, const mjData* data) {
  if (stopping_ || data->time - last_sample_time_ + 1e-9 < 1.0 / config_.publish_rate) return;
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (!lock.owns_lock()) return;

  // Read integrated qpos/qvel directly, avoiding the derived body poses which
  // can still correspond to the start of the last mj_step.
  const mjtNum* position = data->qpos + qpos_address_;
  const mjtNum* quaternion = position + 3;  // MuJoCo: w, x, y, z.
  const mjtNum* velocity = data->qvel + qvel_address_;
  auto& message = pending_message_;
  message.header.stamp = rclcpp::Time(
      static_cast<int64_t>(std::llround(data->time * 1e9)), RCL_ROS_TIME);
  message.pose.pose.position.x = position[0];
  message.pose.pose.position.y = position[1];
  message.pose.pose.position.z = position[2];
  message.pose.pose.orientation.w = quaternion[0];
  message.pose.pose.orientation.x = quaternion[1];
  message.pose.pose.orientation.y = quaternion[2];
  message.pose.pose.orientation.z = quaternion[3];

  // MuJoCo free-joint translation velocity is in world axes, while its angular
  // velocity is already in body axes. ROS Odometry requires BOTH in child axes.
  mjtNum inverse[4], local_linear[3];
  mju_negQuat(inverse, quaternion);
  mju_rotVecQuat(local_linear, velocity, inverse);
  message.twist.twist.linear.x = local_linear[0];
  message.twist.twist.linear.y = local_linear[1];
  message.twist.twist.linear.z = local_linear[2];
  message.twist.twist.angular.x = velocity[3];
  message.twist.twist.angular.y = velocity[4];
  message.twist.twist.angular.z = velocity[5];
  last_sample_time_ = data->time;
  pending_ = true;  // Keep only the newest pending state, never a growing queue.
  lock.unlock();
  cv_.notify_one();
}

void OdometryPublisher::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  ++generation_;
  pending_ = false;
  last_sample_time_ = -std::numeric_limits<double>::infinity();
}

void OdometryPublisher::stop() {
  stopping_ = true;
  cv_.notify_one();
  if (worker_.joinable()) worker_.join();
}

void OdometryPublisher::publish_loop() {
  try {
    while (!stopping_) {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.wait(lock, [this] { return pending_ || stopping_; });
      if (stopping_) break;
      const auto message = pending_message_;
      const auto generation = generation_;
      pending_ = false;
      lock.unlock();

      geometry_msgs::msg::TransformStamped transform;
      transform.header = message.header;
      transform.child_frame_id = message.child_frame_id;
      transform.transform.translation.x = message.pose.pose.position.x;
      transform.transform.translation.y = message.pose.pose.position.y;
      transform.transform.translation.z = message.pose.pose.position.z;
      transform.transform.rotation = message.pose.pose.orientation;

      // Serialize publication with reset to reject any pre-reset snapshot.
      lock.lock();
      if (!stopping_ && generation == generation_) {
        publisher_->publish(message);
        if (tf_) tf_->sendTransform(transform);
      }
    }
  } catch (const std::exception& error) {
    stopping_ = true;
    RCLCPP_ERROR(node_->get_logger(), "Odometry publisher stopped: %s", error.what());
  }
}

}  // namespace ros2_sensors
