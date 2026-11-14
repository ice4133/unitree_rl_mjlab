#pragma once

#include <array>
#include <string>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace autonomy_perception {
struct LidarConfig {
  std::string fixed_frame = "odom";
  std::string base_frame = "base_link";
  std::string footprint_frame = "base_footprint";
  std::string scan_frame = "lidar_scan_frame";
  double ground_height = 0.0;
  double min_height = 0.08;
  double max_height = 2.0;
  double range_min = 0.1;
  double range_max = 20.0;
  std::array<double, 3> self_min{-0.45, -0.30, -0.60};
  std::array<double, 3> self_max{0.45, 0.30, 0.25};
};

struct LidarResult {
  sensor_msgs::msg::PointCloud2 cloud;
  geometry_msgs::msg::TransformStamped footprint;
  geometry_msgs::msg::TransformStamped scan;
};

// No ROS executor or simulator dependency. All transforms refer to cloud.stamp.
class LidarProcessor {
 public:
  explicit LidarProcessor(LidarConfig config);
  LidarResult process(const sensor_msgs::msg::PointCloud2& cloud,
      const geometry_msgs::msg::TransformStamped& fixed_from_cloud,
      const geometry_msgs::msg::TransformStamped& base_from_cloud) const;
 private:
  LidarConfig config_;
};
}  // namespace autonomy_perception
