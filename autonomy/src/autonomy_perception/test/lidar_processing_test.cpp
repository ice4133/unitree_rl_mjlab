#include "autonomy_perception/lidar_processing.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <cstring>
#include <limits>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace ap = autonomy_perception;
namespace {
sensor_msgs::msg::PointCloud2 cloud(const std::vector<std::array<float, 4>>& points) {
  sensor_msgs::msg::PointCloud2 result;
  result.header.frame_id = "mid360_frame";
  result.header.stamp.sec = 4;
  result.header.stamp.nanosec = 200000000;
  result.height = 1;
  result.width = points.size();
  result.point_step = 16;
  result.row_step = result.width * result.point_step;
  result.data.resize(result.row_step);
  for (int i = 0; i < 4; ++i) {
    sensor_msgs::msg::PointField f;
    f.name = std::array<const char*, 4>{"x", "y", "z", "intensity"}[i];
    f.offset = i * 4;
    f.count = 1;
    f.datatype = sensor_msgs::msg::PointField::FLOAT32;
    result.fields.push_back(f);
  }
  for (size_t i = 0; i < points.size(); ++i)
    std::memcpy(result.data.data() + i * 16, points[i].data(), 16);
  return result;
}

geometry_msgs::msg::TransformStamped transform(const std::string& frame, const tf2::Transform& value) {
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = frame;
  t.child_frame_id = "mid360_frame";
  t.transform = tf2::toMsg(value);
  return t;
}

float value(const sensor_msgs::msg::PointCloud2& c, size_t point, size_t field) {
  float result;
  std::memcpy(&result, c.data.data() + point * c.point_step + field * 4, 4);
  return result;
}

TEST(LidarProcessing, RejectsGroundSelfInvalidNearAndFarPoints) {
  const auto input = cloud({{2, 0, 0, 42}, {2, 0, -0.6f, 1}, {0.15f, 0, -0.1f, 2},
      {2, 0, 2, 3}, {30, 0, 0, 4}, {0.01f, 0, 0, 5},
      {std::numeric_limits<float>::quiet_NaN(), 0, 0, 6},
      {std::numeric_limits<float>::infinity(), 0, 0, 7}});
  tf2::Transform fixed(tf2::Quaternion::getIdentity(), tf2::Vector3(1, 2, 0.6));
  tf2::Transform base(tf2::Quaternion::getIdentity(), tf2::Vector3(0, 0, 0.15));
  const auto result = ap::LidarProcessor({}).process(input, transform("odom", fixed), transform("base_link", base));
  ASSERT_EQ(result.cloud.width, 1u);
  EXPECT_EQ(result.cloud.header.stamp, input.header.stamp);
  EXPECT_EQ(result.cloud.header.frame_id, "lidar_scan_frame");
  EXPECT_EQ(result.cloud.fields, input.fields);
  EXPECT_TRUE(result.cloud.is_dense);
  EXPECT_FLOAT_EQ(value(result.cloud, 0, 0), 2);
  EXPECT_FLOAT_EQ(value(result.cloud, 0, 3), 42);
  EXPECT_NEAR(result.footprint.transform.translation.z, 0, 1e-9);
}

TEST(LidarProcessing, LevelsRollPitchWithoutChangingWorldPointsOrSensorOrigin) {
  tf2::Quaternion rotation;
  rotation.setRPY(0.1, 0.2, 0.7);
  const tf2::Transform fixed_base(rotation, tf2::Vector3(1, 2, 0.45));
  const tf2::Transform base_sensor(tf2::Quaternion::getIdentity(), tf2::Vector3(0, 0, 0.15));
  const auto fixed_sensor = fixed_base * base_sensor;
  const auto input = cloud({{2, 1, 0.2f, 17}});
  const auto result = ap::LidarProcessor({}).process(
      input, transform("odom", fixed_sensor), transform("base_link", base_sensor));
  ASSERT_EQ(result.cloud.width, 1u);
  EXPECT_NEAR(result.scan.transform.rotation.x, 0, 1e-9);
  EXPECT_NEAR(result.scan.transform.rotation.y, 0, 1e-9);
  EXPECT_NEAR(tf2::getYaw(result.scan.transform.rotation), 0.7, 1e-9);
  EXPECT_NEAR(result.footprint.transform.translation.x, 1, 1e-9);
  EXPECT_NEAR(result.footprint.transform.translation.y, 2, 1e-9);
  tf2::Transform fixed_scan;
  tf2::fromMsg(result.scan.transform, fixed_scan);
  const auto reconstructed = fixed_scan * tf2::Vector3(value(result.cloud, 0, 0),
      value(result.cloud, 0, 1), value(result.cloud, 0, 2));
  const auto expected = fixed_sensor * tf2::Vector3(2, 1, 0.2f);
  EXPECT_NEAR((reconstructed - expected).length(), 0, 1e-6);
  EXPECT_FLOAT_EQ(value(result.cloud, 0, 3), 17);
  EXPECT_EQ(result.scan.header.stamp, input.header.stamp);
  EXPECT_EQ(result.footprint.header.stamp, input.header.stamp);
}

TEST(LidarProcessing, HandlesOrganizedRowsPaddingAndBigEndianFields) {
  auto input = cloud({{2, 0, 0, 12}, {3, 0, 0, 13}});
  input.width = 1;
  input.height = 2;
  input.row_step = 20;
  input.is_bigendian = true;
  auto original = input.data;
  input.data.assign(40, 0xff);
  for (size_t row = 0; row < 2; ++row)
    for (size_t field = 0; field < 4; ++field)
      for (size_t byte = 0; byte < 4; ++byte)
        input.data[row * 20 + field * 4 + byte] = original[row * 16 + field * 4 + 3 - byte];
  const auto fixed = transform("odom", tf2::Transform(tf2::Quaternion::getIdentity(), tf2::Vector3(0, 0, 0.6)));
  const auto base = transform("base_link", tf2::Transform(tf2::Quaternion::getIdentity(), tf2::Vector3(0, 0, 0.15)));
  const auto result = ap::LidarProcessor({}).process(input, fixed, base);
  EXPECT_EQ(result.cloud.width, 2u);
  EXPECT_EQ(result.cloud.height, 1u);
  EXPECT_EQ(result.cloud.row_step, 32u);
  EXPECT_EQ(result.cloud.data.size(), 32u);
  EXPECT_TRUE(result.cloud.is_bigendian);
  EXPECT_EQ(result.cloud.data[12], original[15]);
  EXPECT_EQ(result.cloud.data[31], original[28]);
}

TEST(LidarProcessing, RejectsMalformedCloudsAndConflictingFrames) {
  auto input = cloud({{2, 0, 0, 1}});
  const auto fixed = transform("odom", tf2::Transform(tf2::Quaternion::getIdentity(), tf2::Vector3(0, 0, 0.6)));
  const auto base = transform("base_link", tf2::Transform(tf2::Quaternion::getIdentity(), tf2::Vector3(0, 0, 0.15)));
  input.row_step = 1;
  EXPECT_THROW(ap::LidarProcessor({}).process(input, fixed, base), std::invalid_argument);
  input = cloud({{2, 0, 0, 1}});
  input.fields[0].offset = 100;
  EXPECT_THROW(ap::LidarProcessor({}).process(input, fixed, base), std::invalid_argument);
  ap::LidarConfig config;
  config.footprint_frame = "base_link";
  EXPECT_THROW(ap::LidarProcessor{config}, std::invalid_argument);
}
}  // namespace
