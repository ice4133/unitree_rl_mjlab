#include "autonomy_perception/lidar_processing.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <utility>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace autonomy_perception {
namespace {
bool big_endian() {
  const uint16_t value = 1;
  return *reinterpret_cast<const uint8_t*>(&value) == 0;
}

float read_float(const uint8_t* data, bool swap) {
  uint32_t bits;
  std::memcpy(&bits, data, sizeof(bits));
  if (swap) bits = __builtin_bswap32(bits);
  float value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

void write_float(uint8_t* data, float value, bool swap) {
  uint32_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  if (swap) bits = __builtin_bswap32(bits);
  std::memcpy(data, &bits, sizeof(bits));
}

uint32_t field_offset(const sensor_msgs::msg::PointCloud2& cloud, const char* name) {
  for (const auto& field : cloud.fields) {
    if (field.name == name && field.datatype == sensor_msgs::msg::PointField::FLOAT32 &&
        field.count == 1 && static_cast<uint64_t>(field.offset) + 4 <= cloud.point_step)
      return field.offset;
  }
  throw std::invalid_argument(std::string("Expected FLOAT32 point field: ") + name);
}

geometry_msgs::msg::TransformStamped transform_message(
    const std_msgs::msg::Header& header, const std::string& child, const tf2::Transform& transform) {
  geometry_msgs::msg::TransformStamped msg;
  msg.header = header;
  msg.child_frame_id = child;
  msg.transform = tf2::toMsg(transform);
  return msg;
}
}  // namespace

LidarProcessor::LidarProcessor(LidarConfig config) : config_(std::move(config)) {
  const auto& c = config_;
  if (c.fixed_frame.empty() || c.base_frame.empty() || c.scan_frame.empty() || c.footprint_frame.empty() ||
      c.fixed_frame == c.base_frame || c.scan_frame == c.footprint_frame ||
      c.scan_frame == c.fixed_frame || c.scan_frame == c.base_frame ||
      c.footprint_frame == c.fixed_frame || c.footprint_frame == c.base_frame ||
      !std::isfinite(c.ground_height) || !std::isfinite(c.min_height) || !std::isfinite(c.max_height) ||
      c.min_height < 0 || c.max_height <= c.min_height || !std::isfinite(c.range_min) ||
      !std::isfinite(c.range_max) || c.range_min < 0 || c.range_max <= c.range_min)
    throw std::invalid_argument("Invalid lidar frames, height band or range limits");
  for (int i = 0; i < 3; ++i) {
    if (!std::isfinite(c.self_min[i]) || !std::isfinite(c.self_max[i]) || c.self_min[i] >= c.self_max[i])
      throw std::invalid_argument("Invalid robot self-filter bounds");
  }
}

LidarResult LidarProcessor::process(const sensor_msgs::msg::PointCloud2& cloud,
    const geometry_msgs::msg::TransformStamped& fixed_from_cloud,
    const geometry_msgs::msg::TransformStamped& base_from_cloud) const {
  if (cloud.header.frame_id.empty() || cloud.header.frame_id == config_.scan_frame ||
      fixed_from_cloud.header.frame_id != config_.fixed_frame ||
      base_from_cloud.header.frame_id != config_.base_frame ||
      fixed_from_cloud.child_frame_id != cloud.header.frame_id ||
      base_from_cloud.child_frame_id != cloud.header.frame_id ||
      cloud.point_step == 0 || static_cast<uint64_t>(cloud.width) * cloud.point_step > cloud.row_step ||
      static_cast<uint64_t>(cloud.row_step) * cloud.height > cloud.data.size())
    throw std::invalid_argument("Invalid cloud layout or transform frames");
  const std::array<uint32_t, 3> offsets{field_offset(cloud, "x"), field_offset(cloud, "y"), field_offset(cloud, "z")};
  tf2::Transform fixed_sensor, base_sensor;
  tf2::fromMsg(fixed_from_cloud.transform, fixed_sensor);
  tf2::fromMsg(base_from_cloud.transform, base_sensor);
  const auto fixed_base = fixed_sensor * base_sensor.inverse();
  tf2::Quaternion yaw;
  yaw.setRPY(0, 0, tf2::getYaw(fixed_base.getRotation()));
  const tf2::Transform fixed_scan(yaw, fixed_sensor.getOrigin());
  const tf2::Transform fixed_footprint(yaw,
      tf2::Vector3(fixed_base.getOrigin().x(), fixed_base.getOrigin().y(), config_.ground_height));
  const auto scan_sensor = fixed_scan.inverse() * fixed_sensor;

  LidarResult result;
  auto fixed_header = cloud.header;
  fixed_header.frame_id = config_.fixed_frame;
  result.footprint = transform_message(fixed_header, config_.footprint_frame, fixed_footprint);
  result.scan = transform_message(fixed_header, config_.scan_frame, fixed_scan);
  result.cloud = cloud;
  result.cloud.header.frame_id = config_.scan_frame;
  result.cloud.height = 1;
  result.cloud.width = 0;
  result.cloud.is_dense = true;
  result.cloud.data.clear();
  result.cloud.data.reserve(cloud.data.size());
  const bool swap = cloud.is_bigendian != big_endian();

  for (uint32_t row = 0; row < cloud.height; ++row) {
    for (uint32_t col = 0; col < cloud.width; ++col) {
      const auto* record = cloud.data.data() + static_cast<size_t>(row) * cloud.row_step +
          static_cast<size_t>(col) * cloud.point_step;
      const tf2::Vector3 p(read_float(record + offsets[0], swap), read_float(record + offsets[1], swap),
          read_float(record + offsets[2], swap));
      if (!std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.z()) ||
          p.length2() < config_.range_min * config_.range_min ||
          p.length2() > config_.range_max * config_.range_max) continue;
      const auto height = (fixed_sensor * p).z() - config_.ground_height;
      if (height < config_.min_height || height > config_.max_height) continue;
      const auto body = base_sensor * p;
      bool self = true;
      for (int i = 0; i < 3; ++i) self = self && body[i] >= config_.self_min[i] && body[i] <= config_.self_max[i];
      if (self) continue;
      const auto level = scan_sensor * p;
      const auto start = result.cloud.data.size();
      result.cloud.data.insert(result.cloud.data.end(), record, record + cloud.point_step);
      for (int i = 0; i < 3; ++i)
        write_float(result.cloud.data.data() + start + offsets[i], static_cast<float>(level[i]), swap);
      ++result.cloud.width;
    }
  }
  result.cloud.row_step = result.cloud.width * result.cloud.point_step;
  return result;
}
}  // namespace autonomy_perception
