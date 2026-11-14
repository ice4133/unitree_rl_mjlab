#include "autonomy_perception/lidar_processing.hpp"

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace autonomy_perception {
class LidarFilterNode : public rclcpp::Node {
 public:
  LidarFilterNode() : Node("lidar_filter") {
    config_.fixed_frame = declare_parameter("fixed_frame", "odom");
    config_.base_frame = declare_parameter("base_frame", "base_link");
    config_.footprint_frame = declare_parameter("footprint_frame", "base_footprint");
    config_.scan_frame = declare_parameter("scan_frame", "lidar_scan_frame");
    config_.ground_height = declare_parameter("ground_height", 0.0);
    config_.min_height = declare_parameter("min_height", 0.08);
    config_.max_height = declare_parameter("max_height", 2.0);
    config_.range_min = declare_parameter("range_min", 0.1);
    config_.range_max = declare_parameter("range_max", 20.0);
    const auto lower = declare_parameter("self_min", std::vector<double>{-0.45, -0.30, -0.60});
    const auto upper = declare_parameter("self_max", std::vector<double>{0.45, 0.30, 0.25});
    if (lower.size() != 3 || upper.size() != 3) throw std::invalid_argument("Self-filter bounds require three coordinates");
    std::copy(lower.begin(), lower.end(), config_.self_min.begin());
    std::copy(upper.begin(), upper.end(), config_.self_max.begin());
    processor_ = std::make_unique<LidarProcessor>(config_);
    const auto input = declare_parameter("input_topic", "/lidar/points");
    const auto output = declare_parameter("output_topic", "/perception/lidar/points_filtered");
    if (input == output) throw std::invalid_argument("Input and output point-cloud topics must differ");
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output, rclcpp::SensorDataQoS().keep_last(1));
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    buffer_->setCreateTimerInterface(std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(), get_node_timers_interface()));
    listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_);
    subscriber_.subscribe(this, input, rclcpp::SensorDataQoS().keep_last(5).get_rmw_qos_profile());
    filter_ = std::make_unique<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
        subscriber_, *buffer_, config_.fixed_frame, 10,
        get_node_logging_interface(), get_node_clock_interface());
    filter_->registerCallback([this](sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {
      try {
        const auto fixed = buffer_->lookupTransform(config_.fixed_frame, cloud->header.frame_id, cloud->header.stamp);
        const auto base = buffer_->lookupTransform(config_.base_frame, cloud->header.frame_id, cloud->header.stamp);
        const auto result = processor_->process(*cloud, fixed, base);
        broadcaster_->sendTransform(std::vector<geometry_msgs::msg::TransformStamped>{result.footprint, result.scan});
        publisher_->publish(result.cloud);
      } catch (const std::exception& error) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Skipping lidar frame: %s", error.what());
      }
    });
    RCLCPP_INFO(get_logger(), "Lidar processing: %s -> %s, level frame=%s, fixed frame=%s",
        input.c_str(), output.c_str(), config_.scan_frame.c_str(), config_.fixed_frame.c_str());
  }

 private:
  LidarConfig config_;
  std::unique_ptr<LidarProcessor> processor_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> subscriber_;
  std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>> filter_;
};
}  // namespace autonomy_perception

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<autonomy_perception::LidarFilterNode>());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("lidar_filter"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
