#include "RosVelocityCommand.h"

#ifdef UNITREE_WITH_ROS2_COMMANDS
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <thread>

namespace go2w
{
struct RosVelocityCommand::Impl
{
    using Clock = std::chrono::steady_clock;
    std::mutex mutex;
    isaaclab::VelocityCommand command{};
    Clock::time_point received{};
    bool valid = false;
    double timeout = 0.5;
    std::atomic<bool> spinning{false};
    rclcpp::Context::SharedPtr context;
    rclcpp::Node::SharedPtr node;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
    std::thread thread;

    ~Impl()
    {
        // Shutdown also wakes an executor whose thread has not entered spin yet.
        if (context && context->is_valid()) context->shutdown("Velocity input stopped");
        if (executor) executor->cancel();
        if (thread.joinable()) thread.join();
    }
};

RosVelocityCommand::RosVelocityCommand(const std::string& topic, double timeout, int domain_id)
    : impl_(std::make_unique<Impl>())
{
    if (topic.empty() || !std::isfinite(timeout) || timeout <= 0 || domain_id < 0)
        throw std::invalid_argument("Invalid cmd_vel topic, timeout or ROS domain");
    const char* rmw = std::getenv("RMW_IMPLEMENTATION");
    if (rmw && std::string(rmw) != "rmw_fastrtps_cpp")
        throw std::runtime_error("ROS velocity input requires RMW_IMPLEMENTATION=rmw_fastrtps_cpp");
    setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp", 1);

    auto& state = *impl_;
    state.timeout = timeout;
    state.context = std::make_shared<rclcpp::Context>();
    rclcpp::InitOptions init_options;
    init_options.set_domain_id(domain_id);
    state.context->init(0, nullptr, init_options);
    rclcpp::NodeOptions options;
    options.context(state.context).use_global_arguments(false)
        .start_parameter_services(false).start_parameter_event_publisher(false);
    state.node = std::make_shared<rclcpp::Node>("go2w_velocity_input", options);
    state.subscription = state.node->create_subscription<geometry_msgs::msg::Twist>(
        topic, rclcpp::QoS(1).reliable().durability_volatile(),
        [&state](const geometry_msgs::msg::Twist& msg) {
            const auto representable = [](double value) {
                return std::isfinite(value) && std::abs(value) <= std::numeric_limits<float>::max();
            };
            const bool valid = representable(msg.linear.x) && representable(msg.linear.y) &&
                               representable(msg.angular.z);
            isaaclab::VelocityCommand command{};
            if (valid) command = {static_cast<float>(msg.linear.x), static_cast<float>(msg.linear.y),
                                  static_cast<float>(msg.angular.z)};
            std::lock_guard<std::mutex> lock(state.mutex);
            state.valid = valid;
            state.command = command;
            state.received = Impl::Clock::now();
        });
    rclcpp::ExecutorOptions executor_options;
    executor_options.context = state.context;
    state.executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(executor_options);
    state.executor->add_node(state.node);
    state.spinning = true;
    state.thread = std::thread([&state] {
        try {
            state.executor->spin();
        } catch (const std::exception& error) {
            if (state.context->is_valid())
                RCLCPP_ERROR(state.node->get_logger(), "Velocity input stopped: %s", error.what());
        }
        state.spinning = false;
    });
    RCLCPP_INFO(state.node->get_logger(), "Automatic velocity: %s, domain=%d, timeout=%.3f s (steady clock)",
                topic.c_str(), domain_id, timeout);
}

RosVelocityCommand::~RosVelocityCommand() = default;

isaaclab::VelocityCommand RosVelocityCommand::read() const
{
    auto& state = *impl_;
    std::lock_guard<std::mutex> lock(state.mutex);
    if (!state.spinning || !state.valid ||
        std::chrono::duration<double>(Impl::Clock::now() - state.received).count() >= state.timeout)
        return {};
    return state.command;
}
} // namespace go2w
#endif
