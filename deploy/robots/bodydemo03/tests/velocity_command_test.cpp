#include "RosVelocityCommand.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>

int main()
{
    try {
        bodydemo03::RosVelocityCommand receiver("/bodydemo03_velocity_test", 0.15, 10);
        if (receiver.read() != isaaclab::VelocityCommand{})
            throw std::runtime_error("Missing commands must return zero velocity");
        rclcpp::init(0, nullptr);
        auto context = std::make_shared<rclcpp::Context>();
        rclcpp::InitOptions init_options;
        init_options.set_domain_id(10);
        context->init(0, nullptr, init_options);
        auto node = std::make_shared<rclcpp::Node>(
            "bodydemo03_velocity_test", rclcpp::NodeOptions().context(context));
        auto publisher = node->create_publisher<geometry_msgs::msg::Twist>(
            "/bodydemo03_velocity_test", rclcpp::QoS(1));
        geometry_msgs::msg::Twist message;
        message.linear.x = 0.3;
        message.angular.z = -0.4;
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        bool received = false;
        while (std::chrono::steady_clock::now() < deadline) {
            publisher->publish(message);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            const auto command = receiver.read();
            if (std::abs(command[0] - 0.3f) < 1e-5 && std::abs(command[2] + 0.4f) < 1e-5) {
                received = true;
                break;
            }
        }
        if (!received) throw std::runtime_error("ROS commands did not reach domain 10");
        message.linear.x = std::numeric_limits<double>::quiet_NaN();
        publisher->publish(message);
        std::this_thread::sleep_for(std::chrono::milliseconds(60));
        if (receiver.read() != isaaclab::VelocityCommand{})
            throw std::runtime_error("Invalid commands must return zero velocity");
        message.linear.x = 0.3;
        publisher->publish(message);
        std::this_thread::sleep_for(std::chrono::milliseconds(60));
        if (std::abs(receiver.read()[0] - 0.3f) >= 1e-5)
            throw std::runtime_error("Valid command did not recover after NaN");
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        if (receiver.read() != isaaclab::VelocityCommand{})
            throw std::runtime_error("Timed-out commands must return zero velocity");
        context->shutdown("Test completed");
        rclcpp::shutdown();
        std::cout << "ROS domain 10 reception, invalid values and timeout passed\n";
    } catch (const std::exception& error) {
        if (rclcpp::ok()) rclcpp::shutdown();
        std::cerr << error.what() << '\n';
        return 1;
    }
}
