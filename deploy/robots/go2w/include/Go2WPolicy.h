#pragma once

#include <cstring>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <unitree/idl/go2/LowCmd_.hpp>
#include "isaaclab/algorithms/algorithms.h"

namespace go2w
{
constexpr int kMotorCount = 16;
constexpr int kLegMotorCount = 12;

struct ActionTarget
{
    int joint_id;
    int motor_id;
    bool velocity;
};

// Joint mappings/default positions use XML order; gains use SDK order.
struct PolicyLayout
{
    explicit PolicyLayout(const YAML::Node &cfg);
    void write_commands(unitree_go::msg::dds_::LowCmd_ &message,
                        const std::vector<float> &actions) const;

    std::vector<int> motor_ids;
    std::vector<float> stiffness;
    std::vector<float> damping;
    std::vector<ActionTarget> targets;
};

// Reuse the common ONNX runner and keep the Go2W command mapping alongside it.
class Go2WPolicy : public isaaclab::OrtRunner
{
public:
    Go2WPolicy(const std::filesystem::path &model, const YAML::Node &cfg);
    void write_commands(unitree_go::msg::dds_::LowCmd_ &message,
                        const std::vector<float> &actions) const;

private:
    PolicyLayout layout_;
};
} // namespace go2w
