#include "Go2WPolicy.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace go2w
{
PolicyLayout::PolicyLayout(const YAML::Node &cfg)
{
    motor_ids = cfg["joint_ids_map"].as<std::vector<int>>();
    stiffness = cfg["stiffness"].as<std::vector<float>>();
    damping = cfg["damping"].as<std::vector<float>>();
    const auto defaults = cfg["default_joint_pos"].as<std::vector<float>>();
    if (motor_ids.size() != kMotorCount || stiffness.size() != kMotorCount ||
        damping.size() != kMotorCount || defaults.size() != kMotorCount)
        throw std::runtime_error("Go2W requires 16 joint mappings, gains and default positions");
    const float dt = cfg["step_dt"].as<float>();
    if (!std::isfinite(dt) || dt <= 0)
        throw std::runtime_error("step_dt must be finite and positive");

    std::vector<bool> seen_motor(kMotorCount, false);
    for (int i = 0; i < kMotorCount; ++i)
    {
        const int motor = motor_ids[i];
        if (motor < 0 || motor >= kMotorCount || seen_motor[motor])
            throw std::runtime_error("joint_ids_map must be a permutation of SDK slots 0..15");
        seen_motor[motor] = true;
        if (!std::isfinite(defaults[i]) || !std::isfinite(stiffness[motor]) ||
            !std::isfinite(damping[motor]) || stiffness[motor] < 0 || damping[motor] < 0)
            throw std::runtime_error("Go2W positions and gains must be finite; gains must be nonnegative");
        if (motor >= kLegMotorCount && (stiffness[motor] != 0 || damping[motor] <= 0))
            throw std::runtime_error("Continuous wheels require kp=0 and kd>0");
    }

    std::vector<bool> seen_joint(kMotorCount, false);
    const auto actions = cfg["actions"];
    for (auto term = actions.begin(); term != actions.end(); ++term)
    {
        const auto name = term->first.as<std::string>();
        if (name != "JointPositionAction" && name != "JointVelocityAction")
            throw std::runtime_error("Unsupported Go2W action term: " + name);
        const bool velocity = name == "JointVelocityAction";
        const auto action = term->second;
        const auto ids = action["joint_ids"].as<std::vector<int>>();
        if (ids.empty()) throw std::runtime_error("Each Go2W action term requires explicit joint_ids");
        for (const auto *key : {"scale", "offset", "clip"})
        {
            if (action[key] && !action[key].IsNull() && action[key].size() != ids.size())
                throw std::runtime_error(name + ": " + key + " length does not match joint_ids");
        }
        for (const int joint : ids)
        {
            if (joint < 0 || joint >= kMotorCount || seen_joint[joint])
                throw std::runtime_error("Action joint_ids must cover each Go2W joint exactly once");
            seen_joint[joint] = true;
            const int motor = motor_ids[joint];
            if (velocity != (motor >= kLegMotorCount))
                throw std::runtime_error("Legs require position actions; wheels require velocity actions");
            targets.push_back({joint, motor, velocity});
        }
    }
    if (targets.size() != kMotorCount)
        throw std::runtime_error("Go2W policy must produce 12 leg positions and 4 wheel velocities");
}

// processed_actions are concatenated by action term, rather than XML order.
void PolicyLayout::write_commands(unitree_go::msg::dds_::LowCmd_ &message, const std::vector<float> &actions) const
{
    if (actions.size() != targets.size() ||
        !std::all_of(actions.begin(), actions.end(), [](float v) { return std::isfinite(v); }))
        throw std::runtime_error("Invalid Go2W policy output");
    for (std::size_t i = 0; i < targets.size(); ++i)
    {
        const auto &target = targets[i];
        auto &motor = message.motor_cmd()[target.motor_id];
        motor.mode() = 1;
        motor.kp() = stiffness[target.motor_id];
        motor.kd() = damping[target.motor_id];
        motor.tau() = 0;
        motor.q() = target.velocity ? 0.0f : actions[i];
        motor.dq() = target.velocity ? actions[i] : 0.0f;
    }
}

Go2WPolicy::Go2WPolicy(const std::filesystem::path &model, const YAML::Node &cfg)
    : isaaclab::OrtRunner(model.string()), layout_(cfg)
{
}

void Go2WPolicy::write_commands(unitree_go::msg::dds_::LowCmd_ &message,
                                const std::vector<float> &actions) const
{
    layout_.write_commands(message, actions);
}
} // namespace go2w
