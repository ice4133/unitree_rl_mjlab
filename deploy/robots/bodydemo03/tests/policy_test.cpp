#include "Bodydemo03Policy.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"

#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace
{
void require(bool value, const std::string& message)
{
    if (!value) throw std::runtime_error(message);
}

void near(float actual, float expected)
{
    require(std::abs(actual - expected) < 1e-5f, "Unexpected observation or motor target");
}

template <typename Fn>
void rejects(Fn fn)
{
    bool rejected = false;
    try { fn(); } catch (const std::runtime_error&) { rejected = true; }
    require(rejected, "Invalid policy contract was accepted");
}

class Robot : public isaaclab::Articulation
{
public:
    void update() override
    {
        data.root_ang_vel_b = Eigen::Vector3f(1, 2, 3);
        data.projected_gravity_b = Eigen::Vector3f(0, 0, -1);
        for (int i = 0; i < 16; ++i) {
            // Cumulative wheel positions must never reach the actor.
            data.joint_pos[i] = (i % 4 == 3) ? 1000.0f : 0.01f * (i + 1);
            data.joint_vel[i] = i + 1;
        }
    }
};
}

int main(int argc, char** argv)
{
    try {
        require(argc >= 2, "Usage: policy_test deploy.yaml [policy.onnx]");
        const auto cfg = YAML::LoadFile(argv[1]);
        auto robot = std::make_shared<Robot>();
        isaaclab::ManagerBasedRLEnv env(cfg, robot, [] {
            return isaaclab::VelocityCommand{2.0f, 0.5f, -2.0f};
        });
        env.reset();
        const auto obs = env.observation_manager->compute().at("obs");
        require(obs.size() == 55, "Flat observations must have 55 entries");
        for (int i = 0; i < 3; ++i) near(obs[i], 0.25f * (i + 1));
        near(obs[5], -1);
        near(obs[6], 1); near(obs[7], 0); near(obs[8], -1);
        near(obs[9] * obs[9] + obs[10] * obs[10], 1);
        const std::array<int, 12> leg_joints{0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14};
        for (int i = 0; i < 12; ++i) near(obs[11 + i], 0.01f * (leg_joints[i] + 1));
        for (int i = 0; i < 16; ++i) {
            near(obs[23 + i], 0.05f * (i + 1));
            near(obs[39 + i], 0);
        }

        std::vector<float> raw(16);
        for (int i = 0; i < 16; ++i) raw[i] = (i % 2 == 0 ? 1.0f : -1.0f) * (i + 1);
        env.action_manager->process_action(raw);
        const auto processed = env.action_manager->processed_actions();
        bodydemo03::PolicyLayout layout(cfg);
        unitree_go::msg::dds_::LowCmd_ message{};
        layout.write_commands(message, processed);
        const std::array<int, 16> motor_ids{3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8, 13, 12, 15, 14};
        for (int i = 0; i < 16; ++i) {
            const auto& motor = message.motor_cmd()[motor_ids[i]];
            const bool wheel = i >= 12;
            const float scale = wheel ? 5.0f : (i % 3 == 0 ? 0.125f : 0.25f);
            near(motor.q(), wheel ? 0 : raw[i] * scale);
            near(motor.dq(), wheel ? raw[i] * scale : 0);
            near(motor.kp(), wheel ? 0 : 400);
            near(motor.kd(), wheel ? 2 : 12);
            near(motor.tau(), 0);
            require(motor.mode() == 1, "Motor was not enabled");
        }
        const auto next_obs = env.observation_manager->compute().at("obs");
        for (int i = 0; i < 16; ++i) near(next_obs[39 + i], raw[i]);

        auto invalid = YAML::Clone(cfg);
        invalid["joint_ids_map"][0] = invalid["joint_ids_map"][1].as<int>();
        rejects([&] { bodydemo03::PolicyLayout bad(invalid); });
        invalid = YAML::Clone(cfg);
        invalid["stiffness"][12] = 1;
        rejects([&] { bodydemo03::PolicyLayout bad(invalid); });
        rejects([&] { layout.write_commands(message, std::vector<float>(12)); });
        auto nan_actions = processed;
        nan_actions[15] = std::numeric_limits<float>::quiet_NaN();
        rejects([&] { layout.write_commands(message, nan_actions); });

        if (argc == 3) {
            bodydemo03::Bodydemo03Policy policy(argv[2], cfg);
            auto action = policy.act({{"obs", obs}});
            require(action.size() == 16, "ONNX must produce 16 actions");
            env.action_manager->process_action(action);
            policy.write_commands(message, env.action_manager->processed_actions());
            rejects([&] { policy.act({{"obs", std::vector<float>(242)}}); });
            std::cout << "ONNX 55 -> 16 inference and input-size validation passed\n";
        }
        std::cout << "Observation ordering, action scales and SDK motor mapping passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
