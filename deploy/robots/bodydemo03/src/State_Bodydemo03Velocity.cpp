#include "Bodydemo03VelocityState.h"
#include "Bodydemo03Policy.h"
#include "RosVelocityCommand.h"
#include "unitree_articulation.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include "isaaclab/envs/mdp/terminations.h"

State_Bodydemo03Velocity::State_Bodydemo03Velocity(int state_mode, std::string state_string)
    : FSMState(state_mode, state_string),
      transition_(param::config["FSM"][state_string]["transition_time"].as<double>(0.5))
{
    const auto cfg = param::config["FSM"][state_string];
    const auto policy_dir = param::parser_policy_dir(cfg["policy_dir"].as<std::string>());
    const auto deploy_cfg = YAML::LoadFile((policy_dir / "params" / "deploy.yaml").string());

    isaaclab::VelocityCommandSource velocity_source;
    const auto control = param::config["control"];
    if (control && control["mode"].as<std::string>("joystick") == "auto") {
#ifdef UNITREE_WITH_ROS2_COMMANDS
        const auto ros = control["ros"] ? control["ros"] : YAML::Node(YAML::NodeType::Map);
        auto receiver = std::make_shared<bodydemo03::RosVelocityCommand>(
            ros["topic"].as<std::string>("/cmd_vel"),
            ros["timeout"].as<double>(0.5), ros["domain_id"].as<int>(10));
        velocity_source = [receiver] { return receiver->read(); };
#else
        throw std::runtime_error("Automatic velocity requires ENABLE_ROS2_COMMANDS=ON");
#endif
    }

    env_ = std::make_unique<isaaclab::ManagerBasedRLEnv>(
        deploy_cfg,
        std::make_shared<unitree::BaseArticulation<LowState_t::SharedPtr>>(FSMState::lowstate),
        std::move(velocity_source)
    );
    env_->alg = std::make_unique<bodydemo03::Bodydemo03Policy>(policy_dir / "exported" / "policy.onnx", deploy_cfg);

    registered_checks.emplace_back(
        [this] {
            return inference_failed_ || bad_orientation_;
        },
        FSMStringMap.right.at("Passive")
    );
}

void State_Bodydemo03Velocity::enter()
{
    exit();
    {
        std::lock_guard<std::mutex> lock(policy_mutex_);
        env_->reset();
        processed_actions_.clear();
    }
    // Preserve the preceding FixStand targets until a complete inference exists.
    // Re-entering this state always starts a fresh transition and action history.
    transition_.reset(lowcmd->msg_);
    entered_ = Clock::now();
    inference_failed_ = false;
    bad_orientation_ = isaaclab::mdp::bad_orientation(env_.get(), 1.0);
    running_ = true;
    policy_thread_ = std::thread([this] {
        const auto dt = std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(env_->step_dt));
        auto next = Clock::now();
        while (running_)
        {
            try
            {
                env_->step();
                auto actions = env_->action_manager->processed_actions();
                bad_orientation_ = isaaclab::mdp::bad_orientation(env_.get(), 1.0);
                // Publish a complete snapshot. Inference never holds the mutex
                // used by the 1 kHz FSM and its state/stop key handling.
                std::lock_guard<std::mutex> lock(policy_mutex_);
                processed_actions_ = std::move(actions);
            }
            catch (const std::exception &error)
            {
                spdlog::error("Bodydemo03 policy failed: {}", error.what());
                inference_failed_ = true;
                running_ = false;
                break;
            }
            next += dt;
            std::this_thread::sleep_until(next);
        }
    });
}

void State_Bodydemo03Velocity::run()
{
    std::vector<float> actions;
    {
        std::lock_guard<std::mutex> lock(policy_mutex_);
        actions = processed_actions_;
    }
    if (actions.empty()) return;
    const auto &policy = static_cast<const bodydemo03::Bodydemo03Policy &>(*env_->alg);
    policy.write_commands(lowcmd->msg_, actions);
    transition_.apply(lowcmd->msg_, std::chrono::duration<double>(Clock::now() - entered_).count());
}

void State_Bodydemo03Velocity::exit()
{
    running_ = false;
    if (policy_thread_.joinable()) policy_thread_.join();
}
