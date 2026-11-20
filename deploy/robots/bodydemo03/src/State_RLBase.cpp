#include "FSM/State_RLBase.h"
#include "Bodydemo03Policy.h"
#include "RosVelocityCommand.h"
#include "unitree_articulation.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"

State_RLBase::State_RLBase(int state_mode, std::string state_string)
    : FSMState(state_mode, state_string)
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

    env = std::make_unique<isaaclab::ManagerBasedRLEnv>(
        deploy_cfg,
        std::make_shared<unitree::BaseArticulation<LowState_t::SharedPtr>>(FSMState::lowstate),
        std::move(velocity_source)
    );
    env->alg = std::make_unique<bodydemo03::Bodydemo03Policy>(policy_dir / "exported" / "policy.onnx", deploy_cfg);

    registered_checks.emplace_back(
        [this] { return isaaclab::mdp::bad_orientation(env.get(), 1.0); },
        FSMStringMap.right.at("Passive")
    );
}

void State_RLBase::run()
{
    const auto actions = env->action_manager->processed_actions();
    const auto &policy = static_cast<const bodydemo03::Bodydemo03Policy &>(*env->alg);
    policy.write_commands(lowcmd->msg_, actions);
}
