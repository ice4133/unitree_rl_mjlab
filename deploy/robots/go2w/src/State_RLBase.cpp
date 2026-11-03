#include "FSM/State_RLBase.h"
#include "Go2WPolicy.h"
#include "unitree_articulation.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"

State_RLBase::State_RLBase(int state_mode, std::string state_string)
    : FSMState(state_mode, state_string)
{
    const auto cfg = param::config["FSM"][state_string];
    const auto policy_dir = param::parser_policy_dir(cfg["policy_dir"].as<std::string>());
    const auto deploy_cfg = YAML::LoadFile((policy_dir / "params" / "deploy.yaml").string());

    env = std::make_unique<isaaclab::ManagerBasedRLEnv>(
        deploy_cfg,
        std::make_shared<unitree::BaseArticulation<LowState_t::SharedPtr>>(FSMState::lowstate)
    );
    env->alg = std::make_unique<go2w::Go2WPolicy>(policy_dir / "exported" / "policy.onnx", deploy_cfg);

    registered_checks.emplace_back(
        [this] { return isaaclab::mdp::bad_orientation(env.get(), 1.0); },
        FSMStringMap.right.at("Passive")
    );
}

void State_RLBase::run()
{
    const auto actions = env->action_manager->processed_actions();
    const auto &policy = static_cast<const go2w::Go2WPolicy &>(*env->alg);
    policy.write_commands(lowcmd->msg_, actions);
}
