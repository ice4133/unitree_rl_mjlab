#pragma once

#include "FSM/FSMState.h"
#include "Bodydemo03Policy.h"
#include "isaaclab/envs/manager_based_rl_env.h"

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

class State_Bodydemo03Velocity : public FSMState
{
public:
    State_Bodydemo03Velocity(int state_mode, std::string state_string);
    ~State_Bodydemo03Velocity() { exit(); }
    void enter() override;
    void run() override;
    void exit() override;

private:
    using Clock = std::chrono::steady_clock;
    std::unique_ptr<isaaclab::ManagerBasedRLEnv> env_;
    std::thread policy_thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> inference_failed_{false};
    std::atomic<bool> bad_orientation_{false};
    std::mutex policy_mutex_;
    std::vector<float> processed_actions_;
    bodydemo03::CommandTransition transition_;
    Clock::time_point entered_;
};

REGISTER_FSM(State_Bodydemo03Velocity)
