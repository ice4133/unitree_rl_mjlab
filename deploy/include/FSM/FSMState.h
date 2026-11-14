#pragma once

#include "Types.h"
#include "param.h"
#include "FSM/BaseState.h"
#include "isaaclab/devices/keyboard/keyboard.h"
#include "unitree_joystick_dsl.hpp"

class FSMState : public BaseState
{
public:
    FSMState(int state, std::string state_string) 
    : BaseState(state, state_string) 
    {
        spdlog::info("Initializing State_{} ...", state_string);

        auto transitions = param::config["FSM"][state_string]["transitions"];

        if(transitions)
        {
            auto transition_map = transitions.as<std::map<std::string, std::string>>();

            for(auto it = transition_map.begin(); it != transition_map.end(); ++it)
            {
                std::string target_fsm = it->first;
                if(!FSMStringMap.right.count(target_fsm))
                {
                    spdlog::warn("FSM State_'{}' not found in FSMStringMap!", target_fsm);
                    continue;
                }

                int fsm_id = FSMStringMap.right.at(target_fsm);

                std::string condition = it->second;
                unitree::common::dsl::Parser p(condition);
                auto ast = p.Parse();
                auto func = unitree::common::dsl::Compile(*ast);
                registered_checks.emplace_back(
                    std::make_pair(
                        [func]()->bool{ return func(FSMState::lowstate->joystick); },
                        fsm_id
                    )
                );
            }
        }

        // register for all states
        registered_checks.emplace_back(
            std::make_pair(
                []()->bool{ return lowstate->isTimeout(); },
                FSMStringMap.right.at("Passive")
            )
        );
    }

	void pre_run()
	{
		if (keyboard) keyboard->update();

		const auto control = param::config["control"];
		const auto mode = control ? control["mode"].as<std::string>("joystick") : "joystick";
		const auto key = keyboard ? keyboard->key() : std::string{};
		const bool state_key = key == "1" || key == "2" || key == "3" || key == "0";
		// In auto, a terminal state key temporarily takes precedence over gamepad buttons.
		if (keyboard && (mode == "keyboard" || (mode == "auto" && state_key)))
		{
			unitree::common::REMOTE_DATA_RX input{};
			auto &buttons = input.RF_RX.btn.components;

			// 状态切换
			if (key == "1") {
				buttons.L2 = 1;
				buttons.up = 1;
			}

			if (key == "2") {
				buttons.R2 = 1;
				buttons.A = 1;
			}

			if (key == "3") {
				buttons.R1 = 1;
				buttons.A = 1;
			}

			if (key == "0") {
				buttons.L2 = 1;
				buttons.B = 1;
			}

			if (mode == "keyboard") {
				if (key == "w") input.RF_RX.ly =  0.3f;
				if (key == "s") input.RF_RX.ly = -0.3f;
				if (key == "a") input.RF_RX.lx = -0.3f;
				if (key == "d") input.RF_RX.lx =  0.3f;
				if (key == "q") input.RF_RX.rx = -0.3f;
				if (key == "e") input.RF_RX.rx =  0.3f;
			}

			auto &joystick = lowstate->joystick;

			// 键盘模拟的扳机立即生效，与按钮的按下沿同步
			const auto lt_smooth = joystick.LT.smooth;
			const auto rt_smooth = joystick.RT.smooth;
			joystick.LT.smooth = 1.0f;
			joystick.RT.smooth = 1.0f;

			joystick.extract(input);
			joystick.LT.smooth = lt_smooth;
			joystick.RT.smooth = rt_smooth;
		}
		else
		{
			lowstate->update();
		}
	}
    void post_run()
    {
        lowcmd->unlockAndPublish();
    }

    static std::unique_ptr<LowCmd_t> lowcmd;
    static std::shared_ptr<LowState_t> lowstate;
    static std::shared_ptr<Keyboard> keyboard;
};
