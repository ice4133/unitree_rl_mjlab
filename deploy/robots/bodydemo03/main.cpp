#include "FSM/CtrlFSM.h"
#include "FSM/State_Passive.h"
#include "FSM/State_FixStand.h"
#include "Bodydemo03VelocityState.h"

std::unique_ptr<LowCmd_t> FSMState::lowcmd = nullptr;
std::shared_ptr<LowState_t> FSMState::lowstate = nullptr;
std::shared_ptr<Keyboard> FSMState::keyboard = nullptr;

bool init_fsm_state()
{
    auto lowcmd_sub = std::make_shared<unitree::robot::go2::subscription::LowCmd>();
    usleep(0.2 * 1e6);
    if(!lowcmd_sub->isTimeout())
    {
        spdlog::critical("Another controller is publishing rt/lowcmd. Stop go2w_ctrl, any other "
                         "bodydemo03_ctrl or other command publisher, then restart.");
        return false;
    }
    FSMState::lowcmd = std::make_unique<LowCmd_t>();
    FSMState::lowstate = std::make_shared<LowState_t>();
    spdlog::info("Waiting for connection to robot...");
    FSMState::lowstate->wait_for_connection();
    spdlog::info("Connected to robot.");
    return true;
}

int main(int argc, char** argv)
{
    // Load parameters
    auto vm = param::helper(argc, argv);

    const auto control = param::config["control"];
    const auto mode = control ? control["mode"].as<std::string>("joystick") : "joystick";
    if (mode != "joystick" && mode != "keyboard" && mode != "auto") {
        spdlog::error("config.yaml control.mode must be joystick, keyboard or auto");
        return EXIT_FAILURE;
    }
    spdlog::info("Velocity control mode: {}", mode);

    std::cout << " --- Unitree Robotics --- \n";
    std::cout << "     Bodydemo03 Controller \n";

    // Unitree DDS Config
    unitree::robot::ChannelFactory::Instance()->Init(0, vm["network"].as<std::string>());

    if (!init_fsm_state())
        return EXIT_FAILURE;

    if (mode == "keyboard" || (mode == "auto" && isatty(STDIN_FILENO)))
    {
        FSMState::keyboard = std::make_shared<Keyboard>();
    }

    // Initialize FSM
    auto fsm = std::make_unique<CtrlFSM>(param::config["FSM"]);
    fsm->start();

    if (mode == "auto")
        std::cout << "Automatic velocity input enabled. Stand and enter Velocity using the state buttons/keys; "
                     "manual movement axes are ignored.\n";

    if (FSMState::keyboard)
    {
        std::cout << "Use this terminal for state keys.\n";
        std::cout << "Press [1] to enter FixStand, then [2] to enter Velocity.\n";
        if (mode != "auto")
            std::cout << "Hold [W/S] to move forward/backward, [A/D] to move left/right, [Q/E] to turn.\n";
        std::cout << "Press [0] to enter Passive (damping) mode.\n";
    }
    if (!FSMState::keyboard || mode == "auto")
    {
        std::cout << "Press [L2 + up] to enter FixStand mode.\n";
        std::cout << "And then press [R2 + A] to start controlling the robot.\n";
    }

    while (true)
    {
        sleep(1);
    }
    
    return 0;
}

