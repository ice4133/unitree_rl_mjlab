#include "FSM/CtrlFSM.h"
#include "FSM/State_Passive.h"
#include "FSM/State_FixStand.h"
#include "FSM/State_RLBase.h"

std::unique_ptr<LowCmd_t> FSMState::lowcmd = nullptr;
std::shared_ptr<LowState_t> FSMState::lowstate = nullptr;
std::shared_ptr<Keyboard> FSMState::keyboard = nullptr;

void init_fsm_state()
{
    auto lowcmd_sub = std::make_shared<unitree::robot::go2::subscription::LowCmd>();
    usleep(0.2 * 1e6);
    if(!lowcmd_sub->isTimeout())
    {
        spdlog::critical("The other process is using the lowcmd channel, please close it first.");
        unitree::robot::go2::shutdown();
        // exit(0);
    }
    FSMState::lowcmd = std::make_unique<LowCmd_t>();
    FSMState::lowstate = std::make_shared<LowState_t>();
    spdlog::info("Waiting for connection to robot...");
    FSMState::lowstate->wait_for_connection();
    spdlog::info("Connected to robot.");
}

int main(int argc, char** argv)
{
    // Load parameters
    auto vm = param::helper(argc, argv);

    if (param::config["keyboard_control"].as<bool>(false))
    {
        FSMState::keyboard = std::make_shared<Keyboard>();
    }

    std::cout << " --- Unitree Robotics --- \n";
    std::cout << "     Go2 Controller \n";

    // Unitree DDS Config
    unitree::robot::ChannelFactory::Instance()->Init(0, vm["network"].as<std::string>());

    init_fsm_state();

    // Initialize FSM
    auto fsm = std::make_unique<CtrlFSM>(param::config["FSM"]);
    fsm->start();

    if (FSMState::keyboard)
    {
        std::cout << "Keyboard control: use this terminal for input.\n";
        std::cout << "Press [1] to enter FixStand, then [2] to enter Velocity.\n";
        std::cout << "Hold [W/S] to move forward/backward, [A/D] to move left/right, [Q/E] to turn.\n";
        std::cout << "Press [0] to enter Passive (damping) mode.\n";
    }
    else
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

