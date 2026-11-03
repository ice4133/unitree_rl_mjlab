#pragma once

#include "unitree/dds_wrapper/robots/go2/go2.h"

// Go2W uses the Go2 DDS message format, with motor slots 0..15 populated.
using LowCmd_t = unitree::robot::go2::publisher::LowCmd;
using LowState_t = unitree::robot::go2::subscription::LowState;
