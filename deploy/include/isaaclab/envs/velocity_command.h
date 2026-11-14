#pragma once

#include <array>
#include <functional>

namespace isaaclab
{
// Body-frame vx [m/s], vy [m/s], yaw rate [rad/s].
using VelocityCommand = std::array<float, 3>;
using VelocityCommandSource = std::function<VelocityCommand()>;
} // namespace isaaclab
