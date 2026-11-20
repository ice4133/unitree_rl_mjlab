#pragma once

#include <memory>
#include <string>
#include "isaaclab/envs/velocity_command.h"

namespace bodydemo03
{
class RosVelocityCommand
{
public:
    RosVelocityCommand(const std::string& topic, double timeout, int domain_id);
    ~RosVelocityCommand();
    isaaclab::VelocityCommand read() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace bodydemo03
