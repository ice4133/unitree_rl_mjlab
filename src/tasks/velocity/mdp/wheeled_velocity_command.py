"""Velocity sampling for rolling, sidestepping, turning and mixed commands."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import torch
from mjlab.tasks.velocity.mdp import UniformVelocityCommand, UniformVelocityCommandCfg

if TYPE_CHECKING:
  from mjlab.envs import ManagerBasedRlEnv


class WheeledVelocityCommand(UniformVelocityCommand):
  cfg: WheeledVelocityCommandCfg

  def _resample_command(self, env_ids: torch.Tensor) -> None:
    super()._resample_command(env_ids)
    # Uniform 3-axis sampling almost never produces pure lateral or yaw commands.
    # Preserve mixed commands in the remaining fraction of non-standing samples.
    mode = torch.rand(len(env_ids), device=self.device)
    forward_end = self.cfg.forward_fraction
    lateral_end = forward_end + self.cfg.lateral_fraction
    turning_end = lateral_end + self.cfg.turning_fraction
    forward = env_ids[mode < forward_end]
    lateral = env_ids[(mode >= forward_end) & (mode < lateral_end)]
    turning = env_ids[(mode >= lateral_end) & (mode < turning_end)]
    self.vel_command_b[forward, 1:] = 0.0
    self.vel_command_b[lateral, 0] = 0.0
    self.vel_command_b[lateral, 2] = 0.0
    self.vel_command_b[turning, :2] = 0.0

    # Pure lateral samples must include a useful stepping command, not mostly
    # near-zero commands below the gait reward's activation threshold.
    lower, upper = self.cfg.ranges.lin_vel_y
    if lower < 0.0 < upper:
      values = self.vel_command_b[lateral, 1]
      minimum = min(self.cfg.min_lateral_speed, -lower, upper)
      self.vel_command_b[lateral, 1] = torch.where(
        values < 0.0, values.clamp(max=-minimum), values.clamp(min=minimum)
      )

    # The base class applies the standing mask in _update_command(). Do not
    # change that mask: standing keeps its independently sampled probability.


@dataclass(kw_only=True)
class WheeledVelocityCommandCfg(UniformVelocityCommandCfg):
  forward_fraction: float = 0.3
  lateral_fraction: float = 0.3
  turning_fraction: float = 0.1
  min_lateral_speed: float = 0.15

  def __post_init__(self):
    super().__post_init__()
    fractions = (self.forward_fraction, self.lateral_fraction, self.turning_fraction)
    if any(value < 0.0 for value in fractions) or sum(fractions) > 1.0:
      raise ValueError("Command fractions must be nonnegative and sum to at most 1.")
    if self.min_lateral_speed < 0.0:
      raise ValueError("min_lateral_speed must be nonnegative.")
    if self.heading_command or self.init_velocity_prob != 0.0:
      raise ValueError(
        "Wheeled command mixing requires direct yaw commands and no velocity injection."
      )

  def build(self, env: ManagerBasedRlEnv) -> WheeledVelocityCommand:
    return WheeledVelocityCommand(self, env)
