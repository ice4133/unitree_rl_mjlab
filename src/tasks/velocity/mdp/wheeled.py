"""Stepping rewards and wheel diagnostics shared by wheeled velocity policies."""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch
from mjlab.managers import SceneEntityCfg
from mjlab.utils.lab_api.math import quat_apply

from src.tasks.velocity.mdp.rewards import feet_air_time, feet_gait

if TYPE_CHECKING:
  from mjlab.envs import ManagerBasedRlEnv


def lateral_step_weight(env: ManagerBasedRlEnv, command_name: str) -> torch.Tensor:
  """Blend stepping guidance in between 0.1 and 0.2 m/s lateral commands."""
  command = env.command_manager.get_command(command_name)
  return ((command[:, 1].abs() - 0.1) / 0.1).clamp(0.0, 1.0)


def maneuver_step_weight(env: ManagerBasedRlEnv, command_name: str) -> torch.Tensor:
  """Guide sidesteps and step-assisted turns, with no guidance for pure X rolling."""
  command = env.command_manager.get_command(command_name)
  turning = ((command[:, 2].abs() - 0.1) / 0.2).clamp(0.0, 1.0)
  return torch.maximum(lateral_step_weight(env, command_name), turning)


def lateral_gait(
  env: ManagerBasedRlEnv,
  command_name: str,
  sensor_name: str,
  period: float,
  offset: list[float],
  threshold: float,
  include_turning: bool = False,
) -> torch.Tensor:
  """Guide sidesteps and optionally turns without forcing a gait when rolling."""
  weight = maneuver_step_weight if include_turning else lateral_step_weight
  return weight(env, command_name) * feet_gait(
    env,
    period=period,
    offset=offset,
    threshold=threshold,
    command_threshold=0.1,
    command_name=command_name,
    sensor_name=sensor_name,
  )


def lateral_air_time(
  env: ManagerBasedRlEnv,
  command_name: str,
  sensor_name: str,
  threshold: float,
  include_turning: bool = False,
) -> torch.Tensor:
  """Encourage alternating support for sidesteps and optionally turns."""
  weight = maneuver_step_weight if include_turning else lateral_step_weight
  return weight(env, command_name) * feet_air_time(
    env,
    sensor_name=sensor_name,
    command_name=command_name,
    threshold=threshold,
  )


def track_lateral_velocity(
  env: ManagerBasedRlEnv,
  command_name: str,
  std: float,
) -> torch.Tensor:
  """Keep lateral tracking visible when mixed forward/yaw commands are sampled."""
  command = env.command_manager.get_command(command_name)
  actual = env.scene["robot"].data.root_link_lin_vel_b[:, 1]
  return lateral_step_weight(env, command_name) * torch.exp(
    -(command[:, 1] - actual).square() / std**2
  )


def wheel_lateral_slip(
  env: ManagerBasedRlEnv,
  sensor_name: str,
  asset_cfg: SceneEntityCfg,
) -> torch.Tensor:
  """Penalize axle-direction motion in contact, allowing forward wheel rolling.

  Sites are on wheel axles. Wheel-joint axes must be parallel to local Y;
  the sign cancels in the squared projection. Rotating Y by wheel orientation
  handles hip tilt, yaw and arbitrary wheel spin. Bodies, sites and sensor
  primaries must share order.
  """
  robot = env.scene[asset_cfg.name]
  wheel_quat = robot.data.body_link_quat_w[:, asset_cfg.body_ids]
  local_axle = torch.zeros_like(wheel_quat[..., :3])
  local_axle[..., 1] = 1.0
  axle_w = quat_apply(wheel_quat, local_axle)
  wheel_velocity = robot.data.site_lin_vel_w[:, asset_cfg.site_ids]
  lateral_velocity = (wheel_velocity * axle_w).sum(dim=-1)
  found = env.scene[sensor_name].data.found
  return (lateral_velocity.square() * (found > 0)).sum(dim=-1)


def lateral_velocity_error(env: ManagerBasedRlEnv, command_name: str) -> torch.Tensor:
  command = env.command_manager.get_command(command_name)
  return (command[:, 1] - env.scene["robot"].data.root_link_lin_vel_b[:, 1]).abs()
