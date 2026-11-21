"""Bodydemo03 Flat posture and rolling guidance using its CAD joint axes."""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from src.tasks.velocity.mdp import wheeled
from src.tasks.velocity.mdp.rewards import variable_posture

if TYPE_CHECKING:
  from mjlab.envs import ManagerBasedRlEnv
  from mjlab.managers import SceneEntityCfg


# Positive signed angles move the wheel toward the body centerline at CAD zero.
# Front hip axes are -X, rear hip axes are +X; wheel axes are +Y left / -Y right.
HIP_INWARD_SIGNS = {"fl": 1.0, "fr": -1.0, "rl": -1.0, "rr": 1.0}
WHEEL_FORWARD_SIGNS = {"fl": 1.0, "fr": -1.0, "rl": 1.0, "rr": -1.0}


def wheel_support_fraction(
  env: ManagerBasedRlEnv, sensor_name: str, min_force: float = 5.0
) -> torch.Tensor:
  """Fraction of wheels actually bearing weight, rather than just touching."""
  data = env.scene[sensor_name].data
  supported = (data.found > 0) & (data.force[..., 2].abs() >= min_force)
  return supported.float().mean(dim=1)


def wheel_support_deficit(
  env: ManagerBasedRlEnv,
  command_name: str,
  sensor_name: str,
  min_force: float = 5.0,
  maneuver_threshold: float = 0.1,
) -> torch.Tensor:
  """Require four loaded wheels at rest/rolling, allowing maneuver swings."""
  data = env.scene[sensor_name].data
  load = data.force[..., 2].abs() * (data.found > 0)
  missing_load = (1.0 - load / min_force).clamp(0.0, 1.0).sum(dim=1)
  command = env.command_manager.get_command(command_name)
  # Even the minimum trained lateral speed (0.15 m/s) must allow full swings.
  # Blending this strong penalty with gait weight would punish those steps.
  rolling = (command[:, 1].abs() <= maneuver_threshold) & (
    command[:, 2].abs() <= maneuver_threshold
  )
  return missing_load * rolling


def stationary_wheel_velocity_l2(
  env: ManagerBasedRlEnv,
  command_name: str,
  asset_cfg: SceneEntityCfg,
  command_threshold: float = 0.1,
) -> torch.Tensor:
  """Penalize steady wheel spin under a zero command, including airborne wheels."""
  command = env.command_manager.get_command(command_name)
  standing = torch.linalg.vector_norm(command, dim=1) < command_threshold
  velocity = env.scene[asset_cfg.name].data.joint_vel[:, asset_cfg.joint_ids]
  return velocity.square().mean(dim=1) * standing


def wheel_air_time_excess(
  env: ManagerBasedRlEnv,
  sensor_name: str,
  max_air_time: float = 0.35,
  time_scale: float = 0.2,
) -> torch.Tensor:
  """Allow normal swings, but penalize a wheel that does not return to support."""
  air_time = env.scene[sensor_name].data.current_air_time
  excess = ((air_time - max_air_time) / time_scale).clamp(0.0, 2.0)
  return excess.square().sum(dim=1)


def max_wheel_air_time(env: ManagerBasedRlEnv, sensor_name: str) -> torch.Tensor:
  return env.scene[sensor_name].data.current_air_time.amax(dim=1)


def wheel_air_time_limit(
  env: ManagerBasedRlEnv, sensor_name: str, max_air_time: float = 0.8
) -> torch.Tensor:
  """End sustained unsupported-leg episodes without forbidding ordinary steps."""
  return max_wheel_air_time(env, sensor_name) > max_air_time


class upright_posture(variable_posture):
  """Keep the neutral posture and reduce its reward further for inward hips."""

  def __init__(self, cfg, env: ManagerBasedRlEnv):
    super().__init__(cfg, env)
    robot = env.scene[cfg.params["asset_cfg"].name]
    self.hip_ids, names = robot.find_joints(r"^(fl|fr|rl|rr)_hip_joint$")
    self.inward_signs = torch.tensor(
      [HIP_INWARD_SIGNS[name.split("_", 1)[0]] for name in names], device=env.device
    )

  def __call__(
    self,
    env: ManagerBasedRlEnv,
    inward_tolerance: float = 0.03,
    inward_std: float = 0.05,
    **params,
  ) -> torch.Tensor:
    posture = super().__call__(env, **params)
    robot = env.scene[params["asset_cfg"].name]
    signed_hips = robot.data.joint_pos[:, self.hip_ids] * self.inward_signs
    inward_error = (signed_hips - inward_tolerance).clamp_min(0.0)
    return posture * torch.exp(-(inward_error / inward_std).square().mean(dim=1))


class locomotion_gait:
  """Use the existing gait term for X rolling or lateral/yaw stepping.

  Rolling requires four-wheel support, measured wheel speeds with the correct
  left/right signs, and matching body progress. Spinning in place or walking
  with stationary wheels cannot earn the full rolling reward.
  """

  def __init__(self, cfg, env: ManagerBasedRlEnv):
    robot = env.scene[cfg.params["asset_cfg"].name]
    _, names = robot.find_joints(cfg.params["asset_cfg"].joint_names)
    self.wheel_signs = torch.tensor(
      [WHEEL_FORWARD_SIGNS[name.split("_", 1)[0]] for name in names], device=env.device
    )

  def __call__(
    self,
    env: ManagerBasedRlEnv,
    command_name: str,
    sensor_name: str,
    period: float,
    offset: list[float],
    threshold: float,
    asset_cfg: SceneEntityCfg,
    wheel_radius: float,
    rolling_velocity_std: float = 0.15,
    include_turning: bool = True,
  ) -> torch.Tensor:
    stepping = wheeled.lateral_gait(
      env, command_name, sensor_name, period, offset, threshold, include_turning
    )
    step_weight = (
      wheeled.maneuver_step_weight(env, command_name)
      if include_turning
      else wheeled.lateral_step_weight(env, command_name)
    )
    command = env.command_manager.get_command(command_name)
    robot = env.scene[asset_cfg.name]
    wheel_speeds = (
      robot.data.joint_vel[:, asset_cfg.joint_ids] * self.wheel_signs * wheel_radius
    )
    target = command[:, 0]
    wheel_error = (wheel_speeds - target.unsqueeze(1)).square().mean(dim=1)
    body_error = (robot.data.root_link_lin_vel_b[:, 0] - target).square()
    supported = (env.scene[sensor_name].data.found > 0).all(dim=1).float()
    rolling = supported * torch.exp(
      -(wheel_error + body_error) / rolling_velocity_std**2
    )
    # No rolling bonus at rest. Blend it out as lateral/yaw stepping activates.
    rolling_weight = ((target.abs() - 0.05) / 0.1).clamp(0.0, 1.0)
    return stepping + (1.0 - step_weight) * rolling_weight * rolling
