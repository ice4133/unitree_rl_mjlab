"""Bodydemo03 stationary objectives; no changes to policy inputs or controls."""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from mjlab.managers import SceneEntityCfg

if TYPE_CHECKING:
  from mjlab.envs import ManagerBasedRlEnv


def _standing_mask(
  env: ManagerBasedRlEnv, command_name: str, command_threshold: float
) -> torch.Tensor:
  command = env.command_manager.get_command(command_name)
  assert command is not None
  # Match the stationary-command definition used by the phase observation.
  return torch.linalg.vector_norm(command, dim=-1) < command_threshold


def standing_wheel_motion_l2(
  env: ManagerBasedRlEnv,
  asset_cfg: SceneEntityCfg,
  command_name: str,
  command_threshold: float = 0.1,
  target_weight: float = 0.25,
) -> torch.Tensor:
  """Penalize actual wheel motion and nonzero wheel targets while standing.

  Including velocity targets also gives stalled, loaded wheels a cost when the
  policy keeps commanding them to turn. Continuous wheel angles are not used.
  """
  robot = env.scene[asset_cfg.name]
  ids = asset_cfg.joint_ids
  velocity = robot.data.joint_vel[:, ids]
  target = robot.data.joint_vel_target[:, ids]
  cost = (velocity.square() + target_weight * target.square()).sum(dim=-1)
  return cost * _standing_mask(env, command_name, command_threshold)


def standing_support_deficit(
  env: ManagerBasedRlEnv,
  sensor_name: str,
  command_name: str,
  command_threshold: float = 0.1,
  min_normal_force: float = 5.0,
) -> torch.Tensor:
  """Charge up to one unit per unloaded wheel under a stationary command.

  Use vertical force from the netforce sensor, not merely a geometric contact
  count: grazing contact without meaningful load must not satisfy this objective.
  Loads above the threshold receive no further benefit, so equal loading is not
  required and impacts cannot earn additional reward.
  """
  if min_normal_force <= 0:
    raise ValueError("min_normal_force must be positive")
  forces = env.scene[sensor_name].data.force
  assert forces is not None
  normal_force = forces[..., 2].abs()
  deficit = (1.0 - normal_force / min_normal_force).clamp(0.0, 1.0)
  cost = deficit.square().sum(dim=-1)
  return cost * _standing_mask(env, command_name, command_threshold)


def standing_base_motion_l2(
  env: ManagerBasedRlEnv,
  asset_cfg: SceneEntityCfg,
  command_name: str,
  command_threshold: float = 0.1,
  linear_scale: float = 0.05,
  angular_scale: float = 0.1,
) -> torch.Tensor:
  """Make small planar drift and yaw motion matter under zero commands."""
  if linear_scale <= 0 or angular_scale <= 0:
    raise ValueError("Standing velocity scales must be positive")
  data = env.scene[asset_cfg.name].data
  linear = data.root_link_lin_vel_b[:, :2] / linear_scale
  angular = data.root_link_ang_vel_b[:, 2] / angular_scale
  cost = linear.square().sum(dim=-1) + angular.square()
  return cost * _standing_mask(env, command_name, command_threshold)


def standing_wheel_height_spread(
  env: ManagerBasedRlEnv,
  asset_cfg: SceneEntityCfg,
  command_name: str,
  command_threshold: float = 0.1,
  height_scale: float = 0.02,
  height_tolerance: float = 0.001,
) -> torch.Tensor:
  """Give an airborne wheel dense feedback as it approaches the flat ground.

  Equal-radius wheel axles should have similar heights on a plane. A small
  tolerance permits mesh/contact compliance; each wheel's cost is capped at one
  to avoid extreme penalties during resets or a fall. Do not use on rough terrain.
  """
  if height_scale <= 0 or height_tolerance < 0:
    raise ValueError("Invalid stationary wheel height scale or tolerance")
  height = env.scene[asset_cfg.name].data.site_pos_w[:, asset_cfg.site_ids, 2]
  raised = (height - height.amin(dim=-1, keepdim=True) - height_tolerance).clamp(min=0)
  cost = (raised / height_scale).clamp(max=1.0).square().sum(dim=-1)
  return cost * _standing_mask(env, command_name, command_threshold)


def wheel_support_fraction(
  env: ManagerBasedRlEnv,
  sensor_name: str,
  min_normal_force: float = 5.0,
) -> torch.Tensor:
  """Fraction of wheels carrying load, logged for all command states."""
  forces = env.scene[sensor_name].data.force
  assert forces is not None
  return (forces[..., 2].abs() >= min_normal_force).float().mean(dim=-1)


def all_wheels_supported(
  env: ManagerBasedRlEnv,
  sensor_name: str,
  min_normal_force: float = 5.0,
) -> torch.Tensor:
  forces = env.scene[sensor_name].data.force
  assert forces is not None
  return (forces[..., 2].abs() >= min_normal_force).all(dim=-1).float()
