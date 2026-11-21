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
