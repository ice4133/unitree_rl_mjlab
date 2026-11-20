"""Regression checks for the stationary training objective, without simulation."""

from types import SimpleNamespace
import unittest

import torch

from mjlab.managers import SceneEntityCfg

from src.tasks.velocity.config.bodydemo03.env_cfgs import (
  bodydemo03_flat_env_cfg,
  bodydemo03_rough_env_cfg,
)
from src.tasks.velocity.config.bodydemo03 import rewards
from src.tasks.velocity.config.go2w.env_cfgs import unitree_go2w_flat_env_cfg


class StandingObjectiveTest(unittest.TestCase):
  def setUp(self):
    # Rows cover zero, forward, yaw-only and small stationary commands.
    self.command = torch.tensor([
      [0.0, 0.0, 0.0], [0.3, 0.0, 0.0], [0.0, 0.0, 0.3], [0.01, 0.0, 0.0]
    ])
    self.data = SimpleNamespace(
      joint_vel=torch.zeros(4, 16),
      joint_vel_target=torch.zeros(4, 16),
      root_link_lin_vel_b=torch.zeros(4, 3),
      root_link_ang_vel_b=torch.zeros(4, 3),
      site_pos_w=torch.zeros(4, 4, 3),
    )
    self.force = torch.zeros(4, 4, 3)
    self.force[..., 2] = 55.0
    self.env = SimpleNamespace(
      scene={
        "robot": SimpleNamespace(data=self.data),
        "contact": SimpleNamespace(data=SimpleNamespace(force=self.force)),
      },
      command_manager=SimpleNamespace(get_command=lambda _: self.command),
    )
    self.asset = SceneEntityCfg("robot")
    self.asset.joint_ids = [3, 7, 11, 15]
    self.asset.site_ids = [0, 1, 2, 3]

  def test_motion_penalty_selects_wheels_and_includes_stalled_targets(self):
    self.data.joint_vel[:, 0] = 1000.0  # Leg motion is not a wheel penalty.
    self.data.joint_vel[:, 3] = 10.0
    self.data.joint_vel_target[:, 7] = 4.0  # Stalled wheel with nonzero target.
    cost = rewards.standing_wheel_motion_l2(self.env, self.asset, "twist")
    torch.testing.assert_close(cost, torch.tensor([104.0, 0.0, 0.0, 104.0]))

  def test_support_uses_load_and_is_bounded(self):
    self.force[..., 2] = torch.tensor([0.0, -55.0, 2.5, 5.0])
    cost = rewards.standing_support_deficit(self.env, "contact", "twist")
    torch.testing.assert_close(cost, torch.tensor([1.25, 0.0, 0.0, 1.25]))
    # Large contact/impact forces cannot create a positive reward.
    self.force[..., 2] = 10000.0
    torch.testing.assert_close(
      rewards.standing_support_deficit(self.env, "contact", "twist"), torch.zeros(4)
    )

  def test_base_drift_is_separate_from_moving_commands(self):
    self.data.root_link_lin_vel_b[:, 0] = 0.05
    self.data.root_link_ang_vel_b[:, 2] = 0.1
    torch.testing.assert_close(
      rewards.standing_base_motion_l2(self.env, self.asset, "twist"),
      torch.tensor([2.0, 0.0, 0.0, 2.0]),
    )

  def test_lowering_an_airborne_wheel_has_dense_feedback(self):
    self.data.site_pos_w[..., 2] = 0.088
    self.data.site_pos_w[:, 0, 2] += 0.01
    raised = rewards.standing_wheel_height_spread(self.env, self.asset, "twist")
    torch.testing.assert_close(raised, torch.tensor([0.2025, 0.0, 0.0, 0.2025]))
    self.data.site_pos_w[:, 0, 2] -= 0.004
    lowered = rewards.standing_wheel_height_spread(self.env, self.asset, "twist")
    self.assertLess(lowered[0].item(), raised[0].item())
    self.data.site_pos_w[:, 0, 2] += 10.0
    capped = rewards.standing_wheel_height_spread(self.env, self.asset, "twist")
    torch.testing.assert_close(capped, torch.tensor([1.0, 0.0, 0.0, 1.0]))

  def test_flat_objective_does_not_change_rough_or_go2w(self):
    flat = bodydemo03_flat_env_cfg()
    rough = bodydemo03_rough_env_cfg()
    go2w = unitree_go2w_flat_env_cfg()
    for name in ("stand_wheel_motion", "stand_wheel_support", "stand_base_motion", "stand_wheel_height"):
      self.assertIn(name, flat.rewards)
      self.assertNotIn(name, rough.rewards)
      self.assertNotIn(name, go2w.rewards)
    self.assertEqual(flat.commands["twist"].rel_standing_envs, 0.3)
    self.assertEqual(rough.commands["twist"].rel_standing_envs, 0.1)
    self.assertEqual(go2w.commands["twist"].rel_standing_envs, 0.1)
    self.assertEqual(tuple(flat.actions), ("joint_pos", "joint_vel"))
    self.assertEqual(tuple(flat.observations["actor"].terms),
      ("base_ang_vel", "projected_gravity", "command", "phase", "joint_pos", "joint_vel", "actions"))


if __name__ == "__main__":
  unittest.main()
