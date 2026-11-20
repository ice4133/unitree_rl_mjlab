"""Check that Bodydemo03 retains the original Go2W reward baseline."""

import unittest

from src.tasks.velocity.config.bodydemo03.env_cfgs import (
  bodydemo03_flat_env_cfg,
  bodydemo03_rough_env_cfg,
)


class Bodydemo03BaselineTest(unittest.TestCase):
  def test_original_reward_baseline_is_preserved(self):
    # Go2W now trains lateral stepping. Bodydemo03 keeps its rolling template;
    # compare with explicit baseline values rather than the evolving Go2W task.
    weights = {
      "track_linear_velocity": 3.0,
      "track_angular_velocity": 1.5,
      "body_orientation_l2": -1.0,
      "pose": 1.0,
      "body_ang_vel": -0.05,
      "is_terminated": -200.0,
      "joint_acc_l2": -2.5e-7,
      "joint_pos_limits": -5.0,
      "action_rate_l2": -0.01,
      "stand_still": -2.0,
      "wheel_acc_l2": -2.5e-9,
      "leg_torques_l2": -2.5e-5,
    }
    for play in (False, True):
      for terrain, body_factory in (
        ("flat", bodydemo03_flat_env_cfg),
        ("rough", bodydemo03_rough_env_cfg),
      ):
        with self.subTest(terrain=terrain, play=play):
          body = body_factory(play=play)
          expected = dict(weights)
          metrics = {"mean_action_acc"}
          if terrain == "rough":
            expected.update(pose=0.5, stand_still=-0.5)
            metrics.update(("leg_joint_vel_l2", "wheel_joint_vel_l2"))
          self.assertEqual(
            {name: term.weight for name, term in body.rewards.items()}, expected
          )
          self.assertEqual(set(body.metrics), metrics)
          self.assertEqual(body.commands["twist"].rel_standing_envs, 0.1)
          self.assertEqual(body.commands["twist"].ranges.lin_vel_y, (0.0, 0.0))


if __name__ == "__main__":
  unittest.main()
