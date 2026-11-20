"""Check that Bodydemo03 retains the original Go2W reward baseline."""

import unittest

from src.tasks.velocity.config.bodydemo03.env_cfgs import (
  bodydemo03_flat_env_cfg,
  bodydemo03_rough_env_cfg,
)
from src.tasks.velocity.config.go2w.env_cfgs import (
  unitree_go2w_flat_env_cfg,
  unitree_go2w_rough_env_cfg,
)


class Bodydemo03BaselineTest(unittest.TestCase):
  def test_rewards_and_metrics_match_go2w(self):
    for play in (False, True):
      for terrain, body_factory, go2w_factory in (
        ("flat", bodydemo03_flat_env_cfg, unitree_go2w_flat_env_cfg),
        ("rough", bodydemo03_rough_env_cfg, unitree_go2w_rough_env_cfg),
      ):
        with self.subTest(terrain=terrain, play=play):
          body = body_factory(play=play)
          go2w = go2w_factory(play=play)
          self.assertEqual(len(body.rewards), 12)
          self.assertEqual(set(body.rewards), set(go2w.rewards))
          for name, term in body.rewards.items():
            self.assertEqual(term.weight, go2w.rewards[name].weight, name)
            self.assertIs(term.func, go2w.rewards[name].func, name)
          self.assertEqual(set(body.metrics), set(go2w.metrics))
          for name, term in body.metrics.items():
            self.assertIs(term.func, go2w.metrics[name].func, name)
          self.assertEqual(body.commands["twist"].rel_standing_envs, 0.1)


if __name__ == "__main__":
  unittest.main()
