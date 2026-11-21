"""Check Bodydemo03 Flat migration and preserve the untouched Rough baseline."""

import re
import unittest
from types import SimpleNamespace

import mujoco
import numpy as np
import torch
from mjlab.entity import Entity
from mjlab.managers import SceneEntityCfg
from mjlab.sensor import ContactSensor

from src.assets.robots.bodydemo03.bodydemo03_constants import BODYDEMO03_WHEEL_RADIUS
from src.tasks.velocity.config.bodydemo03 import mdp as body_mdp
from src.tasks.velocity.config.bodydemo03.env_cfgs import (
  bodydemo03_flat_env_cfg,
  bodydemo03_rough_env_cfg,
)
from src.tasks.velocity.config.go2w.env_cfgs import unitree_go2w_flat_env_cfg
from src.tasks.velocity.mdp.curriculums import commands_vel
from src.tasks.velocity.mdp.wheeled_velocity_command import WheeledVelocityCommandCfg


def reward_env(commands):
  commands = torch.tensor(commands, dtype=torch.float32)
  count = len(commands)
  names = [
    f"{leg}_{joint}_joint"
    for leg in ("fl", "fr", "rl", "rr")
    for joint in ("hip", "thigh", "calf", "foot")
  ]

  def find_joints(pattern):
    if isinstance(pattern, str):
      pattern = (pattern,)
    ids = [
      i
      for i, name in enumerate(names)
      if any(re.fullmatch(expr, name) for expr in pattern)
    ]
    return ids, [names[i] for i in ids]

  robot = SimpleNamespace(
    find_joints=find_joints,
    data=SimpleNamespace(
      default_joint_pos=torch.zeros(count, 16),
      joint_pos=torch.zeros(count, 16),
      joint_vel=torch.zeros(count, 16),
      root_link_lin_vel_b=torch.zeros(count, 3),
    ),
  )
  contact = SimpleNamespace(
    data=SimpleNamespace(
      found=torch.ones(count, 4),
      force=torch.tensor([0.0, 0.0, 60.0]).expand(count, 4, 3).clone(),
      current_contact_time=torch.ones(count, 4) * 0.2,
      current_air_time=torch.zeros(count, 4),
    )
  )
  return SimpleNamespace(
    scene={"robot": robot, "feet_ground_contact": contact},
    command_manager=SimpleNamespace(get_command=lambda _: commands),
    episode_length_buf=torch.zeros(count),
    step_dt=0.02,
    device="cpu",
  )


class Bodydemo03BaselineTest(unittest.TestCase):
  def test_flat_matches_latest_go2w_locomotion_targets(self):
    for play in (False, True):
      with self.subTest(play=play):
        body = bodydemo03_flat_env_cfg(play=play)
        reference = unitree_go2w_flat_env_cfg(play=play)
        extra_rewards = {
          "wheel_support_deficit", "stationary_wheel_velocity", "wheel_air_time_excess"
        }
        self.assertEqual(set(body.rewards) - set(reference.rewards), extra_rewards)
        self.assertEqual(
          {name: body.rewards[name].weight for name in reference.rewards},
          {name: term.weight for name, term in reference.rewards.items()},
        )
        for name in reference.rewards:
          term = body.rewards[name]
          if name not in ("pose", "lateral_gait"):
            self.assertIs(term.func, reference.rewards[name].func)
          # Joint/body selectors differ between models. Other reward parameters,
          # including gait/yaw gates and posture tolerances, must stay aligned.
          for key, expected in reference.rewards[name].params.items():
            if key == "asset_cfg":
              continue
            value = term.params[key]
            if key.startswith("std_"):
              self.assertEqual(list(value.values()), list(expected.values()))
            else:
              self.assertEqual(value, expected)
        command = body.commands["twist"]
        self.assertIsInstance(command, WheeledVelocityCommandCfg)
        for key in (
          "ranges",
          "resampling_time_range",
          "rel_standing_envs",
          "forward_fraction",
          "lateral_fraction",
          "turning_fraction",
          "min_lateral_speed",
        ):
          self.assertEqual(
            getattr(command, key), getattr(reference.commands["twist"], key)
          )
        self.assertEqual(
          set(body.metrics) - set(reference.metrics),
          {"wheel_support_fraction", "max_wheel_air_time"},
        )
        self.assertIn("wheel_air_time_limit", body.terminations)
        self.assertNotIn("height_scan", body.observations["actor"].terms)
        self.assertEqual(set(body.curriculum), set(reference.curriculum))
        if not play:
          self.assertEqual(
            body.curriculum["command_vel"].params,
            reference.curriculum["command_vel"].params,
          )
          env = SimpleNamespace(
            common_step_counter=3000 * 24 + 1,
            command_manager=SimpleNamespace(
              get_term=lambda _, command=command: SimpleNamespace(cfg=command)
            ),
          )
          commands_vel(env, torch.tensor([0]), **body.curriculum["command_vel"].params)
          self.assertEqual(command.ranges.lin_vel_y, (-0.5, 0.5))
        # Bodydemo03's CAD zero and actuator limits must not be replaced by Go2W's.
        robot = body.scene.entities["robot"]
        self.assertEqual(robot.init_state.joint_pos, {".*": 0.0})
        self.assertEqual(robot.articulation.actuators[0].stiffness, 400.0)
        self.assertEqual(robot.articulation.actuators[1].effort_limit, 8.0)

  def test_flat_command_sampling_exposes_rolling_sidesteps_and_turns(self):
    command = bodydemo03_flat_env_cfg().commands["twist"]
    env = SimpleNamespace(num_envs=10000, device="cpu", scene={"robot": object()})
    term = command.build(env)
    torch.manual_seed(17)
    term._resample_command(torch.arange(env.num_envs))
    term._update_command()
    commands = term.command
    standing = (commands == 0).all(dim=1)
    self.assertAlmostEqual(standing.float().mean().item(), 0.1, delta=0.02)
    moving = commands[~standing]
    lateral = (moving[:, 0] == 0) & (moving[:, 2] == 0)
    forward = (moving[:, 1:] == 0).all(dim=1)
    turning = (moving[:, :2] == 0).all(dim=1)
    for mask, expected in ((forward, 0.3), (lateral, 0.3), (turning, 0.1)):
      self.assertAlmostEqual(mask.float().mean().item(), expected, delta=0.02)
    speeds = moving[lateral, 1]
    self.assertGreaterEqual(speeds.abs().min().item(), 0.15 - 1e-6)
    self.assertTrue((speeds > 0).any() and (speeds < 0).any())

  def test_flat_contact_sensor_and_wheel_axles_match_reward_selectors(self):
    cfg = bodydemo03_flat_env_cfg()
    robot = Entity(cfg.scene.entities["robot"])
    sensor_cfg = next(s for s in cfg.scene.sensors if s.name == "feet_ground_contact")
    geoms = ContactSensor(sensor_cfg)._resolve_primary_names(
      {"robot": robot}, sensor_cfg.primary
    )
    asset = cfg.rewards["wheel_lateral_slip"].params["asset_cfg"]
    asset.resolve({"robot": robot})
    _, sites = robot.find_sites(asset.site_names)
    _, bodies = robot.find_bodies(asset.body_names)
    self.assertEqual(sites, [name.removesuffix("_foot_collision") for name in geoms])
    self.assertEqual(bodies, [f"{name}_foot_link" for name in sites])
    model = robot.spec.compile()
    for leg in sites:
      axis = model.joint(f"{leg}_foot_joint").axis
      self.assertEqual(abs(axis[1]), 1.0)
      self.assertEqual(axis[0], 0.0)
      self.assertEqual(axis[2], 0.0)
      mesh = model.mesh(f"{leg}_foot_link")
      vertices = model.mesh_vert[mesh.vertadr[0] : mesh.vertadr[0] + mesh.vertnum[0]]
      rotation = np.empty(9)
      mujoco.mju_quat2Mat(rotation, model.mesh_quat[mesh.id])
      vertices = vertices @ rotation.reshape(3, 3).T + model.mesh_pos[mesh.id]
      radius = np.linalg.norm(vertices[:, [0, 2]], axis=1).max()
      self.assertAlmostEqual(radius, BODYDEMO03_WHEEL_RADIUS, delta=1e-5)

    # Check the sign convention against actual FK rather than assuming all
    # positive hip angles, or the Go2W convention, mean the same movement.
    data = mujoco.MjData(model)
    data.qpos[:7] = [0, 0, 0.34, 1, 0, 0, 0]
    mujoco.mj_forward(model, data)
    neutral_y = {leg: abs(data.site_xpos[model.site(leg).id, 1]) for leg in sites}
    for leg in sites:
      joint = model.joint(f"{leg}_hip_joint").qposadr[0]
      for factor in (1.0, -1.0):
        data.qpos[joint] = factor * 0.1 * body_mdp.HIP_INWARD_SIGNS[leg]
        mujoco.mj_forward(model, data)
        delta = abs(data.site_xpos[model.site(leg).id, 1]) - neutral_y[leg]
        self.assertLess(factor * delta, 0.0)
      data.qpos[joint] = 0.0

  def test_flat_posture_reduces_inward_hips_more_than_equal_outward_motion(self):
    env = reward_env([[0, 0.3, 0]] * 3)
    robot = env.scene["robot"]
    signs = torch.tensor([1.0, -1.0, -1.0, 1.0])
    robot.data.joint_pos[1, 0::4] = 0.15 * signs
    robot.data.joint_pos[2, 0::4] = -0.15 * signs
    cfg = bodydemo03_flat_env_cfg().rewards["pose"]
    params = dict(cfg.params)
    params["asset_cfg"] = SceneEntityCfg(
      "robot",
      joint_names=r".*_(hip|thigh|calf)_joint$",
      joint_ids=[i for i in range(16) if i % 4 != 3],
    )
    reward = cfg.func(SimpleNamespace(params=params), env)
    values = reward(env, **params)
    self.assertEqual(values[0], 1.0)
    self.assertGreater(values[2], values[1])
    self.assertLess(values[1], 0.01)

  def test_flat_gait_requires_rolling_progress_or_maneuver_steps(self):
    env = reward_env(
      [[0.5, 0, 0]] * 3
      + [[-0.3, 0, 0]]
      + [[0.5, 0, 0]] * 2
      + [[0, 0, 0], [0, 0.3, 0], [0, 0, 0.5], [0, 0, -0.5]]
    )
    data = env.scene["robot"].data
    commands = env.command_manager.get_command("twist")
    data.root_link_lin_vel_b[:, 0] = commands[:, 0]
    signs = torch.tensor([1.0, -1.0, 1.0, -1.0])
    data.joint_vel[:, 3::4] = commands[:, :1] * signs / BODYDEMO03_WHEEL_RADIUS
    data.joint_vel[1, 3::4] = 0  # Body progress from legs with stationary wheels.
    data.joint_vel[2, 3::4] = 0.5 / BODYDEMO03_WHEEL_RADIUS  # Wrong right-wheel signs.
    data.root_link_lin_vel_b[5, 0] = 0  # Wheels spin without body progress.
    contact = env.scene["feet_ground_contact"].data
    contact.found[4, 0] = 0  # Rolling must keep all four wheels in support.
    contact.current_contact_time[7:, [1, 2]] = 0
    contact.current_air_time[7:, [1, 2]] = 0.2
    env.episode_length_buf[:] = 5
    cfg = bodydemo03_flat_env_cfg().rewards["lateral_gait"]
    params = dict(cfg.params)
    params["asset_cfg"] = SceneEntityCfg(
      "robot", joint_names=r".*_foot_joint$", joint_ids=[3, 7, 11, 15]
    )
    reward = cfg.func(SimpleNamespace(params=params), env)
    values = reward(env, **params)
    self.assertGreater(values[0], 0.99)
    self.assertGreater(values[3], 0.99)
    self.assertTrue((values[[1, 2, 4, 5, 6]] < 0.01).all())
    self.assertTrue((values[7:] > 0.99).all())

  def test_flat_requires_loaded_wheels_at_rest_and_in_both_rolling_directions(self):
    env = reward_env([
      [0, 0, 0], [0.5, 0, 0], [-0.3, 0, 0],
      [0, 0.3, 0], [0, -0.3, 0], [0, 0, 0.5], [0, 0, -0.5],
      [0, 0.15, 0], [0, 0, 0.15], [0, 0.05, 0], [0, 0, 0],
    ])
    contact = env.scene["feet_ground_contact"].data
    contact.force[:, 1] = 0
    contact.found[:, 1] = 0
    # A grazing wheel must not count as weight-bearing just because it touches.
    contact.found[-1, 1] = 1
    contact.force[-1, 1, 2] = 0.1
    params = bodydemo03_flat_env_cfg().rewards["wheel_support_deficit"].params
    values = body_mdp.wheel_support_deficit(env, **params)
    torch.testing.assert_close(
      values, torch.tensor([1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0.98])
    )
    torch.testing.assert_close(
      body_mdp.wheel_support_fraction(env, "feet_ground_contact"),
      torch.full((11,), 0.75),
    )
    contact.force[:, 1, 2] = 60
    contact.found[:, 1] = 1
    self.assertTrue((body_mdp.wheel_support_deficit(env, **params) == 0).all())

  def test_flat_penalizes_constant_wheel_spin_only_under_standing_commands(self):
    env = reward_env([[0, 0, 0]] * 3 + [[0.3, 0, 0], [0, 0.3, 0], [0, 0, -0.3]])
    data = env.scene["robot"].data
    data.joint_vel[1, 7] = 15
    data.joint_vel[2, 7] = -15
    data.joint_vel[3:, 7] = 15
    # Leg motion is excluded from this wheel-specific cost.
    data.joint_vel[:, 0] = 100
    term = bodydemo03_flat_env_cfg().rewards["stationary_wheel_velocity"]
    params = dict(term.params, asset_cfg=SceneEntityCfg("robot", joint_ids=[3, 7, 11, 15]))
    values = term.func(env, **params)
    torch.testing.assert_close(values, torch.tensor([0, 56.25, 56.25, 0, 0, 0]))
    self.assertLess(term.weight * values[1], -1)

  def test_flat_allows_normal_swing_but_rejects_a_permanently_lifted_leg(self):
    env = reward_env([[0, 0, 0], [0.3, 0, 0], [0, 0.3, 0], [0, 0, -0.3], [0, -0.3, 0]])
    air = env.scene["feet_ground_contact"].data.current_air_time
    air[:, 1] = torch.tensor([0.0, 0.24, 0.35, 0.55, 0.81])
    cfg = bodydemo03_flat_env_cfg()
    cost = cfg.rewards["wheel_air_time_excess"]
    torch.testing.assert_close(cost.func(env, **cost.params), torch.tensor([0, 0, 0, 1, 4.0]))
    limit = cfg.terminations["wheel_air_time_limit"]
    torch.testing.assert_close(limit.func(env, **limit.params), torch.tensor([False] * 4 + [True]))
    air.zero_()  # Landing/reset clears the timeout; no term keeps stale per-env state.
    self.assertFalse(limit.func(env, **limit.params).any())

  def test_rough_reward_baseline_is_preserved(self):
    # This task was excluded from the Flat migration.
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
      with self.subTest(play=play):
        body = bodydemo03_rough_env_cfg(play=play)
        expected = dict(weights, pose=0.5, stand_still=-0.5)
        self.assertEqual(
          {name: term.weight for name, term in body.rewards.items()}, expected
        )
        self.assertEqual(
          set(body.metrics),
          {"mean_action_acc", "leg_joint_vel_l2", "wheel_joint_vel_l2"},
        )
        self.assertEqual(body.commands["twist"].rel_standing_envs, 0.1)
        self.assertEqual(body.commands["twist"].ranges.lin_vel_y, (0.0, 0.0))


if __name__ == "__main__":
  unittest.main()
