"""CPU checks for unified rolling/sidestepping commands and reward semantics."""

from types import SimpleNamespace

import torch
from mjlab.managers import SceneEntityCfg
from mjlab.sensor import ContactSensor

from src.tasks.velocity.config.go2w import mdp
from src.tasks.velocity.config.go2w.commands import Go2WVelocityCommandCfg
from src.tasks.velocity.config.go2w.env_cfgs import (
  unitree_go2w_flat_env_cfg,
  unitree_go2w_rough_env_cfg,
)
from src.tasks.velocity.mdp.curriculums import commands_vel


def make_command(num_envs=20000, **kwargs):
  cfg = Go2WVelocityCommandCfg(
    entity_name="robot",
    resampling_time_range=(3.0, 8.0),
    ranges=Go2WVelocityCommandCfg.Ranges(
      lin_vel_x=(-0.5, 1.0),
      lin_vel_y=(-0.25, 0.25),
      ang_vel_z=(-1.0, 1.0),
    ),
    rel_standing_envs=0.1,
    **kwargs,
  )
  env = SimpleNamespace(num_envs=num_envs, device="cpu", scene={"robot": object()})
  return cfg.build(env)


def test_command_sampling_covers_each_skill_and_both_lateral_directions():
  torch.manual_seed(7)
  term = make_command()
  term._resample_command(torch.arange(term.num_envs))
  term._update_command()
  commands = term.command
  standing = (commands == 0).all(dim=1)
  assert abs(standing.float().mean().item() - 0.1) < 0.02
  moving = commands[~standing]
  forward = (moving[:, 1:] == 0).all(dim=1)
  lateral = (moving[:, 0] == 0) & (moving[:, 2] == 0)
  turning = (moving[:, :2] == 0).all(dim=1)
  mixed = ~(forward | lateral | turning)
  for mask, expected in ((forward, 0.3), (lateral, 0.3), (turning, 0.1), (mixed, 0.3)):
    assert abs(mask.float().mean().item() - expected) < 0.02
  lateral_speeds = moving[lateral, 1]
  assert lateral_speeds.abs().min() >= 0.15
  assert lateral_speeds.abs().max() <= 0.25
  assert (lateral_speeds > 0).any() and (lateral_speeds < 0).any()


def test_partial_resampling_does_not_touch_other_environments():
  term = make_command(num_envs=8)
  term.vel_command_b[:] = 9.0
  term._resample_command(torch.tensor([1, 4, 7]))
  assert (term.command[[0, 2, 3, 5, 6]] == 9.0).all()
  term._resample_command(torch.empty(0, dtype=torch.long))


def test_disabled_lateral_range_stays_disabled():
  term = make_command(num_envs=32)
  term.cfg.ranges.lin_vel_y = (0.0, 0.0)
  term._resample_command(torch.arange(32))
  assert (term.command[:, 1] == 0.0).all()


def reward_env(commands, contact=None):
  count = len(commands)
  commands = torch.tensor(commands, dtype=torch.float32)
  data = SimpleNamespace(
    root_link_lin_vel_b=torch.zeros(count, 3),
    body_link_quat_w=torch.tensor([1.0, 0.0, 0.0, 0.0]).repeat(count, 4, 1),
    site_lin_vel_w=torch.zeros(count, 4, 3),
  )
  contact_data = SimpleNamespace(
    found=torch.ones(count, 4) if contact is None else torch.tensor(contact),
    current_contact_time=torch.ones(count, 4) * 0.2,
    current_air_time=torch.zeros(count, 4),
  )
  return SimpleNamespace(
    command_manager=SimpleNamespace(get_command=lambda _: commands),
    scene={
      "robot": SimpleNamespace(data=data),
      "feet_ground_contact": SimpleNamespace(data=contact_data),
    },
    episode_length_buf=torch.zeros(count),
    step_dt=0.02,
    device="cpu",
  )


def test_gait_guidance_is_off_for_standing_rolling_and_turning():
  env = reward_env([[0, 0, 0], [0.5, 0, 0], [0, 0, 0.5], [0, 0.3, 0]])
  env.episode_length_buf[:] = 5  # Outside the diagonal gait's double-support window.
  contact = env.scene["feet_ground_contact"].data
  contact.current_contact_time[:, [1, 2]] = 0.0
  result = mdp.lateral_gait(
    env,
    command_name="twist",
    sensor_name="feet_ground_contact",
    period=0.6,
    offset=[0.0, 0.5, 0.5, 0.0],
    threshold=0.6,
  )
  assert torch.equal(result[:3], torch.zeros(3))
  assert result[3] == 1.0
  contact.current_contact_time[:] = 0.2
  all_grounded = mdp.lateral_gait(
    env,
    command_name="twist",
    sensor_name="feet_ground_contact",
    period=0.6,
    offset=[0.0, 0.5, 0.5, 0.0],
    threshold=0.6,
  )
  assert all_grounded[3] < result[3]


def test_air_time_requires_alternating_support_and_lateral_command():
  env = reward_env([[0.5, 0, 0], [0, 0.3, 0], [0, -0.3, 0]])
  contact = env.scene["feet_ground_contact"].data
  contact.current_contact_time[:2, [1, 2]] = 0.0
  contact.current_air_time[:2, [1, 2]] = 0.2
  result = mdp.lateral_air_time(
    env,
    command_name="twist",
    sensor_name="feet_ground_contact",
    threshold=0.2,
  )
  assert result[0] == 0.0
  assert result[1] > 0.0
  assert result[2] == 0.0


def test_lateral_tracking_rewards_correct_direction_and_gates_rolling():
  env = reward_env([[0, 0.3, 0], [0, -0.3, 0], [0.5, 0, 0]])
  env.scene["robot"].data.root_link_lin_vel_b[:, 1] = 0.3
  result = mdp.track_lateral_velocity(env, command_name="twist", std=0.25)
  assert result[0] == 1.0
  assert result[1] < 0.01
  assert result[2] == 0.0


def test_wheel_slip_allows_rolling_and_ignores_airborne_wheels():
  env = reward_env([[0, 0, 0]] * 3)
  velocities = env.scene["robot"].data.site_lin_vel_w
  velocities[0, :, 0] = 2.0  # Forward rolling, perpendicular to axle.
  velocities[1:, :, 1] = 2.0  # Axle-direction sliding.
  env.scene["feet_ground_contact"].data.found[2] = 0
  asset = SceneEntityCfg("robot", body_ids=[0, 1, 2, 3], site_ids=[0, 1, 2, 3])
  result = mdp.wheel_lateral_slip(env, "feet_ground_contact", asset)
  torch.testing.assert_close(result, torch.tensor([0.0, 16.0, 0.0]))
  # A 90-degree yaw rotates the axle to world X. The projection must rotate too.
  env.scene["robot"].data.body_link_quat_w[:] = torch.tensor(
    [2**-0.5, 0.0, 0.0, 2**-0.5]
  )
  result = mdp.wheel_lateral_slip(env, "feet_ground_contact", asset)
  torch.testing.assert_close(
    result, torch.tensor([16.0, 0.0, 0.0]), atol=1e-5, rtol=1e-5
  )


def test_training_and_play_curricula_and_observations():
  for factory, rough in (
    (unitree_go2w_flat_env_cfg, False),
    (unitree_go2w_rough_env_cfg, True),
  ):
    for play in (False, True):
      cfg = factory(play=play)
      command = cfg.commands["twist"]
      assert isinstance(command, Go2WVelocityCommandCfg)
      assert command.ranges.lin_vel_y == ((-0.5, 0.5) if play else (-0.25, 0.25))
      assert ("height_scan" in cfg.observations["actor"].terms) == rough
      assert len(cfg.actions) == 2
      assert "angular_momentum" not in cfg.rewards
      assert "foot_slip" not in cfg.rewards
      assert "foot_clearance" not in cfg.rewards
      assert cfg.rewards["lateral_gait"].params["offset"] == [0.0, 0.5, 0.5, 0.0]
      if play:
        assert cfg.curriculum == {}
      else:
        assert "command_vel" in cfg.curriculum
        assert ("terrain_levels" in cfg.curriculum) == rough
        env = SimpleNamespace(
          common_step_counter=3000 * 24 + 1,
          command_manager=SimpleNamespace(
            get_term=lambda _: SimpleNamespace(cfg=command)
          ),
        )
        commands_vel(env, torch.tensor([0]), **cfg.curriculum["command_vel"].params)
        assert command.ranges.lin_vel_y == (-0.5, 0.5)


def test_wheel_sensor_sites_and_bodies_resolve_in_same_order():
  # ContactSensor intentionally resolves in entity order, not tuple order.
  from mjlab.entity import Entity

  cfg = unitree_go2w_flat_env_cfg()
  robot = Entity(cfg.scene.entities["robot"])
  sensor_cfg = next(s for s in cfg.scene.sensors if s.name == "feet_ground_contact")
  geom_names = ContactSensor(sensor_cfg)._resolve_primary_names(
    {"robot": robot}, sensor_cfg.primary
  )
  asset = cfg.rewards["wheel_lateral_slip"].params["asset_cfg"]
  asset.resolve({"robot": robot})
  _, sites = robot.find_sites(asset.site_names)
  _, bodies = robot.find_bodies(asset.body_names)
  assert sites == [name.removesuffix("_foot_collision") for name in geom_names]
  assert bodies == [f"{name}_wheel_link" for name in sites]
