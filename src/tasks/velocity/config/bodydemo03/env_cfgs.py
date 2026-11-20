"""Bodydemo03 Flat/Rough velocity templates adapted from the Go2W tasks.

Reward weights and action scales are initial simulation values for later tuning.
"""

from copy import deepcopy

from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.envs import mdp
from mjlab.envs.mdp.actions import JointPositionActionCfg, JointVelocityActionCfg
from mjlab.managers import EventTermCfg, RewardTermCfg, SceneEntityCfg, TerminationTermCfg
from mjlab.managers.metrics_manager import MetricsTermCfg
from mjlab.sensor import ContactMatch, ContactSensorCfg, RayCastSensorCfg
from mjlab.tasks.velocity import mdp as velocity_mdp
from mjlab.tasks.velocity.mdp import UniformVelocityCommandCfg

from src.assets.robots.bodydemo03.bodydemo03_constants import (
  BODYDEMO03_LEG_JOINTS,
  BODYDEMO03_WHEEL_JOINTS,
  WHEEL_GEOMS,
  WHEEL_SITES,
  get_bodydemo03_velocity_robot_cfg,
)
from src.tasks.velocity.velocity_env_cfg import make_velocity_env_cfg

from . import rewards as bodydemo03_rewards

LEG_JOINTS = BODYDEMO03_LEG_JOINTS
WHEEL_JOINTS = BODYDEMO03_WHEEL_JOINTS


def _make_bodydemo03_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
  """Configure the common 12 leg-position and 4 wheel-velocity actions."""
  cfg = make_velocity_env_cfg()

  cfg.scene.entities = {"robot": get_bodydemo03_velocity_robot_cfg()}
  cfg.scene.num_envs = 1024 if not play else 1
  for sensor in cfg.scene.sensors or ():
    if sensor.name == "terrain_scan":
      assert isinstance(sensor, RayCastSensorCfg)
      sensor.frame.name = "base_link"
      # Terrain is group 0; ignore the robot's visual and collision geoms.
      sensor.include_geom_groups = (0,)

  # Bodydemo03's named wheel geoms/sites define the ground-contact sensor contract.
  site_names = WHEEL_SITES
  geom_names = WHEEL_GEOMS
  feet_ground_cfg = ContactSensorCfg(
    name="feet_ground_contact",
    primary=ContactMatch(mode="geom", pattern=geom_names, entity="robot"),
    secondary=ContactMatch(mode="body", pattern="terrain"),
    fields=("found", "force"),
    reduce="netforce",
    num_slots=1,
    track_air_time=True,
  )
  nonfoot_ground_cfg = ContactSensorCfg(
    name="nonfoot_ground_touch",
    primary=ContactMatch(
      mode="geom",
      entity="robot",
      pattern=r".*_collision\d*$",
      exclude=geom_names,
    ),
    secondary=ContactMatch(mode="body", pattern="terrain"),
    fields=("found", "force"),
    reduce="none",
    num_slots=1,
    history_length=4,
  )
  cfg.scene.sensors = (cfg.scene.sensors or ()) + (
    feet_ground_cfg,
    nonfoot_ground_cfg,
  )
  cfg.viewer.body_name = "base_link"
  cfg.viewer.distance = 1.5
  cfg.viewer.elevation = -10.0
  cfg.observations["critic"].terms["foot_height"].params["asset_cfg"].site_names = site_names
  cfg.events["foot_friction"].params["asset_cfg"].geom_names = geom_names
  cfg.events["base_com"].params["asset_cfg"].body_names = ("base_link",)

  cfg.rewards["pose"].params["std_standing"] = {
    r".*(fl|fr|rl|rr)_hip_joint.*": 0.05,
    r".*(fl|fr|rl|rr)_thigh_joint.*": 0.1,
    r".*(fl|fr|rl|rr)_calf_joint.*": 0.15,
  }
  cfg.rewards["pose"].params["std_walking"] = {
    r".*(fl|fr|rl|rr)_hip_joint.*": 0.15,
    r".*(fl|fr|rl|rr)_thigh_joint.*": 0.35,
    r".*(fl|fr|rl|rr)_calf_joint.*": 0.5,
  }
  cfg.rewards["pose"].params["std_running"] = {
    r".*(fl|fr|rl|rr)_hip_joint.*": 0.15,
    r".*(fl|fr|rl|rr)_thigh_joint.*": 0.35,
    r".*(fl|fr|rl|rr)_calf_joint.*": 0.5,
  }
  for name in ("body_orientation_l2", "body_ang_vel"):
    cfg.rewards[name].params["asset_cfg"].body_names = ("base_link",)
  cfg.terminations["illegal_contact"] = TerminationTermCfg(
    func=velocity_mdp.illegal_contact,
    params={"sensor_name": nonfoot_ground_cfg.name, "force_threshold": 10.0},
  )

  # Policy output order follows XML joint order within each action term:
  # fl/fr/rl/rr legs (12), then fl/fr/rl/rr wheels (4).
  cfg.actions = {
    "joint_pos": JointPositionActionCfg(
      entity_name="robot",
      actuator_names=LEG_JOINTS,
      scale={r".*_hip_joint": 0.125, r".*_(thigh|calf)_joint": 0.25},
      use_default_offset=True,
    ),
    "joint_vel": JointVelocityActionCfg(
      entity_name="robot",
      actuator_names=WHEEL_JOINTS,
      scale=5.0,
      use_default_offset=True,
    ),
  }

  # Omit cumulative wheel angles while retaining all 16 velocities.
  # Normalization is exported in ONNX.
  for group in cfg.observations.values():
    group.terms["base_ang_vel"].scale = 0.25
    group.terms["joint_pos"].params["asset_cfg"] = SceneEntityCfg(
      "robot", joint_names=LEG_JOINTS
    )
    group.terms["joint_vel"].scale = 0.05

  command = cfg.commands["twist"]
  assert isinstance(command, UniformVelocityCommandCfg)
  command.heading_command = False
  command.ranges.heading = None
  command.rel_standing_envs = 0.1
  command.ranges.lin_vel_x = (-0.5, 1.0)
  command.ranges.lin_vel_y = (0.0, 0.0)
  command.ranges.ang_vel_z = (-1.0, 1.0)
  # Keep command ranges compatible with the non-steering wheels. Rough uses
  # terrain progression, without the shared template's lateral-speed curriculum.
  cfg.curriculum.pop("command_vel", None)

  # These foot rewards assume stationary stance feet and/or a walking gait.
  for name in ("foot_gait", "foot_clearance", "foot_slip", "soft_landing"):
    cfg.rewards.pop(name, None)
  # Wheel rotation naturally contributes angular momentum.
  cfg.rewards.pop("angular_momentum", None)
  cfg.rewards["track_linear_velocity"].weight = 3.0
  cfg.rewards["track_angular_velocity"].weight = 1.5
  cfg.rewards["action_rate_l2"].weight = -0.01

  # Continuous wheels have no position target or position limit to penalize.
  for name in ("pose", "stand_still", "joint_pos_limits", "joint_acc_l2"):
    cfg.rewards[name].params["asset_cfg"] = SceneEntityCfg(
      "robot", joint_names=LEG_JOINTS
    )
  cfg.rewards["stand_still"].weight = -2.0
  cfg.rewards["joint_pos_limits"].weight = -5.0
  cfg.rewards["joint_acc_l2"].weight = -2.5e-7
  cfg.rewards["wheel_acc_l2"] = RewardTermCfg(
    func=mdp.joint_acc_l2,
    weight=-2.5e-9,
    params={"asset_cfg": SceneEntityCfg("robot", joint_names=WHEEL_JOINTS)},
  )
  # This mjlab function selects actuator_ids, rather than joint_ids.
  cfg.rewards["leg_torques_l2"] = RewardTermCfg(
    func=mdp.joint_torques_l2,
    weight=-2.5e-5,
    params={"asset_cfg": SceneEntityCfg("robot", actuator_names=LEG_JOINTS)},
  )

  # Moderate randomization shared by the flat and rough baselines.
  cfg.events["foot_friction"].params["ranges"] = (0.5, 1.2)
  cfg.events["base_com"].params["ranges"] = {
    0: (-0.02, 0.02), 1: (-0.02, 0.02), 2: (-0.02, 0.02)
  }
  if "push_robot" in cfg.events:
    cfg.events["push_robot"].interval_range_s = (8.0, 12.0)
    cfg.events["push_robot"].params["velocity_range"] = {
      "x": (-0.3, 0.3), "y": (-0.2, 0.2), "yaw": (-0.4, 0.4)
    }

  if play:
    cfg.episode_length_s = int(1e9)
    cfg.observations["actor"].enable_corruption = False
    cfg.events.pop("push_robot", None)
    cfg.curriculum = {}
    cfg.events["randomize_terrain"] = EventTermCfg(
      func=mdp.randomize_terrain,
      mode="reset",
      params={},
    )

  return cfg


def bodydemo03_rough_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
  """Train terrain-aware rolling and stepping without prescribing a foot gait."""
  cfg = _make_bodydemo03_env_cfg(play=play)
  cfg.sim.mujoco.ccd_iterations = 500
  cfg.sim.contact_sensor_maxmatch = 500
  # Let MJWarp size contact storage for the imported mesh/terrain contacts.
  cfg.sim.nconmax = None

  assert cfg.scene.terrain is not None
  assert cfg.scene.terrain.terrain_generator is not None
  # The shared factory shallow-copies the preset. Own its nested sub-terrains
  # so generation and play overrides cannot change other registered tasks.
  generator = deepcopy(cfg.scene.terrain.terrain_generator)
  cfg.scene.terrain.terrain_generator = generator
  generator.curriculum = not play
  cfg.scene.terrain.max_init_terrain_level = 1

  # Climbing requires vertical base motion and larger leg excursions. Keep the
  # shared reward's original defaults for Flat and the other robots.
  cfg.rewards["track_linear_velocity"].params["z_vel_weight"] = 0.0
  cfg.rewards["pose"].weight = 0.5
  for name in ("std_walking", "std_running"):
    cfg.rewards["pose"].params[name] = {
      r".*(fl|fr|rl|rr)_hip_joint.*": 0.2,
      r".*(fl|fr|rl|rr)_thigh_joint.*": 0.5,
      r".*(fl|fr|rl|rr)_calf_joint.*": 0.75,
    }
  cfg.rewards["stand_still"].weight = -0.5

  # Monitor leg and wheel motion separately alongside velocity error, terrain
  # level and terminations. These diagnostics do not reward unnecessary steps.
  for name, joints in (("leg", LEG_JOINTS), ("wheel", WHEEL_JOINTS)):
    cfg.metrics[f"{name}_joint_vel_l2"] = MetricsTermCfg(
      func=mdp.joint_vel_l2,
      params={"asset_cfg": SceneEntityCfg("robot", joint_names=joints)},
    )

  if play:
    generator.num_cols = 5
    generator.num_rows = 5
    generator.border_width = 10.0

  return cfg


def bodydemo03_flat_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
  """Create the flat rolling template with its 55-value actor input."""
  cfg = _make_bodydemo03_env_cfg(play=play)
  cfg.sim.njmax = 300
  cfg.sim.mujoco.ccd_iterations = 50
  cfg.sim.contact_sensor_maxmatch = 64
  cfg.sim.nconmax = None

  assert cfg.scene.terrain is not None
  cfg.scene.terrain.terrain_type = "plane"
  cfg.scene.terrain.terrain_generator = None
  cfg.scene.sensors = tuple(
    sensor for sensor in (cfg.scene.sensors or ()) if sensor.name != "terrain_scan"
  )
  for group in cfg.observations.values():
    group.terms.pop("height_scan", None)
  cfg.curriculum = {}

  # Match the Go2W baseline; keep the optional standing terms disabled.
  cfg.commands["twist"].rel_standing_envs = 0.1
  cfg.rewards["stand_wheel_motion"] = RewardTermCfg(
    func=bodydemo03_rewards.standing_wheel_motion_l2,
    weight=0.0,
    params={
      "asset_cfg": SceneEntityCfg("robot", joint_names=WHEEL_JOINTS),
      "command_name": "twist",
      "command_threshold": 0.1,
      "target_weight": 0.25,
    },
  )
  cfg.rewards["stand_wheel_support"] = RewardTermCfg(
    func=bodydemo03_rewards.standing_support_deficit,
    weight=0.0,
    params={
      "sensor_name": "feet_ground_contact",
      "command_name": "twist",
      "command_threshold": 0.1,
      "min_normal_force": 5.0,
    },
  )
  cfg.rewards["stand_base_motion"] = RewardTermCfg(
    func=bodydemo03_rewards.standing_base_motion_l2,
    weight=0.0,
    params={
      "asset_cfg": SceneEntityCfg("robot"),
      "command_name": "twist",
      "command_threshold": 0.1,
      "linear_scale": 0.05,
      "angular_scale": 0.1,
    },
  )
  cfg.rewards["stand_wheel_height"] = RewardTermCfg(
    func=bodydemo03_rewards.standing_wheel_height_spread,
    weight=0.0,
    params={
      "asset_cfg": SceneEntityCfg("robot", site_names=WHEEL_SITES),
      "command_name": "twist",
      "command_threshold": 0.1,
      "height_scale": 0.02,
      "height_tolerance": 0.001,
    },
  )
  for name, func in (
    ("wheel_support_fraction", bodydemo03_rewards.wheel_support_fraction),
    ("all_wheels_supported", bodydemo03_rewards.all_wheels_supported),
  ):
    cfg.metrics[name] = MetricsTermCfg(
      func=func,
      params={"sensor_name": "feet_ground_contact", "min_normal_force": 5.0},
    )
  cfg.metrics["standing_wheel_velocity_l2"] = MetricsTermCfg(
    func=bodydemo03_rewards.standing_wheel_motion_l2,
    params={
      "asset_cfg": SceneEntityCfg("robot", joint_names=WHEEL_JOINTS),
      "command_name": "twist",
      "command_threshold": 0.1,
      "target_weight": 0.0,
    },
  )

  return cfg
