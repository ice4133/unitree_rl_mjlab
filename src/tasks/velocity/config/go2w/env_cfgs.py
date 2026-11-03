"""Go2W flat velocity baseline using the shared velocity environment factory.

Action/observation scales and the split leg/wheel penalties reference robot_lab:
https://github.com/fan-ziqi/robot_lab/blob/main/source/robot_lab/robot_lab/tasks/manager_based/locomotion/velocity/config/wheeled/unitree_go2w/rough_env_cfg.py
Reward functions retain this project's definitions, so weights are starting values.
"""

from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.envs import mdp
from mjlab.envs.mdp.actions import JointPositionActionCfg, JointVelocityActionCfg
from mjlab.managers import EventTermCfg, RewardTermCfg, SceneEntityCfg, TerminationTermCfg
from mjlab.sensor import ContactMatch, ContactSensorCfg
from mjlab.tasks.velocity import mdp as velocity_mdp
from mjlab.tasks.velocity.mdp import UniformVelocityCommandCfg

from src.assets.robots.unitree_go2w.go2w_constants import get_go2w_robot_cfg
from src.tasks.velocity.velocity_env_cfg import make_velocity_env_cfg

LEG_JOINTS = (r".*_(hip|thigh|calf)_joint",)
WHEEL_JOINTS = (r".*_wheel_joint",)


def unitree_go2w_flat_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
  """Create a rolling velocity task with 12 leg and 4 wheel actions."""
  cfg = make_velocity_env_cfg()
  cfg.sim.njmax = 300
  cfg.sim.mujoco.ccd_iterations = 50
  cfg.sim.contact_sensor_maxmatch = 64
  cfg.sim.nconmax = None

  cfg.scene.entities = {"robot": get_go2w_robot_cfg()}
  cfg.scene.num_envs = 1024 if not play else 1
  assert cfg.scene.terrain is not None
  cfg.scene.terrain.terrain_type = "plane"
  cfg.scene.terrain.terrain_generator = None

  # Go2W's named wheel geoms/sites define the ground-contact sensor contract.
  site_names = ("FR", "FL", "RR", "RL")
  geom_names = tuple(f"{name}_foot_collision" for name in site_names)
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
  cfg.scene.sensors = (feet_ground_cfg, nonfoot_ground_cfg)
  cfg.viewer.body_name = "base_link"
  cfg.viewer.distance = 1.5
  cfg.viewer.elevation = -10.0
  cfg.observations["critic"].terms["foot_height"].params["asset_cfg"].site_names = site_names
  cfg.events["foot_friction"].params["asset_cfg"].geom_names = geom_names
  cfg.events["base_com"].params["asset_cfg"].body_names = ("base_link",)

  cfg.rewards["pose"].params["std_standing"] = {
    r".*(FR|FL|RR|RL)_hip_joint.*": 0.05,
    r".*(FR|FL|RR|RL)_thigh_joint.*": 0.1,
    r".*(FR|FL|RR|RL)_calf_joint.*": 0.15,
  }
  cfg.rewards["pose"].params["std_walking"] = {
    r".*(FR|FL|RR|RL)_hip_joint.*": 0.15,
    r".*(FR|FL|RR|RL)_thigh_joint.*": 0.35,
    r".*(FR|FL|RR|RL)_calf_joint.*": 0.5,
  }
  cfg.rewards["pose"].params["std_running"] = {
    r".*(FR|FL|RR|RL)_hip_joint.*": 0.15,
    r".*(FR|FL|RR|RL)_thigh_joint.*": 0.35,
    r".*(FR|FL|RR|RL)_calf_joint.*": 0.5,
  }
  for name in ("body_orientation_l2", "body_ang_vel"):
    cfg.rewards[name].params["asset_cfg"].body_names = ("base_link",)
  cfg.terminations["illegal_contact"] = TerminationTermCfg(
    func=velocity_mdp.illegal_contact,
    params={"sensor_name": nonfoot_ground_cfg.name, "force_threshold": 10.0},
  )

  # Policy output order follows XML joint order within each action term:
  # FL/FR/RL/RR legs (12), then FL/FR/RL/RR wheels (4).
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

  # Keep the deployment's 55-value actor contract, omitting cumulative wheel
  # angles while retaining all 16 velocities. Normalization is exported in ONNX.
  for group in cfg.observations.values():
    group.terms.pop("height_scan", None)
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
  # This flat rolling baseline uses fixed command ranges and no terrain levels.
  cfg.curriculum = {}

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

  # Moderate startup randomization for the first flat-ground baseline.
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
    cfg.events["randomize_terrain"] = EventTermCfg(
      func=mdp.randomize_terrain,
      mode="reset",
      params={},
    )

  return cfg
