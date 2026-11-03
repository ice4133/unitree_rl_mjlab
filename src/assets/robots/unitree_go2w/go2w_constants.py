"""Unitree Go2W model, actuator and initial-state configuration.

Leg joints use position targets; continuous wheel joints use velocity targets.
The task must assign separate position and velocity action terms to them.
"""

from copy import deepcopy
from pathlib import Path

import mujoco

from src import SRC_PATH
from mjlab.actuator import BuiltinPositionActuatorCfg, IdealPdActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.os import update_assets
from mjlab.utils.spec_config import CollisionCfg

##
# MJCF and assets.
##

GO2W_XML: Path = (
  SRC_PATH / "assets" / "robots" / "unitree_go2w" / "xmls" / "go2w.xml"
)
assert GO2W_XML.exists()


def get_assets(meshdir: str) -> dict[str, bytes]:
  assets: dict[str, bytes] = {}
  update_assets(assets, GO2W_XML.parent / "assets", meshdir)
  return assets


def get_spec() -> mujoco.MjSpec:
  spec = mujoco.MjSpec.from_file(str(GO2W_XML))
  spec.assets = get_assets(spec.meshdir)
  return spec


##
# Actuator config.
##

# Leg gains and limits follow Go2 as a starting point for training.
GO2W_ACTUATOR_HIP = BuiltinPositionActuatorCfg(
  target_names_expr=(".*_hip_joint",),
  stiffness=20.0,
  damping=1.0,
  effort_limit=23.5,
  armature=0.01,
)
GO2W_ACTUATOR_THIGH = BuiltinPositionActuatorCfg(
  target_names_expr=(".*_thigh_joint",),
  stiffness=20.0,
  damping=1.0,
  effort_limit=23.5,
  armature=0.01,
)
GO2W_ACTUATOR_CALF = BuiltinPositionActuatorCfg(
  target_names_expr=(".*_calf_joint",),
  stiffness=40.0,
  damping=2.0,
  effort_limit=45.0,
  armature=0.02,
)

# Zero stiffness gives torque = damping * (target_velocity - velocity).
# IdealPdActuatorCfg supports continuous wheels without inheriting a position
# range, as BuiltinVelocityActuatorCfg does in the installed mjlab version.
# The 15 Nm limit follows go2w_source.xml; damping is a tunable starting value.
GO2W_ACTUATOR_WHEEL = IdealPdActuatorCfg(
  target_names_expr=(".*_wheel_joint",),
  stiffness=0.0,
  damping=2.0,
  effort_limit=15.0,
  armature=0.01,
)

##
# Initial state.
##

INIT_STATE = EntityCfg.InitialStateCfg(
  # The larger wheel radius requires more clearance than Go2's spherical feet.
  pos=(0.0, 0.0, 0.4),
  joint_pos={
    ".*_thigh_joint": 0.9,
    ".*_calf_joint": -1.8,
    ".*R_hip_joint": 0.1,
    ".*L_hip_joint": -0.1,
    ".*_wheel_joint": 0.0,
  },
  joint_vel={".*": 0.0},
)

##
# Collision config.
##

# Wheel contact geoms keep the Go2 task's *_foot_collision naming convention.
_foot_regex = "^[FR][LR]_foot_collision$"

FEET_ONLY_COLLISION = CollisionCfg(
  geom_names_expr=(_foot_regex,),
  contype=0,
  conaffinity=1,
  condim=3,
  priority=1,
  friction=(0.8,),
  solimp=(0.9, 0.95, 0.023),
)

# Enable robot-terrain contacts and disable robot self collisions. Use sliding
# friction on wheels without adding torsional or rolling friction constraints.
FULL_COLLISION = CollisionCfg(
  geom_names_expr=(".*_collision",),
  condim={_foot_regex: 3, ".*_collision": 1},
  priority={_foot_regex: 1},
  friction={_foot_regex: (0.8,)},
  solimp={_foot_regex: (0.9, 0.95, 0.023)},
  contype=1,
  conaffinity=0,
)

##
# Final config.
##

GO2W_ARTICULATION = EntityArticulationInfoCfg(
  actuators=(
    GO2W_ACTUATOR_HIP,
    GO2W_ACTUATOR_THIGH,
    GO2W_ACTUATOR_CALF,
    GO2W_ACTUATOR_WHEEL,
  ),
  soft_joint_pos_limit_factor=0.9,
)


def get_go2w_robot_cfg() -> EntityCfg:
  """Return an independent Go2W configuration for each environment factory."""
  return EntityCfg(
    init_state=deepcopy(INIT_STATE),
    collisions=(deepcopy(FULL_COLLISION),),
    spec_fn=get_spec,
    articulation=deepcopy(GO2W_ARTICULATION),
  )


if __name__ == "__main__":
  import mujoco.viewer as viewer

  from mjlab.entity import Entity

  robot = Entity(get_go2w_robot_cfg())
  viewer.launch(robot.spec.compile())
