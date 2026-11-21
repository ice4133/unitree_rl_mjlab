"""Bodydemo03 model loading, robot parameters and mjlab configurations.

The base loader preserves the imported CAD model. The velocity adapter adds
actuators, named contact geoms, axle sites, an IMU and finite leg limits.
All motor/limit values below are provisional simulation defaults.
"""

import re
from copy import deepcopy
from pathlib import Path

import mujoco
from mjlab.actuator import BuiltinPositionActuatorCfg, IdealPdActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.spec_config import CollisionCfg

BODYDEMO03_DIR = Path(__file__).resolve().parent
BODYDEMO03_XML = BODYDEMO03_DIR / "xmls" / "bodydemo03.xml"

BODYDEMO03_LEG_JOINTS = (r"^(fl|fr|rl|rr)_(hip|thigh|calf)_joint$",)
BODYDEMO03_WHEEL_JOINTS = (r"^(fl|fr|rl|rr)_foot_joint$",)
# Radial extent in the wheel's local XZ plane, measured from the imported mesh.
BODYDEMO03_WHEEL_RADIUS = 0.088

WHEEL_SITES = ("fl", "fr", "rl", "rr")
WHEEL_GEOMS = tuple(f"{leg}_foot_collision" for leg in WHEEL_SITES)
_WHEEL_REGEX = r"^(fl|fr|rl|rr)_foot_collision$"

# TODO: These symmetric ranges are simulation placeholders about the CAD zero,
# not measured mechanical limits. Wheel joints deliberately remain continuous.
LEG_JOINT_LIMITS = {
  r".*_hip_joint": (-0.5, 0.5),
  r".*_thigh_joint": (-1.0, 1.0),
  r".*_calf_joint": (-1.5, 1.5),
}

# Match the current standalone scene's generic servos as a starting point.
# TODO: Tune stiffness, damping and effort limits for this robot in simulation.
LEG_ACTUATOR = BuiltinPositionActuatorCfg(
  target_names_expr=BODYDEMO03_LEG_JOINTS,
  stiffness=400.0,
  damping=12.0,
  effort_limit=50.0,
  armature=0.01,
)
WHEEL_ACTUATOR = IdealPdActuatorCfg(
  target_names_expr=BODYDEMO03_WHEEL_JOINTS,
  stiffness=0.0,
  damping=2.0,
  effort_limit=8.0,
  armature=0.01,
)

# The CAD zero is already a standing pose. Keep raw joint axes: right wheel
# positive rotation has the opposite rolling direction to left wheel rotation.
INIT_STATE = EntityCfg.InitialStateCfg(
  pos=(0.0, 0.0, 0.34),
  joint_pos={".*": 0.0},
  joint_vel={".*": 0.0},
)

# Robot-terrain contacts remain enabled; robot self collisions are disabled.
# Wheels use sliding friction without torsional or rolling constraints.
FULL_COLLISION = CollisionCfg(
  geom_names_expr=(r".*_collision",),
  condim={_WHEEL_REGEX: 3, r".*_collision": 1},
  priority={_WHEEL_REGEX: 1},
  friction={_WHEEL_REGEX: (0.8,)},
  solimp={_WHEEL_REGEX: (0.9, 0.95, 0.023)},
  contype=1,
  conaffinity=0,
)


def get_bodydemo03_spec() -> mujoco.MjSpec:
  """Load an independent model with all referenced meshes in its asset bundle."""
  spec = mujoco.MjSpec.from_file(str(BODYDEMO03_XML))
  spec.assets = {
    mesh.file: (BODYDEMO03_XML.parent / mesh.file).read_bytes()
    for mesh in spec.meshes
  }
  return spec


def get_bodydemo03_robot_cfg() -> EntityCfg:
  """Return the imported model configuration, ready for task-specific settings."""
  return EntityCfg(
    init_state=deepcopy(INIT_STATE),
    spec_fn=get_bodydemo03_spec,
  )


def get_bodydemo03_velocity_spec() -> mujoco.MjSpec:
  """Adapt an independent copy of the imported model for velocity training."""
  spec = get_bodydemo03_spec()
  for body in spec.bodies:
    for geom in body.geoms:
      # The URDF import supplies one visual and one collision mesh per link.
      is_visual = geom.contype == 0 and geom.conaffinity == 0
      suffix = "visual" if is_visual else "collision"
      geom.name = f"{body.name.removesuffix('_link')}_{suffix}"
      geom.group = 2 if is_visual else 3

  for joint in spec.joints:
    for pattern, limits in LEG_JOINT_LIMITS.items():
      if re.fullmatch(pattern, joint.name):
        joint.limited = True
        joint.range[:] = limits
    if re.fullmatch(BODYDEMO03_WHEEL_JOINTS[0], joint.name):
      joint.limited = False

  base = spec.body("base_link")
  base.add_site(name="imu", size=(0.01, 0.0, 0.0), group=5)
  for name, sensor_type in (
    ("imu_ang_vel", mujoco.mjtSensor.mjSENS_GYRO),
    ("imu_lin_vel", mujoco.mjtSensor.mjSENS_VELOCIMETER),
    ("imu_lin_acc", mujoco.mjtSensor.mjSENS_ACCELEROMETER),
  ):
    spec.add_sensor(
      name=name,
      type=sensor_type,
      objtype=mujoco.mjtObj.mjOBJ_SITE,
      objname="imu",
    )
  for leg in WHEEL_SITES:
    # Axle sites stay fixed under wheel rotation and supply critic heights.
    spec.body(f"{leg}_foot_link").add_site(
      name=leg, size=(0.01, 0.0, 0.0), group=5
    )
  return spec


def get_bodydemo03_velocity_robot_cfg() -> EntityCfg:
  """Return the actuated training template, independent of the preview config."""
  return EntityCfg(
    init_state=deepcopy(INIT_STATE),
    spec_fn=get_bodydemo03_velocity_spec,
    collisions=(deepcopy(FULL_COLLISION),),
    articulation=EntityArticulationInfoCfg(
      actuators=(deepcopy(LEG_ACTUATOR), deepcopy(WHEEL_ACTUATOR)),
      soft_joint_pos_limit_factor=0.9,
    ),
  )
