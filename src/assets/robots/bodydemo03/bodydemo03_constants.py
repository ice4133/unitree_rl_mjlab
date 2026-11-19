"""Bodydemo03 model loading and baseline mjlab entity configuration.

The robot has a floating base. Task-specific actuators can be added when
defining the training task.
"""

from pathlib import Path

import mujoco
from mjlab.entity import EntityCfg


BODYDEMO03_DIR = Path(__file__).resolve().parent
BODYDEMO03_XML = BODYDEMO03_DIR / "xmls" / "bodydemo03.xml"

BODYDEMO03_LEG_JOINTS = (r"^(fl|fr|rl|rr)_(hip|thigh|calf)_joint$",)
BODYDEMO03_WHEEL_JOINTS = (r"^(fl|fr|rl|rr)_foot_joint$",)


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
    init_state=EntityCfg.InitialStateCfg(pos=(0.0, 0.0, 0.34)),
    spec_fn=get_bodydemo03_spec,
  )
