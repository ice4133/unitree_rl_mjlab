"""Independent Go2W export adapter using only shared mjlab/RL utilities."""

from pathlib import Path

import torch
import yaml
from mjlab.rl import MjlabOnPolicyRunner
from mjlab.rl.exporter_utils import attach_metadata_to_onnx, get_base_metadata

from src import SRC_PATH


def export_flat_deploy_cfg(metadata: dict, output: Path) -> None:
  """Pair a new flat policy with its actual defaults, actions and PD gains.

  Keep the installed ONNX and its parameters intact until the new policy has
  been evaluated. Observation layout and SDK mapping come from the flat
  deployment template; model-dependent values come from the exported policy.
  """
  template = (
    SRC_PATH.parent / "deploy/robots/go2w/config/policy/velocity/v0/params/deploy.yaml"
  )
  cfg = yaml.safe_load(template.read_text())
  if metadata["action_terms"] != ["joint_pos", "joint_vel"]:
    raise ValueError(
      "Flat deployment requires leg positions followed by wheel velocities"
    )
  joint_names = metadata["joint_names"]
  action_index = 0
  for term in cfg["actions"].values():
    ids = term["joint_ids"]
    end = action_index + len(ids)
    if metadata["action_joint_names"][action_index:end] != [
      joint_names[i] for i in ids
    ]:
      raise ValueError("Policy joint order does not match the Go2W deployment template")
    term["scale"] = metadata["action_scale"][action_index:end]
    term["offset"] = metadata["action_offset"][action_index:end]
    action_index = end
  cfg["default_joint_pos"] = metadata["default_joint_pos"]
  cfg["step_dt"] = metadata["step_dt"]
  for key, source in (("stiffness", "joint_stiffness"), ("damping", "joint_damping")):
    for joint, motor in enumerate(cfg["joint_ids_map"]):
      cfg[key][motor] = metadata[source][joint]
  output.write_text(yaml.safe_dump(cfg, sort_keys=False))


def get_go2w_metadata(env, run_name: str) -> dict:
  """Describe all 16 actions and PD gains in their explicitly named orders."""
  metadata = get_base_metadata(env, run_name)
  robot = env.scene["robot"]
  action_names, action_scales, action_offsets = [], [], []
  for name in env.action_manager.active_terms:
    term = env.action_manager.get_term(name)
    action_names.extend(term.target_names)
    for value, result in ((term.scale, action_scales), (term.offset, action_offsets)):
      if isinstance(value, torch.Tensor):
        result.extend(value[0].detach().cpu().tolist())
      else:
        result.extend([float(value)] * term.action_dim)
  metadata["action_joint_names"] = action_names
  metadata["action_scale"] = action_scales
  metadata["action_offset"] = action_offsets
  metadata["action_terms"] = list(env.action_manager.active_terms)
  metadata["step_dt"] = env.step_dt

  # IdealPd wheel actuators appear as torque motors in mjModel; their motor
  # gain=1 is not the PD stiffness. Read the actual actuator configuration.
  gains = {
    name: (act.cfg.stiffness, act.cfg.damping)
    for act in robot.actuators
    for name in act.target_names
  }
  metadata["joint_stiffness"] = [gains[name][0] for name in robot.joint_names]
  metadata["joint_damping"] = [gains[name][1] for name in robot.joint_names]
  return metadata


class Go2WVelocityOnPolicyRunner(MjlabOnPolicyRunner):
  """Reuse PPO and policy export, supplying metadata for both action terms."""

  def save(self, path: str, infos=None):
    super().save(path, infos)
    policy_dir = Path(path).parent
    self.export_policy_to_onnx(str(policy_dir), "policy.onnx")
    run_name = "local"
    logger_type = self.cfg.get("logger", "tensorboard").lower()
    if logger_type == "wandb":
      import wandb

      if wandb.run is not None:
        run_name = wandb.run.name
    metadata = get_go2w_metadata(self.env.unwrapped, run_name)
    onnx_path = policy_dir / "policy.onnx"
    attach_metadata_to_onnx(str(onnx_path), metadata)
    if (
      "height_scan" not in self.env.unwrapped.observation_manager.active_terms["actor"]
    ):
      export_flat_deploy_cfg(metadata, policy_dir / "deploy.yaml")
    if logger_type == "wandb":
      wandb.save(str(onnx_path), base_path=str(policy_dir))
