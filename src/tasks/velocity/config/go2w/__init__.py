"""Register the Go2W flat and rough velocity tasks."""

from dataclasses import replace

from mjlab.tasks.registry import register_mjlab_task

from .env_cfgs import unitree_go2w_flat_env_cfg, unitree_go2w_rough_env_cfg
from .rl_cfg import unitree_go2w_ppo_runner_cfg
from .runner import Go2WVelocityOnPolicyRunner

register_mjlab_task(
  task_id="Unitree-Go2W-Flat",
  env_cfg=unitree_go2w_flat_env_cfg(),
  play_env_cfg=unitree_go2w_flat_env_cfg(play=True),
  rl_cfg=unitree_go2w_ppo_runner_cfg(),
  runner_cls=Go2WVelocityOnPolicyRunner,
)

register_mjlab_task(
  task_id="Unitree-Go2W-Rough",
  env_cfg=unitree_go2w_rough_env_cfg(),
  play_env_cfg=unitree_go2w_rough_env_cfg(play=True),
  rl_cfg=replace(
    unitree_go2w_ppo_runner_cfg(), experiment_name="go2w_velocity_rough"
  ),
  runner_cls=Go2WVelocityOnPolicyRunner,
)
