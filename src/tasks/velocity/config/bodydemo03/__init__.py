"""Register the Bodydemo03 flat and rough velocity tasks."""

from dataclasses import replace

from mjlab.tasks.registry import register_mjlab_task

from .env_cfgs import bodydemo03_flat_env_cfg, bodydemo03_rough_env_cfg
from .rl_cfg import bodydemo03_ppo_runner_cfg
from .runner import Bodydemo03VelocityOnPolicyRunner

register_mjlab_task(
  task_id="Bodydemo03-Flat",
  env_cfg=bodydemo03_flat_env_cfg(),
  play_env_cfg=bodydemo03_flat_env_cfg(play=True),
  rl_cfg=bodydemo03_ppo_runner_cfg(),
  runner_cls=Bodydemo03VelocityOnPolicyRunner,
)

register_mjlab_task(
  task_id="Bodydemo03-Rough",
  env_cfg=bodydemo03_rough_env_cfg(),
  play_env_cfg=bodydemo03_rough_env_cfg(play=True),
  rl_cfg=replace(
    bodydemo03_ppo_runner_cfg(), experiment_name="bodydemo03_velocity_rough"
  ),
  runner_cls=Bodydemo03VelocityOnPolicyRunner,
)
