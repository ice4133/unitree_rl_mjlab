"""Go2W command types backed by the shared wheeled velocity sampler."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

from src.tasks.velocity.mdp.wheeled_velocity_command import (
  WheeledVelocityCommand,
  WheeledVelocityCommandCfg,
)

if TYPE_CHECKING:
  from mjlab.envs import ManagerBasedRlEnv


class Go2WVelocityCommand(WheeledVelocityCommand):
  cfg: Go2WVelocityCommandCfg


@dataclass(kw_only=True)
class Go2WVelocityCommandCfg(WheeledVelocityCommandCfg):
  def build(self, env: ManagerBasedRlEnv) -> Go2WVelocityCommand:
    return Go2WVelocityCommand(self, env)
