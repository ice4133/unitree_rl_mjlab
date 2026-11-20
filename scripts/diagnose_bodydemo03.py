"""Compare fixed controls with a trained Bodydemo03 policy in mjlab.

This only changes the configuration of the diagnostic process. It does not edit
training parameters, exported weights, or the deployment controller.
"""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

import mujoco
import numpy as np
import torch

from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import RslRlVecEnvWrapper

from src.tasks.velocity.config.bodydemo03.env_cfgs import bodydemo03_flat_env_cfg
from src.tasks.velocity.config.bodydemo03.rl_cfg import bodydemo03_ppo_runner_cfg
from src.tasks.velocity.config.bodydemo03.runner import Bodydemo03VelocityOnPolicyRunner


LEGS = ("fl", "fr", "rl", "rr")
CASES = ("zero", "fixed_wheels", "policy", "policy_legs_only", "frozen_policy_pose")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--num-envs", type=int, default=8)
    parser.add_argument("--seconds", type=float, default=6.0)
    parser.add_argument("--warmup", type=float, default=2.0)
    parser.add_argument("--randomization", choices=("off", "training"), default="off")
    parser.add_argument("--cases", nargs="+", choices=CASES, default=list(CASES))
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.num_envs < 1 or args.seconds < 2 or args.warmup < 0:
        parser.error("num-envs must be positive; seconds >= 2; warmup >= 0")
    if not args.checkpoint.is_file():
        parser.error(f"Checkpoint does not exist: {args.checkpoint}")
    if "frozen_policy_pose" in args.cases and (
        "policy" not in args.cases
        or args.cases.index("policy") > args.cases.index("frozen_policy_pose")
    ):
        parser.error("policy must precede frozen_policy_pose")
    output = args.output or Path("logs/diagnostics/bodydemo03") / (
        datetime.now(ZoneInfo("Asia/Shanghai")).strftime("%Y-%m-%d_%H-%M-%S")
        + "_" + args.randomization
    )
    output.mkdir(parents=True, exist_ok=True)

    cfg = bodydemo03_flat_env_cfg(play=True)
    cfg.scene.num_envs = args.num_envs
    cfg.seed = 42
    command = cfg.commands["twist"]
    command.ranges.lin_vel_x = (0.0, 0.0)
    command.ranges.lin_vel_y = (0.0, 0.0)
    command.ranges.ang_vel_z = (0.0, 0.0)
    command.rel_standing_envs = 1.0
    # Keep the initial heading and position identical across comparisons.
    cfg.events["reset_base"].params["pose_range"] = {}
    if args.randomization == "off":
        for name in ("foot_friction", "base_com", "encoder_bias"):
            cfg.events.pop(name, None)

    device = "cuda:0" if torch.cuda.is_available() else "cpu"
    env = RslRlVecEnvWrapper(ManagerBasedRlEnv(cfg=cfg, device=device))
    try:
        runner = Bodydemo03VelocityOnPolicyRunner(
            env, asdict(bodydemo03_ppo_runner_cfg()), device=device
        )
        runner.load(str(args.checkpoint), load_cfg={"actor": True}, strict=True, map_location=device)
        policy = runner.get_inference_policy(device=device)
        core = env.unwrapped
        robot = core.scene["robot"]
        sensor = core.scene["feet_ground_contact"]
        # Resolve the actual sensor output order rather than assuming it.
        primary_names = [slot.primary_name for slot in sensor._slots if slot.field_name == "found"]
        sensor_ids = [primary_names.index(f"{leg}_foot_collision") for leg in LEGS]
        wheel_ids = [robot.joint_names.index(f"{leg}_foot_joint") for leg in LEGS]
        geom_ids = [core.sim.mj_model.geom(f"robot/{leg}_foot_collision").id for leg in LEGS]
        zero = torch.zeros((args.num_envs, 16), device=device)
        warmup_steps = round(args.warmup / core.step_dt)
        steps = round(args.seconds / core.step_dt)
        summaries: dict = {}
        frozen_actions = zero.clone()

        for case in args.cases:
            obs, _ = env.reset()
            with torch.no_grad():
                for _ in range(warmup_steps):
                    obs, _, _, _ = env.step(zero)
            start = robot.data.root_link_pos_w.detach().cpu().numpy().copy()
            records: dict[str, list] = {
                name: [] for name in (
                    "force_z", "contact", "velocity", "angular_velocity", "position",
                    "joint_pos", "wheel_velocity", "raw_action", "reward", "done", "reward_terms",
                )
            }
            for step in range(steps):
                with torch.no_grad():
                    action = zero.clone()
                    if case == "fixed_wheels":
                        # Raw joint axes are mirrored on the right side.
                        action[:, 12:] = torch.tensor([0.2, -0.2, 0.2, -0.2], device=device)
                    elif case in ("policy", "policy_legs_only"):
                        action = policy(obs).clone()
                        if case == "policy_legs_only":
                            action[:, 12:] = 0.0
                    elif case == "frozen_policy_pose":
                        # Replay the preceding full policy's final mean leg pose
                        # without inference or wheel rotation; ramp in over 1 s.
                        action = frozen_actions * min(1.0, (step + 1) * core.step_dt)
                    obs, reward, done, _ = env.step(action)
                values = {
                    "force_z": sensor.data.force[:, sensor_ids, 2].abs(),
                    "contact": sensor.data.found[:, sensor_ids] > 0,
                    "velocity": robot.data.root_link_lin_vel_b,
                    "angular_velocity": robot.data.root_link_ang_vel_b,
                    "position": robot.data.root_link_pos_w,
                    "joint_pos": robot.data.joint_pos,
                    "wheel_velocity": robot.data.joint_vel[:, wheel_ids],
                    "raw_action": action,
                    "reward": reward,
                    "done": done,
                    "reward_terms": core.reward_manager._step_reward,
                }
                for name, value in values.items():
                    records[name].append(value.detach().cpu().numpy().copy())

            arrays = {name: np.stack(value) for name, value in records.items()}
            # Exclude transition/settling: use the final two seconds.
            tail = slice(-round(2.0 / core.step_dt), None)
            force = arrays["force_z"][tail]
            speed = np.linalg.norm(arrays["velocity"][tail, :, :2], axis=-1)
            if case == "policy":
                pose = arrays["raw_action"][tail].mean(axis=0)
                pose[:, 12:] = 0.0
                frozen_actions = torch.as_tensor(pose, device=device)

            # Calculate actual mesh bottoms from the final GPU configuration.
            model = core.sim.mj_model
            bottoms = []
            for qpos in core.sim.data.qpos.detach().cpu().numpy():
                data = mujoco.MjData(model)
                data.qpos[:] = qpos
                mujoco.mj_forward(model, data)
                row = []
                for geom_id in geom_ids:
                    mesh = model.mesh(model.geom_dataid[geom_id])
                    verts = model.mesh_vert[mesh.vertadr[0] : mesh.vertadr[0] + mesh.vertnum[0]]
                    z = verts @ data.geom_xmat[geom_id].reshape(3, 3)[2] + data.geom_xpos[geom_id, 2]
                    row.append(float(z.min() * 1000.0))
                bottoms.append(row)

            summary = {
                "mean_wheel_load_n": force.mean(axis=(0, 1)).tolist(),
                "wheel_support_fraction_over_5n": (force > 5.0).mean(axis=(0, 1)).tolist(),
                "all_four_supported_fraction": (force > 5.0).all(axis=-1).mean().item(),
                "mean_planar_speed_m_s": float(speed.mean()),
                "mean_yaw_rate_rad_s": float(np.abs(arrays["angular_velocity"][tail, :, 2]).mean()),
                "mean_wheel_velocity_rad_s": arrays["wheel_velocity"][tail].mean(axis=(0, 1)).tolist(),
                "mean_wheel_target_rad_s": (arrays["raw_action"][tail, :, 12:] * 5).mean(axis=(0, 1)).tolist(),
                "mean_height_m": float(arrays["position"][tail, :, 2].mean()),
                "displacement_per_env_m": (arrays["position"][-1] - start).tolist(),
                "final_wheel_bottom_mm": bottoms,
                "mean_raw_leg_action": arrays["raw_action"][tail, :, :12].mean(axis=(0, 1)).tolist(),
                "mean_reward_per_step": float(arrays["reward"][tail].mean()),
                "mean_weighted_reward_terms": dict(zip(
                    core.reward_manager.active_terms,
                    arrays["reward_terms"][tail].mean(axis=(0, 1)).tolist(),
                    strict=True,
                )),
                "termination_count": int(arrays["done"].sum()),
            }
            summaries[case] = summary
            np.savez_compressed(output / f"{case}.npz", **arrays)
            print("CASE", case, json.dumps(summary), flush=True)

        report = {
            "checkpoint": str(args.checkpoint.resolve()),
            "randomization": args.randomization,
            "num_envs": args.num_envs,
            "step_dt": core.step_dt,
            "physics_dt": core.physics_dt,
            "seconds_per_case": args.seconds,
            "warmup_seconds": args.warmup,
            "wheel_order": LEGS,
            "joint_order": robot.joint_names,
            "encoder_bias": robot.data.encoder_bias.detach().cpu().tolist(),
            "cases": summaries,
        }
        (output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
        print("REPORT", output / "report.json", flush=True)
    finally:
        env.close()


if __name__ == "__main__":
    main()
