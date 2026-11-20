"""Check the standalone scene against the SDK bridge and training contract.

Run with the project's Python environment (MuJoCo, NumPy and PyYAML).
"""

from pathlib import Path
import unittest

import mujoco
import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[4]
SCENE = ROOT / "src/assets/robots/bodydemo03/xmls/scene_bodydemo03_deploy.xml"
DEPLOY = ROOT / "deploy/robots/bodydemo03/config/policy/velocity/v0/params/deploy.yaml"


class DeploymentSceneTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = mujoco.MjModel.from_xml_path(str(SCENE))
        cls.cfg = yaml.safe_load(DEPLOY.read_text())
        cls.joints = [
            f"{leg}_{part}_joint"
            for leg in ("fl", "fr", "rl", "rr")
            for part in ("hip", "thigh", "calf", "foot")
        ]

    def test_sdk_actuator_and_sensor_mapping(self):
        model = self.model
        self.assertEqual(model.nu, 16)
        for joint_index, motor_id in enumerate(self.cfg["joint_ids_map"]):
            joint_id = model.joint(self.joints[joint_index]).id
            self.assertEqual(model.actuator_trnid[motor_id, 0], joint_id)
            for block in range(3):
                sensor_id = block * 16 + motor_id
                self.assertEqual(model.sensor_objid[sensor_id], joint_id)
                self.assertEqual(model.sensor_adr[sensor_id], sensor_id)
            # No built-in position/velocity feedback: the SDK bridge owns PD.
            self.assertEqual(model.actuator_biastype[motor_id], mujoco.mjtBias.mjBIAS_NONE)
            np.testing.assert_array_equal(model.actuator_gear[motor_id], [1, 0, 0, 0, 0, 0])
            limit = 8 if joint_index % 4 == 3 else 50
            np.testing.assert_array_equal(model.actuator_ctrlrange[motor_id], [-limit, limit])
        for name, dim in (("imu_quat", 4), ("imu_gyro", 3), ("imu_acc", 3)):
            self.assertEqual(model.sensor(name).dim[0], dim)

    def test_training_geometry_and_joint_parameters(self):
        from src.assets.robots.bodydemo03.bodydemo03_constants import (
            get_bodydemo03_velocity_spec,
        )

        training = get_bodydemo03_velocity_spec().compile()
        for name in self.joints:
            actual, expected = self.model.joint(name), training.joint(name)
            np.testing.assert_allclose(actual.axis, expected.axis)
            np.testing.assert_allclose(actual.range, expected.range)
            np.testing.assert_array_equal(actual.limited, expected.limited)
        for body_id in range(1, training.nbody):
            name = training.body(body_id).name
            actual, expected = self.model.body(name), training.body(name)
            np.testing.assert_allclose(actual.mass, expected.mass)
            np.testing.assert_allclose(actual.inertia, expected.inertia)
        for leg in ("fl", "fr", "rl", "rr"):
            wheel = self.model.geom(f"{leg}_foot_collision")
            self.assertEqual(wheel.condim[0], 3)
            self.assertEqual(wheel.friction[0], 0.8)

    def test_explicit_pd_zero_pose(self):
        model = self.model
        data = mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
        mujoco.mj_forward(model, data)
        kp = np.asarray(self.cfg["stiffness"])
        kd = np.asarray(self.cfg["damping"])
        max_torque = np.zeros(16)
        max_late_speed = 0.0
        for step in range(round(5.0 / model.opt.timestep)):
            # The exact formula and SDK sensor blocks used in RobotBridge::run.
            data.ctrl[:] = -kp * data.sensordata[:16] - kd * data.sensordata[16:32]
            mujoco.mj_step(model, data)
            self.assertTrue(np.isfinite(data.qpos).all())
            self.assertGreater(data.qpos[2], 0.2, "Zero-pose controller collapsed")
            max_torque = np.maximum(max_torque, np.abs(data.actuator_force))
            if step > 4.0 / model.opt.timestep:
                max_late_speed = max(max_late_speed, np.max(np.abs(data.qvel[6:])))
        np.testing.assert_array_less(max_torque, np.array([50.0001] * 12 + [8.0001] * 4))
        self.assertLess(max_late_speed, 0.5, "Persistent explicit-PD oscillation")


if __name__ == "__main__":
    unittest.main()
