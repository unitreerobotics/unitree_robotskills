"""Offline coverage of the shipped model, its assets and the strict IK adapter."""

import hashlib
import importlib.abc
import importlib.util
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from unitree_gpt6.g1d_gomoku.models.g1 import resources
from unitree_gpt6.g1d_gomoku.models.g1.config import (
    ARM_JOINT_NAMES,
    G1_29_JointArmIndex,
    G1ArmConfig,
)


class BundledAssetsTests(unittest.TestCase):
    def test_all_shipped_meshes_match_verified_manifest(self):
        root = resources.asset_directory()
        self.assertEqual(root, Path(__file__).resolve().parents[1] / "models/g1/assets")
        manifest = json.loads((root.parent / "manifest.json").read_text())
        self.assertEqual(resources.validate_assets()["mesh_count"], 49)
        self.assertEqual(len(manifest["files"]), 50)
        for name, expected in manifest["files"].items():
            with self.subTest(asset=name):
                self.assertEqual(hashlib.sha256((root / name).read_bytes()).hexdigest(), expected)

    def test_missing_urdf_or_mesh_fails_before_model_initialization(self):
        urdf = (resources.asset_directory() / resources.URDF_NAME).read_bytes()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with mock.patch.object(resources, "asset_directory", return_value=root):
                with self.assertRaisesRegex(ValueError, "URDF missing"):
                    resources.validate_assets()
                (root / resources.URDF_NAME).write_bytes(urdf)
                with self.assertRaisesRegex(ValueError, "mesh missing"):
                    resources.validate_assets()

    def test_resource_import_does_not_load_numerical_or_device_modules(self):
        code = """
import sys
from unitree_gpt6.g1d_gomoku.models.g1 import validate_assets
assert validate_assets()['mesh_count'] == 49
assert not {'numpy', 'pinocchio', 'casadi', 'unitree_deploy', 'unitree_sdk2py'} & sys.modules.keys()
assert 'unitree_gpt6.g1d_gomoku.src.unitree_sdk2py' not in sys.modules
"""
        subprocess.run([sys.executable, "-B", "-c", code], check=True)

    def test_dds_joint_order_and_gains_match_deployed_contract(self):
        self.assertEqual([int(index) for index in G1_29_JointArmIndex], list(range(15, 29)))
        self.assertEqual(ARM_JOINT_NAMES[0], "left_shoulder_pitch_joint")
        self.assertEqual(ARM_JOINT_NAMES[7], "right_shoulder_pitch_joint")
        self.assertEqual(ARM_JOINT_NAMES[-1], "right_wrist_yaw_joint")
        gains = G1ArmConfig()
        self.assertEqual(
            (
                gains.kp_high,
                gains.kd_high,
                gains.kp_low,
                gains.kd_low,
                gains.kp_wrist,
                gains.kd_wrist,
            ),
            (300.0, 3.0, 80.0, 3.0, 40.0, 1.5),
        )


class NoExternalRobotImports(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split(".")[0] in {
            "unitree_deploy",
            "unitree_sdk2py",
            "cyclonedds",
        } or fullname.startswith("unitree_gpt6.g1d_gomoku.src.unitree_sdk2py"):
            raise AssertionError(f"Offline model must not import {fullname}")
        return None


class BundledKinematicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        for name in ("numpy", "scipy", "pinocchio", "casadi"):
            if importlib.util.find_spec(name) is None:
                raise unittest.SkipTest("optional numerical dependencies are not installed")
        cls.blocker = NoExternalRobotImports()
        sys.meta_path.insert(0, cls.blocker)
        cls.addClassCleanup(sys.meta_path.remove, cls.blocker)
        from unitree_gpt6.g1d_gomoku.src.hardware.kinematics import Kinematics

        cls.kin = Kinematics()

    def test_mesh_geometry_fk_and_gravity_with_nonzero_waist(self):
        import numpy as np
        import pinocchio as pin

        kin = self.kin
        self.assertEqual(tuple(kin.names), ARM_JOINT_NAMES)
        self.assertEqual(len(kin.ik.robot.visual_model.geometryObjects), 49)
        q = np.linspace(-0.1, 0.1, 14)
        kin.set_waist([0.1, 0.05, -0.03])
        poses = kin.fk(q)
        model = kin.ik.robot.model
        full_q = kin.full_q.copy()
        for name, value in zip(kin.names, q, strict=True):
            full_q[model.joints[model.getJointId(name)].idx_q] = value
        data = model.createData()
        pin.framesForwardKinematics(model, data, full_q)
        offset = pin.SE3(np.eye(3), np.array([0.05, 0.0, 0.0]))
        for side in ("left", "right"):
            wrist = data.oMi[model.getJointId(f"{side}_wrist_yaw_joint")]
            np.testing.assert_allclose(poses[side], (wrist * offset).homogeneous, atol=1e-12)
        gravity = kin.tau(q)
        self.assertTrue(np.isfinite(gravity).all())
        kin.set_waist([0.0, 0.0, 0.0])
        self.assertFalse(np.allclose(gravity, kin.tau(q)))

    def test_real_ik_holds_other_arm_and_rejects_unreachable_target(self):
        import numpy as np

        from unitree_gpt6.g1d_gomoku.src.hardware.kinematics import matrix_pose

        kin = self.kin
        kin.set_waist([0.1, 0.05, -0.03])
        seed = np.zeros(14)
        target = matrix_pose(kin.fk(seed)["right"])
        target[0] -= 0.005
        result = kin.solve({"right": target}, seed)
        np.testing.assert_array_equal(result["joint_positions"][:7], seed[:7])
        self.assertLess(result["errors"]["right"]["position_m"], 0.0001)
        self.assertLess(result["errors"]["right"]["rotation_rad"], 0.0001)
        target[0] += 10
        with self.assertRaises(ValueError):
            kin.solve({"right": target}, seed)


if __name__ == "__main__":
    unittest.main()
