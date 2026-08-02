from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.coordinate_transform import RigidTransform  # noqa: E402
from src.hand_eye_calibration import (HandEyeSample, calibrate_eye_in_hand,
                                      load_hand_eye_samples)  # noqa: E402


class HandEyeCalibrationTests(unittest.TestCase):
    def test_recovers_eye_in_hand_transform(self):
        gripper_from_camera = RigidTransform.from_xyz_rpy(
            [0.035, -0.018, 0.072], [8.0, -12.0, 176.0])
        base_from_target = RigidTransform.from_xyz_rpy(
            [0.55, 0.12, 0.28], [90.0, 0.0, -20.0])
        poses = [
            ([0.20, -0.10, 0.32], [5, -20, -35]),
            ([0.27, -0.04, 0.40], [-18, -5, -10]),
            ([0.31, 0.08, 0.35], [12, 14, 18]),
            ([0.18, 0.13, 0.43], [-12, 22, 42]),
            ([0.36, -0.12, 0.29], [25, -8, 65]),
            ([0.23, 0.02, 0.48], [-22, 18, 92]),
            ([0.39, 0.10, 0.38], [17, 27, 118]),
            ([0.29, -0.16, 0.45], [-28, -16, 145]),
        ]
        samples = []
        for index, (xyz, rpy) in enumerate(poses):
            base_from_gripper = RigidTransform.from_xyz_rpy(xyz, rpy)
            camera_from_target = base_from_gripper.compose(
                gripper_from_camera).inverse().compose(base_from_target)
            samples.append(HandEyeSample(
                str(index), base_from_gripper, camera_from_target))

        result = calibrate_eye_in_hand(samples, "PARK")
        np.testing.assert_allclose(
            result.transform_gripper_camera.translation,
            gripper_from_camera.translation, atol=1e-7)
        np.testing.assert_allclose(
            result.transform_gripper_camera.rotation,
            gripper_from_camera.rotation, atol=1e-7)
        self.assertLess(result.translation_rms_m, 1e-8)
        self.assertLess(result.rotation_rms_deg, 1e-5)

    def test_example_file_is_intentionally_empty(self):
        with self.assertRaisesRegex(ValueError, "at least 5"):
            load_hand_eye_samples(ROOT / "configs" / "hand_eye_samples.example.json")

    def test_rejects_insufficient_motion(self):
        identity = RigidTransform.identity()
        samples = tuple(HandEyeSample(str(index), identity, identity) for index in range(5))
        with self.assertRaisesRegex(ValueError, "translation span"):
            calibrate_eye_in_hand(samples)


if __name__ == "__main__":
    unittest.main()
