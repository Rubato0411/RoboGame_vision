from pathlib import Path
import sys
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.block_detector_robust import BlockDetection  # noqa: E402
from src.block_pose_estimator import BlockPoseEstimator  # noqa: E402
from src.camera_calibration import CalibrationResult, ChessboardSpec  # noqa: E402
from src.coordinate_transform import CoordinateTransformer, RigidTransform, rotation_from_rpy  # noqa: E402


class BlockPoseEstimatorTests(unittest.TestCase):
    def test_center_and_top_grasp_height(self):
        calibration = CalibrationResult(
            640, 480, np.array([[500., 0., 320.], [0., 500., 240.], [0., 0., 1.]]),
            np.zeros(5), 0, 0, (), (), (), ChessboardSpec())
        robot_camera = RigidTransform(rotation_from_rpy(180, 0, 0, degrees=True), [0, 0, 1])
        transformer = CoordinateTransformer(calibration, robot_camera, {})
        detection = BlockDetection(
            "orange", (320, 220), (270, 140, 100, 101),
            ((270, 140), (370, 140), (370, 241), (270, 241)),
            0, 10000, 1, 1, 1, 1, .9, False)
        result = BlockPoseEstimator(transformer, cube_size_m=.15).estimate(detection)
        self.assertTrue(result.valid)
        self.assertAlmostEqual(result.center_robot_m[2], .075, places=6)
        self.assertAlmostEqual(result.grasp_point_robot_m[2], .15, places=6)


if __name__ == "__main__": unittest.main()
