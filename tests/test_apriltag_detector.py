from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.apriltag_detector import AprilTagConfig, AprilTagDetector  # noqa: E402
from src.camera_calibration import CalibrationResult, ChessboardSpec  # noqa: E402
from run_apriltag_demo import fit_for_display  # noqa: E402


class AprilTagDetectorTests(unittest.TestCase):
    def make_scene(self, tag_id=3, size=240):
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        marker = cv2.aruco.generateImageMarker(dictionary, tag_id, size)
        image = np.full((600, 800, 3), 230, np.uint8)
        image[180:180 + size, 280:280 + size] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
        return image

    def test_detects_allowed_tag(self):
        detector = AprilTagDetector(AprilTagConfig())
        result = detector.process(self.make_scene(3))
        self.assertEqual([item.tag_id for item in result.detections], [3])
        self.assertAlmostEqual(result.detections[0].center_px[0], 399.5, delta=2)

    def test_ignores_id_outside_competition_set(self):
        detector = AprilTagDetector(AprilTagConfig())
        result = detector.process(self.make_scene(10))
        self.assertEqual(result.detections, ())
        self.assertEqual(result.ignored_ids, (10,))

    def test_pose_is_available_with_calibration(self):
        calibration = CalibrationResult(
            800, 600, np.array([[800., 0., 400.], [0., 800., 300.], [0., 0., 1.]]),
            np.zeros(5), 0., 0., (), (), (), ChessboardSpec(),
        )
        detector = AprilTagDetector(AprilTagConfig(), calibration)
        result = detector.process(self.make_scene(2))
        self.assertEqual(len(result.detections), 1)
        self.assertIsNotNone(result.detections[0].distance_m)
        self.assertGreater(result.detections[0].distance_m, 0)

    def test_rejects_wrong_resolution_for_pose(self):
        calibration = CalibrationResult(
            640, 480, np.eye(3), np.zeros(5), 0., 0., (), (), (), ChessboardSpec(),
        )
        detector = AprilTagDetector(AprilTagConfig(), calibration)
        with self.assertRaises(ValueError):
            detector.process(self.make_scene(1))

    def test_rejects_unsupported_pose_image_rotation(self):
        with self.assertRaisesRegex(ValueError, "must be 0 or 180"):
            AprilTagDetector(AprilTagConfig(pose_image_rotation_deg=90))

    def test_default_pose_corner_order_is_unchanged(self):
        detector = AprilTagDetector(AprilTagConfig())
        points = np.array([[10, 20], [30, 20], [30, 40], [10, 40]], np.float32)
        np.testing.assert_array_equal(detector._pose_image_points(points), points)

    def test_upside_down_pose_corner_order_is_rotated_180(self):
        detector = AprilTagDetector(AprilTagConfig(pose_image_rotation_deg=180))
        points = np.array([[10, 20], [30, 20], [30, 40], [10, 40]], np.float32)
        np.testing.assert_array_equal(
            detector._pose_image_points(points), points[[2, 3, 0, 1]])

    def test_pose_uses_remapped_points_for_solve_and_reprojection_error(self):
        calibration = CalibrationResult(
            800, 600, np.array([[800., 0., 400.], [0., 800., 300.], [0., 0., 1.]]),
            np.zeros(5), 0., 0., (), (), (), ChessboardSpec(),
        )
        detector = AprilTagDetector(
            AprilTagConfig(pose_image_rotation_deg=180), calibration)
        points = np.array([[10, 20], [30, 20], [30, 40], [10, 40]], np.float32)
        expected = points[[2, 3, 0, 1]]
        with (patch("src.apriltag_detector.cv2.solvePnP",
                    return_value=(True, np.zeros((3, 1)), np.ones((3, 1)))) as solve,
              patch("src.apriltag_detector.cv2.projectPoints",
                    return_value=(expected.reshape(-1, 1, 2), None))):
            pose = detector._estimate_pose(points, (600, 800))
        np.testing.assert_array_equal(solve.call_args.args[1], expected)
        self.assertEqual(pose[3], 0.0)

    def test_preview_is_scaled_without_changing_aspect_ratio(self):
        image = np.zeros((3000, 4000, 3), np.uint8)
        preview = fit_for_display(image, 1280, 720)
        self.assertEqual(preview.shape[:2], (720, 960))

    def test_small_preview_is_not_enlarged(self):
        image = np.zeros((480, 640, 3), np.uint8)
        self.assertIs(fit_for_display(image, 1280, 720), image)


if __name__ == "__main__":
    unittest.main()
