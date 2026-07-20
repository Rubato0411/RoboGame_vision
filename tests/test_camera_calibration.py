from pathlib import Path
import sys
import tempfile
import unittest

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.camera_calibration import (  # noqa: E402
    CalibrationObservation,
    CalibrationResult,
    CameraCalibrator,
    ChessboardSpec,
)


class CameraCalibrationTests(unittest.TestCase):
    def test_object_points_use_metric_square_size(self):
        board = ChessboardSpec(9, 6, 0.025)
        points = board.object_points()
        self.assertEqual(points.shape, (54, 3))
        self.assertAlmostEqual(float(points[1, 0] - points[0, 0]), 0.025, places=6)

    def test_detects_generated_chessboard(self):
        board = ChessboardSpec(9, 6, 0.025)
        image = np.full((700, 1000), 255, np.uint8)
        square = 80
        for row in range(7):
            for col in range(10):
                if (row + col) % 2 == 0:
                    cv2.rectangle(image, (100 + col*square, 70 + row*square),
                                  (100 + (col+1)*square, 70 + (row+1)*square), 0, -1)
        observation = CameraCalibrator(board).detect_corners(cv2.cvtColor(image, cv2.COLOR_GRAY2BGR))
        self.assertIsNotNone(observation)
        self.assertEqual(observation.image_points.shape, (54, 2))

    def test_synthetic_calibration_has_low_error(self):
        board = ChessboardSpec(9, 6, 0.025)
        calibrator = CameraCalibrator(board)
        matrix = np.array([[820.0, 0, 640.0], [0, 815.0, 360.0], [0, 0, 1.0]])
        distortion = np.array([-0.12, 0.04, 0.001, -0.001, 0.0])
        observations = []
        poses = [
            ((0.05, -0.10, 0.02), (-0.10, -0.05, 0.75)),
            ((-0.10, 0.06, 0.08), (0.05, -0.08, 0.90)),
            ((0.12, 0.03, -0.08), (-0.05, 0.02, 0.65)),
            ((-0.05, -0.12, 0.12), (0.12, 0.03, 1.00)),
            ((0.16, 0.10, 0.04), (-0.12, 0.08, 0.85)),
            ((-0.14, 0.02, -0.10), (0.02, 0.10, 0.70)),
            ((0.08, -0.18, 0.15), (0.10, -0.10, 0.95)),
            ((-0.18, 0.12, 0.05), (-0.08, 0.04, 0.80)),
            ((0.20, 0.04, -0.12), (0.00, -0.12, 1.10)),
            ((-0.08, -0.04, 0.20), (0.15, 0.05, 0.88)),
            ((0.03, 0.15, -0.16), (-0.15, -0.02, 0.92)),
            ((-0.15, -0.10, -0.04), (0.08, 0.12, 0.78)),
        ]
        for index, (rotation, translation) in enumerate(poses):
            image_points, _ = cv2.projectPoints(
                board.object_points(), np.array(rotation, dtype=np.float64),
                np.array(translation, dtype=np.float64), matrix, distortion
            )
            observations.append(CalibrationObservation(
                f"synthetic_{index}", (1280, 720), image_points.reshape(-1, 2).astype(np.float32)
            ))
        result = calibrator.calibrate(observations, min_views=10, max_view_error_px=None)
        self.assertLess(result.mean_reprojection_error, 0.05)
        self.assertAlmostEqual(result.camera_matrix[0, 0], matrix[0, 0], delta=2.0)

    def test_json_round_trip(self):
        result = CalibrationResult(
            640, 480, np.eye(3), np.zeros(5), 0.2, 0.15, (0.1, 0.2),
            ("a.jpg", "b.jpg"), (), ChessboardSpec(),
        )
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "camera.json"
            result.save_json(path)
            loaded = CalibrationResult.load_json(path)
        np.testing.assert_allclose(loaded.camera_matrix, result.camera_matrix)
        self.assertEqual(loaded.board, result.board)


if __name__ == "__main__":
    unittest.main()
