from pathlib import Path
import sys
import unittest

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.black_line_detector import BlackLineConfig, BlackLineDetector  # noqa: E402


class BlackLineDetectorTests(unittest.TestCase):
    def setUp(self):
        self.detector = BlackLineDetector(BlackLineConfig(
            roi_normalized=(0, 0, 1, 1), grayscale_max=80,
            min_line_width_px=8, max_line_width_ratio=.2,
            min_valid_scan_rows=4,
        ))

    def scene(self):
        return np.full((480, 640, 3), 190, np.uint8)

    def test_centered_vertical_line(self):
        image = self.scene()
        cv2.line(image, (320, 479), (320, 0), (20, 20, 20), 24)
        result = self.detector.process(image)
        self.assertTrue(result.found)
        self.assertAlmostEqual(result.lateral_offset_px, 0, delta=3)
        self.assertAlmostEqual(result.heading_error_deg, 0, delta=2)

    def test_slanted_line_has_direction_and_offset(self):
        image = self.scene()
        cv2.line(image, (400, 479), (260, 0), (15, 15, 15), 22)
        result = self.detector.process(image)
        self.assertTrue(result.found)
        self.assertGreater(result.lateral_offset_px, 40)
        self.assertLess(result.heading_error_deg, -10)

    def test_no_line_reports_lost(self):
        result = self.detector.process(self.scene())
        self.assertFalse(result.found)
        self.assertIsNone(result.center_px)

    def test_large_dark_wall_is_not_a_line(self):
        image = self.scene()
        cv2.rectangle(image, (0, 0), (639, 250), (10, 10, 10), -1)
        result = self.detector.process(image)
        self.assertFalse(result.found)

    def test_cross_line_reports_intersection(self):
        image = self.scene()
        cv2.line(image, (320, 479), (320, 0), (20, 20, 20), 24)
        cv2.line(image, (0, 250), (639, 250), (20, 20, 20), 24)
        result = self.detector.process(image)
        self.assertTrue(result.found)
        self.assertTrue(result.intersection_detected)

    def test_empty_image_raises(self):
        with self.assertRaises(ValueError):
            self.detector.process(np.array([], np.uint8))


if __name__ == "__main__":
    unittest.main()
