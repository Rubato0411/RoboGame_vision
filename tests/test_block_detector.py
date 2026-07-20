from pathlib import Path
import sys
import unittest

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.block_detector_robust import BlockDetector  # noqa: E402


class BlockDetectorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.detector = BlockDetector.from_json(ROOT / "configs" / "block_detector_robust.json")

    def test_detects_orange_and_purple_rectangles(self):
        image = np.full((480, 640, 3), (120, 120, 120), dtype=np.uint8)
        cv2.rectangle(image, (80, 120), (220, 270), (0, 140, 255), -1)
        cv2.rectangle(image, (380, 150), (520, 300), (180, 50, 150), -1)

        result = self.detector.process(image)
        classes = sorted(item.class_name for item in result.detections)

        self.assertEqual(classes, ["orange", "purple"])
        centers = {item.class_name: item.center_px for item in result.detections}
        self.assertAlmostEqual(centers["orange"][0], 150, delta=3)
        self.assertAlmostEqual(centers["purple"][0], 450, delta=3)

    def test_rejects_small_color_noise(self):
        image = np.full((480, 640, 3), 100, dtype=np.uint8)
        cv2.circle(image, (100, 100), 4, (0, 140, 255), -1)
        cv2.circle(image, (300, 250), 3, (180, 50, 150), -1)

        result = self.detector.process(image)

        self.assertEqual(result.detections, ())

    def test_rejects_extremely_thin_region(self):
        image = np.full((480, 640, 3), 100, dtype=np.uint8)
        cv2.rectangle(image, (50, 200), (500, 210), (0, 140, 255), -1)

        result = self.detector.process(image)

        self.assertEqual(result.detections, ())

    def test_empty_image_raises(self):
        with self.assertRaises(ValueError):
            self.detector.process(np.array([], dtype=np.uint8))

    def test_detects_rotated_block(self):
        image = np.full((480, 640, 3), 110, dtype=np.uint8)
        box = cv2.boxPoints(((320, 240), (150, 150), 32)).astype(np.int32)
        cv2.fillConvexPoly(image, box, (0, 140, 255))
        result = self.detector.process(image)
        self.assertEqual(len(result.detections), 1)
        self.assertEqual(result.detections[0].class_name, "orange")
        self.assertGreater(result.detections[0].rotated_rectangularity, 0.85)

    def test_reports_rejection_reasons(self):
        image = np.full((480, 640, 3), 100, dtype=np.uint8)
        cv2.rectangle(image, (20, 20), (25, 25), (0, 140, 255), -1)
        result = self.detector.process(image)
        self.assertGreaterEqual(result.rejection_counts.get("area_too_small", 0), 1)

    def test_splits_three_connected_blocks(self):
        image = np.full((500, 700, 3), 100, dtype=np.uint8)
        orange = (0, 140, 255)
        cv2.rectangle(image, (180, 260), (330, 410), orange, -1)
        cv2.rectangle(image, (335, 260), (485, 410), orange, -1)
        cv2.rectangle(image, (260, 105), (410, 265), orange, -1)
        result = self.detector.process(image)
        orange_detections = [item for item in result.detections if item.class_name == "orange"]
        self.assertEqual(len(orange_detections), 3)
        self.assertGreaterEqual(result.rejection_counts.get("connected_regions_split", 0), 2)


if __name__ == "__main__":
    unittest.main()
