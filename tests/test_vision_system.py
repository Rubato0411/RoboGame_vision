from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from run_vision_system import annotate_output, fit_for_display, parse_source  # noqa: E402
from src.vision_output import StreamOutput, VisionOutput  # noqa: E402


class VisionSystemTests(unittest.TestCase):
    def test_source_parser(self):
        self.assertEqual(parse_source("0"), 0)
        self.assertEqual(parse_source("video.mp4"), "video.mp4")

    def test_display_fit_preserves_aspect(self):
        image = np.zeros((2160, 3840, 3), np.uint8)
        self.assertEqual(fit_for_display(image, 1280, 720).shape[:2], (720, 1280))

    def test_annotation_keeps_image_shape(self):
        image = np.zeros((480, 640, 3), np.uint8)
        output = VisionOutput(0, 0, "test", "IDLE",
                              StreamOutput("HEALTHY", True, 30, 0, 0, "ok"))
        self.assertEqual(annotate_output(image, output).shape, image.shape)


if __name__ == "__main__":
    unittest.main()
