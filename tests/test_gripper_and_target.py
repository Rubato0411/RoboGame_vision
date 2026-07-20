from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.gripper_alignment import GripperAligner, GripperAlignmentConfig  # noqa: E402
from src.target_selector import TargetSelector, TargetSelectionConfig  # noqa: E402
from src.vision_output import BlockOutput  # noqa: E402


class GripperAndTargetTests(unittest.TestCase):
    def test_alignment_requires_position_angle_and_real_detection(self):
        aligner = GripperAligner(GripperAlignmentConfig())
        self.assertTrue(aligner.align((646, 355), 3, .9, 2).aligned)
        self.assertTrue(aligner.align((646, 355), -90, .9, 2).aligned)
        self.assertFalse(aligner.align((700, 355), 3, .9, 2).aligned)
        self.assertFalse(aligner.align((646, 355), 3, .9, 2, predicted=True).valid)

    def test_target_selector_filters_color_and_prefers_confidence(self):
        blocks = [
            BlockOutput(1, "purple", True, False, (640, 300), (0, 0, 10, 10), None, .99),
            BlockOutput(2, "orange", True, False, (620, 300), (0, 0, 10, 10), None, .92),
            BlockOutput(3, "orange", True, False, (640, 300), (0, 0, 10, 10), None, .70),
        ]
        result = TargetSelector(TargetSelectionConfig()).select(blocks)
        self.assertTrue(result.valid)
        self.assertEqual(result.track_id, 2)


if __name__ == "__main__":
    unittest.main()
