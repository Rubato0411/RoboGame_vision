from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.competition_controller import GripperPoseFeedback  # noqa: E402
from src.coordinate_transform import RigidTransform  # noqa: E402
from tools.run_gripper_3d_commissioning import (  # noqa: E402
    DynamicPoseState, observation_payload as gripper_payload)
from tools.test_placement_slot_live import (  # noqa: E402
    configured_slot_ids, observation_payload as placement_payload)
from src.vision_output import StreamOutput, VisionOutput  # noqa: E402


class CommissioningToolTests(unittest.TestCase):
    def test_dynamic_pose_requires_new_sequence_and_expires(self):
        state = DynamicPoseState(RigidTransform.identity(), 0.15)
        pose = GripperPoseFeedback(True, 7, (0.1, 0.2, 0.3), (0.0, 0.0, 0.0))
        self.assertTrue(state.update(pose, 1.0))
        transform, age, reason = state.current(1.1)
        self.assertIsNotNone(transform)
        self.assertAlmostEqual(age, 0.1)
        self.assertEqual(reason, "fresh pose")
        self.assertFalse(state.update(pose, 1.12))
        transform, _, reason = state.current(1.16)
        self.assertIsNone(transform)
        self.assertEqual(reason, "gripper pose timed out")

    def test_invalid_pose_revokes_transform_immediately(self):
        state = DynamicPoseState(RigidTransform.identity(), 0.15)
        state.update(GripperPoseFeedback(True, 1, (0, 0, 0), (0, 0, 0)), 0.0)
        state.update(GripperPoseFeedback(False), 0.01)
        self.assertIsNone(state.current(0.01)[0])

    def test_repository_exposes_twelve_configured_levels(self):
        ids = configured_slot_ids(ROOT / "configs" / "placement_slots.json")
        self.assertEqual(len(ids), 12)
        self.assertIn("building_1_level_1", ids)
        self.assertIn("building_3_level_4", ids)

    def test_payload_helpers_serialize_nested_dataclasses(self):
        vision = VisionOutput(
            1, 0.0, "test", "IDLE",
            StreamOutput("OK", True, 30.0, 0.0, 0, "healthy"))
        state = DynamicPoseState(RigidTransform.identity(), 0.15)
        self.assertEqual(gripper_payload(
            vision, state, None, None, "no valid gripper pose")["blocks"], [])
        self.assertFalse(gripper_payload(
            vision, state, None, None, "no valid gripper pose")["pose_valid"])
        self.assertEqual(placement_payload(vision)["slot"]["valid"], False)


if __name__ == "__main__":
    unittest.main()
