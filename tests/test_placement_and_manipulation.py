from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.apriltag_detector import AprilTagDetection  # noqa: E402
from src.coordinate_transform import RigidTransform  # noqa: E402
from src.manipulation_verifier import (ManipulationEvidence, ManipulationPhase,
                                       ManipulationVerifier)  # noqa: E402
from src.placement_tag_locator import PlacementSlot, PlacementTagLocator  # noqa: E402


class PlacementAndManipulationTests(unittest.TestCase):
    def test_tag_relative_slot_is_transformed_to_robot(self):
        slot = PlacementSlot("base", 4,
                             RigidTransform.from_xyz_rpy([.2, 0, 0], [0, 0, 0]))
        locator = PlacementTagLocator([slot])
        detection = AprilTagDetection(4, ((0, 0),)*4, (0, 0), 1, 1,
                                      np.zeros(3), np.array([1., 0., 2.]), 2, .2)
        target = locator.select_next([detection], RigidTransform.identity())
        self.assertTrue(target.valid)
        np.testing.assert_allclose(target.position_robot_m, [1.2, 0, 2])

    def test_occupied_slot_is_skipped(self):
        slot = PlacementSlot("base", 4, RigidTransform.identity())
        locator = PlacementTagLocator([slot])
        detection = AprilTagDetection(4, ((0, 0),)*4, (0, 0), 1, 1,
                                      np.zeros(3), np.ones(3), 1, .2)
        self.assertFalse(locator.select_next([detection], RigidTransform.identity(), ["base"]).valid)

    def test_grasp_requires_sensor_and_visual_motion(self):
        verifier = ManipulationVerifier()
        incomplete = verifier.verify(ManipulationPhase.VERIFY_GRASP,
                                     ManipulationEvidence(gripper_closed=True,
                                                          target_visible=False))
        self.assertFalse(incomplete.success)
        complete = verifier.verify(ManipulationPhase.VERIFY_GRASP,
                                   ManipulationEvidence(gripper_closed=True,
                                                        contact_detected=True,
                                                        target_moved_with_gripper=True))
        self.assertTrue(complete.success)

    def test_release_requires_target_in_slot(self):
        result = ManipulationVerifier().verify(
            ManipulationPhase.VERIFY_RELEASE,
            ManipulationEvidence(gripper_open=True, contact_detected=False,
                                 target_in_placement_slot=True))
        self.assertTrue(result.success)


if __name__ == "__main__":
    unittest.main()
