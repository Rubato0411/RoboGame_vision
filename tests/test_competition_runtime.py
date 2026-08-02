from pathlib import Path
import sys
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.competition_controller import (CompetitionController, GripperPoseFeedback,
                                        RobotFeedback)  # noqa: E402
from src.coordinate_transform import RigidTransform  # noqa: E402
from src.competition_rules import CompetitionRules  # noqa: E402
from src.competition_runtime import CompetitionRuntime  # noqa: E402
from src.image_source import FramePacket  # noqa: E402
from src.vision_output import (SelectedTargetOutput, StreamOutput,
                               VisionOutput)  # noqa: E402


class FakePipeline:
    def __init__(self):
        self.calls = []

    def process(self, packet, mode, desired_block_color=None, occupied_slot_ids=(),
                transform_robot_camera=None, requested_placement_slot_id=None):
        self.calls.append((mode, desired_block_color, occupied_slot_ids,
                           transform_robot_camera, requested_placement_slot_id))
        return VisionOutput(
            packet.frame_id, packet.timestamp, packet.source_name, mode.value,
            StreamOutput("HEALTHY", True, 30.0, 0.0, 0, "ok"),
            selected_target=SelectedTargetOutput(True, 1, .9, "test"),
        )


class CompetitionRuntimeTests(unittest.TestCase):
    def test_offline_simulation_must_be_explicit_and_passes_strategy_inputs(self):
        front_pipeline = FakePipeline()
        gripper_pipeline = FakePipeline()
        controller = CompetitionController(CompetitionRules())
        runtime = CompetitionRuntime(
            front_pipeline, controller, require_hardware_ready=False,
            gripper_pipeline=gripper_pipeline)
        runtime.update_feedback(
            RobotFeedback(start_signal=True, at_material_zone=True), 0.0)
        packet = FramePacket(np.zeros((2, 2, 3), np.uint8), 0.0, 0, "test")
        cycle = runtime.process(packet)
        self.assertEqual(front_pipeline.calls, [])
        self.assertEqual(gripper_pipeline.calls[0][1], "purple")
        self.assertEqual(gripper_pipeline.calls[0][2], ())
        self.assertEqual(cycle.output.selected_target.track_id, 1)

    def test_dynamic_camera_transform_uses_fresh_gripper_pose(self):
        hand_eye = RigidTransform.from_xyz_rpy([0.02, 0.0, 0.05], [0, 0, 0])
        runtime = CompetitionRuntime(
            FakePipeline(), CompetitionController(CompetitionRules()),
            require_hardware_ready=False, transform_gripper_camera=hand_eye,
            gripper_pose_timeout_s=0.1)
        feedback = RobotFeedback(gripper_pose=GripperPoseFeedback(
            True, 7, (0.3, -0.1, 0.4), (0.0, 0.0, 0.0)))
        runtime.update_feedback(feedback, 1.0)
        transform = runtime.current_transform_robot_camera(1.05)
        self.assertIsNotNone(transform)
        np.testing.assert_allclose(transform.translation, [0.32, -0.1, 0.45])
        self.assertIsNone(runtime.current_transform_robot_camera(1.11))

    def test_repeated_pose_sequence_does_not_refresh_stale_pose(self):
        runtime = CompetitionRuntime(
            FakePipeline(), CompetitionController(CompetitionRules()),
            require_hardware_ready=False,
            transform_gripper_camera=RigidTransform.identity(),
            gripper_pose_timeout_s=0.1)
        feedback = RobotFeedback(gripper_pose=GripperPoseFeedback(
            True, 9, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)))
        runtime.update_feedback(feedback, 1.0)
        changed_with_old_sequence = RobotFeedback(gripper_pose=GripperPoseFeedback(
            True, 9, (9.0, 9.0, 9.0), (0.0, 0.0, 0.0)))
        runtime.update_feedback(changed_with_old_sequence, 1.05)
        np.testing.assert_allclose(runtime.feedback.gripper_pose.translation_m,
                                   [0.0, 0.0, 0.0])
        self.assertIsNone(runtime.current_transform_robot_camera(1.2))

    def test_production_runtime_rejects_missing_hardware_registry(self):
        with self.assertRaises(RuntimeError):
            CompetitionRuntime(FakePipeline(), CompetitionController(CompetitionRules()))
