from pathlib import Path
import sys
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.competition_controller import CompetitionController, RobotFeedback  # noqa: E402
from src.competition_rules import CompetitionRules  # noqa: E402
from src.competition_runtime import CompetitionRuntime  # noqa: E402
from src.image_source import FramePacket  # noqa: E402
from src.vision_output import (SelectedTargetOutput, StreamOutput,
                               VisionOutput)  # noqa: E402


class FakePipeline:
    def __init__(self):
        self.calls = []

    def process(self, packet, mode, desired_block_color=None, occupied_slot_ids=()):
        self.calls.append((mode, desired_block_color, occupied_slot_ids))
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
        self.assertEqual(gripper_pipeline.calls[0][1], "orange")
        self.assertEqual(gripper_pipeline.calls[0][2], ())
        self.assertEqual(cycle.output.selected_target.track_id, 1)

    def test_production_runtime_rejects_missing_hardware_registry(self):
        with self.assertRaises(RuntimeError):
            CompetitionRuntime(FakePipeline(), CompetitionController(CompetitionRules()))
