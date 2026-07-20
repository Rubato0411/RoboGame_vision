from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.competition_controller import CompetitionController, RobotFeedback  # noqa: E402
from src.vision_output import (GripperAlignmentOutput, PlacementOutput,
                               SelectedTargetOutput, StreamOutput,
                               VisionOutput)  # noqa: E402


def synthetic_vision(timestamp: float, *, target=False, aligned=False,
                     placement=False) -> VisionOutput:
    return VisionOutput(
        int(timestamp * 10), timestamp, "competition-simulation", "DEBUG_ALL",
        StreamOutput("HEALTHY", True, 30.0, 0.0, 0, "simulated"),
        selected_target=SelectedTargetOutput(
            target, 1 if target else None, .9 if target else 0.0, "simulated"),
        gripper_alignment=GripperAlignmentOutput(
            target, 1 if target else None, 0.0, 0.0, 0.0,
            aligned, .9 if target else 0.0, False, "simulated"),
        placement=(PlacementOutput(
            True, "building_1_layer_1", 1, (0.5, 0.0, 0.05),
            (0.0, 0.0, 0.0), 1, "simulated") if placement else PlacementOutput()),
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run one hardware-free competition-controller cycle")
    parser.add_argument("--rules", default=str(ROOT / "configs" / "competition_rules.json"))
    parser.add_argument("--strategy", default=str(ROOT / "configs" / "competition_strategy.json"))
    parser.add_argument("--compact", action="store_true", help="Print one compact JSON object per step")
    args = parser.parse_args()

    controller = CompetitionController.from_json(args.rules, args.strategy)
    steps = [
        ("start", 0.0, None, RobotFeedback(start_signal=True)),
        ("arrive_material", 0.1, None, RobotFeedback(at_material_zone=True)),
        ("target_aligned", 0.2, synthetic_vision(.2, target=True, aligned=True), RobotFeedback()),
        ("grasp_confirmed", 0.3, None, RobotFeedback(grasp_confirmed=True)),
        ("arrive_build", 0.4, None, RobotFeedback(at_build_zone=True)),
        ("placement_visible", 0.5, synthetic_vision(.5, placement=True), RobotFeedback()),
        ("place_pose_reached", 0.6, None, RobotFeedback(place_pose_reached=True)),
        ("released", 0.7, None, RobotFeedback(
            release_confirmed=True, target_in_slot=True, structure_stable=True)),
        ("stable_three_seconds", 3.7, None, RobotFeedback(structure_stable=True)),
    ]
    for name, now, vision, feedback in steps:
        decision = controller.step(vision, feedback, now)
        value = {"step": name, **decision.to_dict()}
        print(json.dumps(value, ensure_ascii=False,
                         separators=(",", ":") if args.compact else None,
                         indent=None if args.compact else 2))
    print(json.dumps({
        "placed_colors": controller.placed_colors,
        "occupied_slot_ids": sorted(controller.occupied_slot_ids),
    }, ensure_ascii=False, indent=None if args.compact else 2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

