from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.manipulation_verifier import (ManipulationEvidence, ManipulationPhase,
                                       ManipulationVerifier)  # noqa: E402


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Evaluate grasp/release/build evidence without robot hardware")
    parser.add_argument("--phase", required=True,
                        choices=[item.value for item in ManipulationPhase])
    parser.add_argument("--gripper-closed", action="store_true")
    parser.add_argument("--gripper-open", action="store_true")
    parser.add_argument("--contact", action="store_true")
    parser.add_argument("--pressure", type=float)
    parser.add_argument("--target-visible", action="store_true")
    parser.add_argument("--target-moved-with-gripper", action="store_true")
    parser.add_argument("--target-in-slot", action="store_true")
    parser.add_argument("--structure-stable", action="store_true")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    evidence = ManipulationEvidence(
        gripper_closed=args.gripper_closed,
        gripper_open=args.gripper_open,
        contact_detected=args.contact,
        pressure_value=args.pressure,
        target_visible=args.target_visible,
        target_moved_with_gripper=args.target_moved_with_gripper,
        target_in_placement_slot=args.target_in_slot,
        structure_stable=args.structure_stable,
    )
    result = ManipulationVerifier().verify(args.phase, evidence)
    print(json.dumps({
        "valid": result.valid,
        "success": result.success,
        "phase": result.phase,
        "confidence": result.confidence,
        "reason": result.reason,
    }, ensure_ascii=False, indent=2))
    return 0 if result.success else 2


if __name__ == "__main__":
    raise SystemExit(main())

