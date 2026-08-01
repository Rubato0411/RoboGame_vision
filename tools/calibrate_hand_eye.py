from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.hand_eye_calibration import calibrate_eye_in_hand, load_hand_eye_samples  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Solve eye-in-hand calibration from paired robot and AprilTag poses")
    parser.add_argument("--samples", required=True, help="Paired hand-eye sample JSON")
    parser.add_argument("--output", required=True, help="Output T_gripper_camera JSON")
    parser.add_argument(
        "--method", choices=["TSAI", "PARK", "HORAUD", "ANDREFF", "DANIILIDIS"],
        default="PARK")
    args = parser.parse_args()

    samples = load_hand_eye_samples(args.samples)
    result = calibrate_eye_in_hand(samples, args.method)
    result.save_json(args.output)
    print(json.dumps(result.to_dict(), ensure_ascii=False, indent=2))
    print(f"Saved: {Path(args.output).resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
