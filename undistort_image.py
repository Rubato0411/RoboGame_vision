from __future__ import annotations

import argparse
from pathlib import Path

import cv2

from src.camera_calibration import CalibrationResult, CameraCalibrator


def main() -> int:
    parser = argparse.ArgumentParser(description="Undistort one image with saved calibration")
    parser.add_argument("--calibration", required=True)
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--crop", action="store_true")
    args = parser.parse_args()
    image = cv2.imread(args.input)
    if image is None:
        print(f"ERROR: cannot read {args.input}")
        return 1
    result = CalibrationResult.load_json(args.calibration)
    try:
        corrected = CameraCalibrator.undistort(image, result, crop=args.crop)
    except ValueError as exc:
        print(f"ERROR: {exc}")
        return 1
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output), corrected)
    print(output.resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
