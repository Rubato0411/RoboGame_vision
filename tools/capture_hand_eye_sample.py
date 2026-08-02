from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from time import sleep

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.apriltag_detector import AprilTagDetector  # noqa: E402
from src.coordinate_transform import RigidTransform, rpy_from_rotation  # noqa: E402
from src.image_source import CameraConfig, ImageSource  # noqa: E402


def parse_source(value: str):
    return int(value) if value.isdigit() else value


def average_rotation(rotations: list[np.ndarray]) -> np.ndarray:
    matrix = sum(rotations)
    u, _, vt = np.linalg.svd(matrix)
    result = u @ vt
    if np.linalg.det(result) < 0:
        u[:, -1] *= -1
        result = u @ vt
    return result


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Append one stationary eye-in-hand pose pair to a sample JSON")
    parser.add_argument("--source", default="0", help="Gripper camera index/path")
    parser.add_argument("--backend", choices=["auto", "dshow", "msmf", "v4l2", "picamera2"],
                        default="picamera2")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--calibration", required=True, help="Gripper intrinsic JSON")
    parser.add_argument("--tag-config", default=str(ROOT / "configs" / "apriltag_detector.json"))
    parser.add_argument("--tag-id", required=True, type=int, help="Fixed calibration target tag ID")
    parser.add_argument("--base-gripper-xyz", required=True, nargs=3, type=float, metavar=("X", "Y", "Z"),
                        help="T_base_gripper translation in metres")
    parser.add_argument("--base-gripper-rpy", required=True, nargs=3, type=float,
                        metavar=("ROLL", "PITCH", "YAW"), help="T_base_gripper RPY in degrees")
    parser.add_argument("--samples", required=True, help="Sample JSON to create or append")
    parser.add_argument("--sample-id", help="Unique sample label")
    parser.add_argument("--frames", type=int, default=30, help="Valid detections to average")
    parser.add_argument("--warmup-frames", type=int, default=15)
    parser.add_argument("--max-attempts", type=int, default=300)
    args = parser.parse_args()
    if min(args.width, args.height, args.fps, args.frames, args.max_attempts) <= 0:
        parser.error("dimensions, fps, frames and max-attempts must be positive")
    if args.warmup_frames < 0 or args.max_attempts < args.frames:
        parser.error("warmup-frames must be non-negative and max-attempts >= frames")

    # Validate the robot transform before touching the sample file.
    base_from_gripper = RigidTransform.from_xyz_rpy(
        args.base_gripper_xyz, args.base_gripper_rpy)
    detector = AprilTagDetector.from_json(args.tag_config, args.calibration)
    camera = CameraConfig(width=args.width, height=args.height, fps=args.fps,
                          backend=args.backend, buffer_size=1)
    transforms = []
    with ImageSource(parse_source(args.source), camera_config=camera) as source:
        for _ in range(args.warmup_frames):
            if source.read() is None:
                raise RuntimeError("camera ended during warm-up")
        attempts = 0
        while len(transforms) < args.frames and attempts < args.max_attempts:
            attempts += 1
            packet = source.read()
            if packet is None:
                break
            matches = [item for item in detector.process(packet.image).detections
                       if item.tag_id == args.tag_id and item.rvec is not None and item.tvec_m is not None]
            if len(matches) == 1:
                transforms.append(RigidTransform.from_rvec_tvec(
                    matches[0].rvec, matches[0].tvec_m))
            sleep(0.001)
    if len(transforms) < args.frames:
        raise RuntimeError(
            f"only {len(transforms)}/{args.frames} valid detections after {attempts} attempts")

    rotation = average_rotation([item.rotation for item in transforms])
    translation = np.median(np.stack([item.translation for item in transforms]), axis=0)
    camera_from_target = RigidTransform(rotation, translation)
    rvec, _ = cv2.Rodrigues(camera_from_target.rotation)

    output = Path(args.samples)
    if output.exists():
        raw = json.loads(output.read_text(encoding="utf-8"))
        if raw.get("mode") != "eye_in_hand" or raw.get("length_unit", "m") != "m":
            raise ValueError("existing sample file is not an eye_in_hand metre file")
    else:
        raw = {
            "format_version": 1,
            "mode": "eye_in_hand",
            "length_unit": "m",
            "rotation_convention": "rpy_deg uses Rz(yaw) @ Ry(pitch) @ Rx(roll)",
            "target": {"type": "fixed_apriltag", "tag_id": args.tag_id},
            "samples": [],
        }
    if raw.get("target", {}).get("tag_id") not in (None, args.tag_id):
        raise ValueError("all samples must observe the same fixed tag ID")
    raw.setdefault("target", {})["tag_id"] = args.tag_id
    sample_id = args.sample_id or f"pose_{len(raw.get('samples', [])) + 1:02d}"
    if any(str(item.get("id")) == sample_id for item in raw.get("samples", [])):
        raise ValueError(f"duplicate sample id: {sample_id}")
    raw.setdefault("samples", []).append({
        "id": sample_id,
        "base_from_gripper": {
            "translation_m": [float(v) for v in base_from_gripper.translation],
            "rpy_deg": list(rpy_from_rotation(base_from_gripper.rotation, degrees=True)),
        },
        "camera_from_target": {
            "rvec": [float(v) for v in rvec.reshape(3)],
            "tvec_m": [float(v) for v in camera_from_target.translation],
        },
        "capture_quality": {"valid_frames": len(transforms), "attempted_frames": attempts},
    })
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(raw, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"Saved sample {sample_id}: {output.resolve()}")
    print("camera_from_target tvec_m=", [round(float(v), 6) for v in translation])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
