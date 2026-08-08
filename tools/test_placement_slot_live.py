from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.image_source import CameraConfig, ImageSource  # noqa: E402
from src.placement_tag_locator import PlacementTagLocator  # noqa: E402
from src.vision_output import VisionMode  # noqa: E402
from src.vision_pipeline import VisionPipeline, VisionPipelinePaths  # noqa: E402


def parse_source(value: str):
    return int(value) if value.isdigit() else value


def configured_slot_ids(path: str | Path) -> tuple[str, ...]:
    return tuple(slot.slot_id for slot in PlacementTagLocator.from_json(path).slots)


def observation_payload(vision) -> dict:
    data = vision.to_dict()
    return {
        "frame_id": data["frame_id"],
        "stream": data["stream"],
        "slot": data["placement"],
        "tags": data["tags"],
        "errors": data["errors"],
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Observe one configured placement slot without commanding robot motion")
    parser.add_argument("--slot-id", required=True)
    parser.add_argument("--source", default="1", help="Front CSI index/path")
    parser.add_argument("--backend", choices=["auto", "v4l2", "picamera2"],
                        default="picamera2")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--calibration", default=str(ROOT / "configs" / "camera_front_calibration.json"))
    parser.add_argument("--coordinates", default=str(ROOT / "configs" / "coordinate_front.json"))
    parser.add_argument("--slots", default=str(ROOT / "configs" / "placement_slots.json"))
    parser.add_argument("--max-frames", type=int, default=0, help="0 means run until Ctrl+C")
    parser.add_argument("--jsonl", help="Optional copy of compact observations")
    parser.add_argument("--quiet", action="store_true")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if min(args.width, args.height, args.fps) <= 0 or args.max_frames < 0:
        raise SystemExit("camera values must be positive and max-frames non-negative")
    slot_ids = configured_slot_ids(args.slots)
    if args.slot_id not in slot_ids:
        raise SystemExit(
            f"slot is missing or not configured: {args.slot_id}; available={','.join(slot_ids)}")
    paths = VisionPipelinePaths.project_defaults(ROOT)
    paths = VisionPipelinePaths(
        paths.apriltag_config, paths.block_config, paths.line_config,
        paths.temporal_config, paths.stream_health_config,
        paths.gripper_alignment_config, Path(args.slots))
    pipeline = VisionPipeline.from_paths(paths, args.calibration, args.coordinates,
                                         require_field_tags=True)
    camera_config = CameraConfig(
        width=args.width, height=args.height, fps=args.fps,
        backend=args.backend, buffer_size=1)
    output_path = Path(args.jsonl) if args.jsonl else None
    if output_path is not None:
        output_path.parent.mkdir(parents=True, exist_ok=True)
    output_handle = output_path.open("w", encoding="utf-8") if output_path else None
    processed = 0
    print("COMMISSIONING ONLY: this tool observes a slot and sends no arm motion commands.")
    try:
        with ImageSource(parse_source(args.source), camera_config=camera_config) as camera:
            while not args.max_frames or processed < args.max_frames:
                packet = camera.read()
                if packet is None:
                    raise RuntimeError("front camera stopped producing frames")
                vision = pipeline.process(
                    packet, VisionMode.PLACE_ASSIST,
                    requested_placement_slot_id=args.slot_id)
                payload = observation_payload(vision)
                line = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
                if not args.quiet:
                    print(line, flush=True)
                if output_handle is not None:
                    output_handle.write(line + "\n")
                    output_handle.flush()
                processed += 1
    except KeyboardInterrupt:
        return 0
    finally:
        if output_handle is not None:
            output_handle.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
