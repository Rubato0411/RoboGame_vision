from __future__ import annotations

import argparse
from contextlib import ExitStack
from pathlib import Path
from time import monotonic

import cv2
import numpy as np

from src.image_source import CameraConfig, ImageSource
from src.raspberry_pi_endpoint import RaspberryPiVisionEndpoint
from src.vision_output import VisionMode, VisionOutput
from src.vision_pipeline import VisionPipeline, VisionPipelinePaths


ROOT = Path(__file__).resolve().parent
WINDOW_NAME = "RoboGame Unified Vision"


def parse_source(value: str):
    return int(value) if value.isdigit() else value


def fit_for_display(image: np.ndarray, max_width: int, max_height: int) -> np.ndarray:
    height, width = image.shape[:2]
    scale = min(max_width/width, max_height/height, 1.0)
    if scale >= 1:
        return image
    return cv2.resize(image, (max(1, int(width*scale)), max(1, int(height*scale))),
                      interpolation=cv2.INTER_AREA)


def annotate_output(image: np.ndarray, output: VisionOutput) -> np.ndarray:
    canvas = image.copy()
    for tag in output.tags:
        points = np.asarray(tag.corners_px, np.int32)
        color = (0, 200, 0) if not tag.predicted else (0, 180, 255)
        cv2.polylines(canvas, [points], True, color, 2, cv2.LINE_AA)
        cx, cy = map(lambda value: int(round(value)), tag.center_px)
        cv2.putText(canvas, f"Tag {tag.tag_id} T{tag.track_id}" + (" PRED" if tag.predicted else ""),
                    (cx-35, cy-10), cv2.FONT_HERSHEY_SIMPLEX, .55, color, 2, cv2.LINE_AA)
    for block in output.blocks:
        x, y, width, height = block.bbox_px
        color = (0, 165, 255) if block.color == "orange" else (255, 0, 180)
        if block.predicted:
            color = (0, 180, 255)
        cv2.rectangle(canvas, (x, y), (x+width, y+height), color, 2)
        cv2.putText(canvas, f"{block.color} T{block.track_id}" + (" PRED" if block.predicted else ""),
                    (x, max(18, y-7)), cv2.FONT_HERSHEY_SIMPLEX, .50, color, 2, cv2.LINE_AA)

    status_color = (0, 220, 0) if output.stream.healthy else (0, 0, 255)
    lines = [
        f"mode={output.mode} frame={output.frame_id} valid={output.valid}",
        f"stream={output.stream.status} fps={output.stream.fps:.1f} total={output.processing.total_ms:.1f}ms",
        f"tags={len(output.tags)} blocks={len(output.blocks)} line={output.line.valid}",
    ]
    if output.line.valid:
        lines.append(f"line offset={output.line.lateral_offset_px:+.0f}px "
                     f"heading={output.line.heading_error_deg:+.1f}deg "
                     f"conf={output.line.confidence:.2f}")
    if output.robot_pose.valid:
        x, y, z = output.robot_pose.position_field_m
        _, _, yaw = output.robot_pose.rpy_field_deg
        lines.append(f"robot field=({x:.2f},{y:.2f},{z:.2f})m yaw={yaw:.1f}deg")
    for index, text in enumerate(lines):
        cv2.putText(canvas, text, (12, 26+index*27), cv2.FONT_HERSHEY_SIMPLEX,
                    .58, status_color if index < 2 else (0, 255, 255), 2, cv2.LINE_AA)
    if output.errors:
        cv2.putText(canvas, " | ".join(output.errors)[:150],
                    (12, canvas.shape[0]-16), cv2.FONT_HERSHEY_SIMPLEX,
                    .48, (0, 0, 255), 2, cv2.LINE_AA)
    return canvas


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Unified RoboGame vision system")
    parser.add_argument("--source", required=True, help="Image/video path or camera index")
    parser.add_argument("--mode", choices=[item.value for item in VisionMode], default="IDLE")
    parser.add_argument("--calibration", help="Camera intrinsic calibration JSON")
    parser.add_argument("--coordinates", help="Configured coordinate_frames JSON")
    parser.add_argument("--backend", choices=["auto", "dshow", "msmf", "v4l2"], default="auto")
    parser.add_argument("--fourcc", default="MJPG")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=float, default=30)
    parser.add_argument("--loop", action="store_true")
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--still-repeats", type=int, default=3,
                        help="Repeat still images for temporal confirmation")
    parser.add_argument("--jsonl", help="Write one VisionOutput JSON per line")
    parser.add_argument("--protocol-output", help="Write framed protocol bytes for transport tests")
    parser.add_argument("--record", help="Write annotated MP4")
    parser.add_argument("--save-last", help="Save the final annotated frame")
    parser.add_argument("--no-display", action="store_true")
    parser.add_argument("--quiet", action="store_true", help="Do not print every frame JSON")
    parser.add_argument("--display-width", type=int, default=1280)
    parser.add_argument("--display-height", type=int, default=720)
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    if bool(args.calibration) != bool(args.coordinates):
        parser.error("--calibration and --coordinates must be supplied together")
    positive = (args.width, args.height, args.fps, args.still_repeats,
                args.display_width, args.display_height)
    if any(value <= 0 for value in positive) or args.max_frames < 0:
        parser.error("dimensions/fps/repeats must be positive and max-frames cannot be negative")

    pipeline = VisionPipeline.from_paths(
        VisionPipelinePaths.project_defaults(ROOT), args.calibration, args.coordinates)
    endpoint = RaspberryPiVisionEndpoint(VisionMode(args.mode))
    camera = CameraConfig(width=args.width, height=args.height, fps=args.fps,
                          backend=args.backend, fourcc=args.fourcc)
    processed, latest, writer = 0, None, None

    try:
        with ExitStack() as stack:
            json_file = stack.enter_context(open(args.jsonl, "w", encoding="utf-8")) if args.jsonl else None
            protocol_file = stack.enter_context(open(args.protocol_output, "wb")) if args.protocol_output else None
            source = stack.enter_context(ImageSource(parse_source(args.source), camera, args.loop))
            while True:
                packet = source.read()
                if packet is None:
                    break
                output = pipeline.process(packet, endpoint.mode)
                if json_file:
                    json_file.write(output.to_json() + "\n")
                    json_file.flush()
                if protocol_file:
                    protocol_file.write(endpoint.encode_vision(output))
                    protocol_file.flush()
                latest = annotate_output(packet.image, output)
                if args.record and writer is None:
                    path = Path(args.record)
                    path.parent.mkdir(parents=True, exist_ok=True)
                    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"),
                                             args.fps, (latest.shape[1], latest.shape[0]))
                    if not writer.isOpened():
                        raise RuntimeError(f"could not create recording: {path}")
                if writer is not None:
                    writer.write(latest)

                processed += 1
                if not args.quiet:
                    print(output.to_json())
                if not args.no_display:
                    preview = fit_for_display(latest, args.display_width, args.display_height)
                    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
                    cv2.resizeWindow(WINDOW_NAME, preview.shape[1], preview.shape[0])
                    cv2.imshow(WINDOW_NAME, preview)
                    key = cv2.waitKey(0 if source.is_still_image and processed >= args.still_repeats else 1) & 0xff
                    if key in (27, ord("q"), ord("Q")):
                        break
                if args.max_frames and processed >= args.max_frames:
                    break
                if source.is_still_image and processed >= args.still_repeats:
                    break
    except (FileNotFoundError, RuntimeError, ValueError) as exc:
        print(f"ERROR: {exc}")
        return 1
    finally:
        if writer is not None:
            writer.release()
        cv2.destroyAllWindows()

    if args.save_last and latest is not None:
        Path(args.save_last).parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(args.save_last, latest)
    print(f"Completed frames={processed} mode={endpoint.mode.value}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
