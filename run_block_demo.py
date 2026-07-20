from __future__ import annotations

import argparse
from pathlib import Path
from time import monotonic

import cv2

from src.block_detector_robust import BlockDetector
from src.image_source import CameraConfig, ImageSource


ROOT = Path(__file__).resolve().parent


def parse_source(value: str):
    return int(value) if value.isdigit() else value


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Orange/purple block detector demo")
    parser.add_argument("--source", required=True, help="Image/video path or camera index")
    parser.add_argument("--config", default=str(ROOT / "configs" / "block_detector_robust.json"))
    parser.add_argument("--backend", choices=["auto", "dshow", "msmf", "v4l2"], default="auto")
    parser.add_argument("--fourcc", default="MJPG")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--loop", action="store_true")
    parser.add_argument("--save", help="Save the latest annotated frame to this path")
    parser.add_argument("--no-display", action="store_true", help="Process without opening a GUI window")
    return parser


def make_panel(annotated, masks):
    orange = cv2.cvtColor(masks["orange"], cv2.COLOR_GRAY2BGR)
    purple = cv2.cvtColor(masks["purple"], cv2.COLOR_GRAY2BGR)
    target_height = annotated.shape[0] // 2
    target_width = annotated.shape[1] // 2
    orange = cv2.resize(orange, (target_width, target_height))
    purple = cv2.resize(purple, (target_width, target_height))
    masks_panel = cv2.hconcat([orange, purple])
    if masks_panel.shape[1] != annotated.shape[1]:
        masks_panel = cv2.resize(masks_panel, (annotated.shape[1], target_height))
    return cv2.vconcat([annotated, masks_panel])


def main() -> int:
    args = build_parser().parse_args()
    detector = BlockDetector.from_json(args.config)
    camera = CameraConfig(
        width=args.width,
        height=args.height,
        fps=args.fps,
        backend=args.backend,
        fourcc=args.fourcc,
    )
    last_annotated = None

    try:
        with ImageSource(parse_source(args.source), camera_config=camera, loop_video=args.loop) as source:
            while True:
                packet = source.read()
                if packet is None:
                    break
                start = monotonic()
                result = detector.process(packet.image)
                elapsed_ms = (monotonic() - start) * 1000.0
                annotated = detector.annotate(packet.image, result.detections)
                cv2.putText(
                    annotated,
                    f"blocks={len(result.detections)} process={elapsed_ms:.1f} ms",
                    (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2, cv2.LINE_AA,
                )
                rejection_summary = " ".join(
                    f"{name}:{count}" for name, count in sorted(result.rejection_counts.items())
                ) or "none"
                cv2.putText(
                    annotated,
                    f"rejected: {rejection_summary}",
                    (12, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 0), 1, cv2.LINE_AA,
                )
                rejection_summary = " ".join(
                    f"{name}:{count}" for name, count in sorted(result.rejection_counts.items())
                ) or "none"
                cv2.putText(
                    annotated,
                    f"rejected: {rejection_summary}",
                    (12, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 0), 1, cv2.LINE_AA,
                )
                last_annotated = annotated
                if args.no_display:
                    if source.is_still_image:
                        break
                    continue
                cv2.imshow("Block Detector: result / orange mask / purple mask", make_panel(annotated, result.masks))
                key = cv2.waitKey(0 if source.is_still_image else 1) & 0xFF
                if key in (ord("q"), ord("Q"), 27):
                    break
                if key in (ord("s"), ord("S")):
                    output = Path(args.save or "block_detection_result.jpg")
                    cv2.imwrite(str(output), annotated)
                    print(f"Saved: {output.resolve()}")
    except (FileNotFoundError, RuntimeError, ValueError) as exc:
        print(f"ERROR: {exc}")
        return 1
    finally:
        cv2.destroyAllWindows()

    if args.save and last_annotated is not None:
        cv2.imwrite(args.save, last_annotated)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
