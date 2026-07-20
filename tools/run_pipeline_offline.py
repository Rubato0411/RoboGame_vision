from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
import sys

import cv2

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.image_source import FramePacket  # noqa: E402
from src.vision_output import VisionMode  # noqa: E402
from src.vision_pipeline import VisionPipeline, VisionPipelinePaths  # noqa: E402


IMAGE_EXTENSIONS = {".bmp", ".jpg", ".jpeg", ".png", ".tif", ".tiff", ".webp"}


def main() -> int:
    parser = argparse.ArgumentParser(description="Offline VisionPipeline JSON tester")
    parser.add_argument("--source", required=True, help="Image or video path")
    parser.add_argument("--mode", choices=[item.value for item in VisionMode], default="DEBUG_ALL")
    parser.add_argument("--output", required=True, help="Output JSONL path")
    parser.add_argument("--max-frames", type=int, default=0, help="0 means all video frames")
    parser.add_argument("--sample-step", type=int, default=1)
    parser.add_argument("--repeat-image", type=int, default=3,
                        help="Repeat a still image to exercise temporal confirmation")
    parser.add_argument("--calibration")
    parser.add_argument("--coordinates")
    args = parser.parse_args()
    if args.sample_step < 1 or args.repeat_image < 1 or args.max_frames < 0:
        parser.error("sample-step/repeat-image must be >= 1 and max-frames >= 0")
    if bool(args.calibration) != bool(args.coordinates):
        parser.error("--calibration and --coordinates must be supplied together")

    source_path = Path(args.source)
    if not source_path.exists():
        raise FileNotFoundError(source_path)
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    pipeline = VisionPipeline.from_paths(
        VisionPipelinePaths.project_defaults(ROOT), args.calibration, args.coordinates)
    mode = VisionMode(args.mode)
    is_image = source_path.suffix.lower() in IMAGE_EXTENSIONS

    capture = None
    if is_image:
        still = cv2.imread(str(source_path), cv2.IMREAD_COLOR)
        if still is None:
            raise RuntimeError(f"Cannot decode image: {source_path}")
        fps, total = 30.0, args.repeat_image
    else:
        capture = cv2.VideoCapture(str(source_path))
        if not capture.isOpened():
            raise RuntimeError(f"Cannot open video: {source_path}")
        fps = capture.get(cv2.CAP_PROP_FPS) or 30.0
        total = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))

    statuses, tag_frames, block_frames, line_frames = Counter(), 0, 0, 0
    errors, timings, written, source_index = Counter(), [], 0, 0
    with output_path.open("w", encoding="utf-8") as handle:
        while True:
            if is_image:
                if source_index >= total:
                    break
                image, ok = still.copy(), True
            else:
                ok, image = capture.read()
                if not ok:
                    break
            current_index = source_index
            source_index += 1
            if current_index % args.sample_step:
                continue
            if args.max_frames and written >= args.max_frames:
                break
            packet = FramePacket(image, current_index/fps, current_index,
                                 f"offline:{source_path.name}")
            output = pipeline.process(packet, mode)
            handle.write(output.to_json() + "\n")
            written += 1
            statuses[output.stream.status] += 1
            tag_frames += bool(output.tags)
            block_frames += bool(output.blocks)
            line_frames += output.line.valid
            timings.append(output.processing.total_ms)
            errors.update(output.errors)
    if capture is not None:
        capture.release()

    summary = {
        "source": str(source_path.resolve()), "mode": mode.value,
        "processed_frames": written, "source_fps": fps,
        "stream_status_frames": dict(statuses),
        "frames_with_tags": tag_frames, "frames_with_blocks": block_frames,
        "frames_with_valid_line": line_frames,
        "mean_processing_ms": sum(timings)/len(timings) if timings else 0.0,
        "max_processing_ms": max(timings, default=0.0),
        "errors": dict(errors), "jsonl": str(output_path.resolve()),
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
