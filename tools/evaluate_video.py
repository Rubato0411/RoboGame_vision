from __future__ import annotations

import argparse
from collections import Counter
from pathlib import Path
import sys

import cv2

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.apriltag_detector import AprilTagDetector
from src.block_detector_robust import BlockDetector
from src.image_source import FramePacket
from src.stream_health import StreamHealthConfig, StreamHealthMonitor
from src.temporal_tracker import (TemporalObjectTracker, TemporalTrackerConfig,
                                  observations_from_apriltags, observations_from_blocks)


def main() -> int:
    parser = argparse.ArgumentParser(description="Offline full-pipeline video evaluator")
    parser.add_argument("--source", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--sample-step", type=int, default=1)
    args = parser.parse_args()

    capture = cv2.VideoCapture(args.source)
    if not capture.isOpened():
        raise RuntimeError(f"Cannot open video: {args.source}")
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = capture.get(cv2.CAP_PROP_FPS) or 30.0
    writer = cv2.VideoWriter(args.output, cv2.VideoWriter_fourcc(*"mp4v"),
                             fps / args.sample_step, (width, height))
    if not writer.isOpened():
        raise RuntimeError(f"Cannot create output: {args.output}")

    blocks = BlockDetector.from_json(ROOT / "configs" / "block_detector_robust.json")
    tags = AprilTagDetector.from_json(ROOT / "configs" / "apriltag_detector.json")
    tracker = TemporalObjectTracker(TemporalTrackerConfig())
    health_monitor = StreamHealthMonitor(StreamHealthConfig(expected_fps=fps))
    frame_count = decode_failures = 0
    raw_blocks = Counter()
    raw_tags = Counter()
    stable_categories = Counter()
    health_states = Counter()
    block_rejections = Counter()

    try:
        while True:
            ok, image = capture.read()
            if not ok:
                break
            frame_id = frame_count
            frame_count += 1
            if frame_id % args.sample_step:
                continue
            timestamp = frame_id / fps
            packet = FramePacket(image, timestamp, frame_id, args.source)
            health = health_monitor.observe(packet, now=timestamp)
            block_result = blocks.process(image)
            tag_result = tags.process(image)
            observations = (observations_from_blocks(block_result.detections) +
                            observations_from_apriltags(tag_result.detections))
            temporal = tracker.update(observations)

            health_states[health.status.value] += 1
            block_rejections.update(block_result.rejection_counts)
            raw_blocks.update(item.class_name for item in block_result.detections)
            raw_tags.update(item.tag_id for item in tag_result.detections)
            stable_categories.update(item.category for item in temporal.tracks if not item.predicted)

            annotated = blocks.annotate(image, block_result.detections, draw_roi=False)
            annotated = tags.annotate(annotated, tag_result.detections)
            cv2.putText(annotated,
                        f"frame={frame_id} blocks={len(block_result.detections)} "
                        f"tags={len(tag_result.detections)} stable={len(temporal.tracks)}",
                        (12, 28), cv2.FONT_HERSHEY_SIMPLEX, .60, (0, 255, 0), 2)
            cv2.putText(annotated, f"stream={health.status.value} fps={health.fps:.1f}",
                        (12, 55), cv2.FONT_HERSHEY_SIMPLEX, .56,
                        (0, 200, 0) if health.healthy else (0, 0, 255), 2)
            writer.write(annotated)
    finally:
        capture.release()
        writer.release()

    print(f"frames={frame_count} decode_failures={decode_failures} fps={fps:.3f} size={width}x{height}")
    print(f"raw_block_frame_hits={dict(raw_blocks)}")
    print(f"raw_tag_frame_hits={dict(raw_tags)}")
    print(f"stable_track_frame_hits={dict(stable_categories)}")
    print(f"stream_states={dict(health_states)}")
    print(f"block_rejections={dict(block_rejections)}")
    print(f"output={Path(args.output).resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
