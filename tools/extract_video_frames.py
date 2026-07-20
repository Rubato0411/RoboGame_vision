from __future__ import annotations

import argparse
import csv
from pathlib import Path

import cv2


def main() -> int:
    parser = argparse.ArgumentParser(description="Extract evenly spaced frames for manual annotation")
    parser.add_argument("--source", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--interval", type=float, default=0.5, help="Seconds between extracted frames")
    parser.add_argument("--start", type=float, default=0.0)
    parser.add_argument("--end", type=float, help="End time in seconds")
    args = parser.parse_args()
    if args.interval <= 0 or args.start < 0 or (args.end is not None and args.end <= args.start):
        parser.error("require interval > 0, start >= 0, and end > start")

    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    capture = cv2.VideoCapture(args.source)
    if not capture.isOpened():
        raise RuntimeError(f"Cannot open video: {args.source}")
    fps = capture.get(cv2.CAP_PROP_FPS) or 30.0
    frame_total = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    duration = frame_total / fps
    end = min(args.end if args.end is not None else duration, duration)
    rows = []
    timestamp = args.start
    while timestamp < end + 1e-9:
        frame_id = min(int(round(timestamp * fps)), max(frame_total - 1, 0))
        capture.set(cv2.CAP_PROP_POS_FRAMES, frame_id)
        ok, image = capture.read()
        if not ok:
            timestamp += args.interval
            continue
        name = f"frame_{frame_id:06d}_{timestamp:07.2f}s.jpg"
        cv2.imwrite(str(output / name), image)
        rows.append({
            "image": name, "frame_id": frame_id, "timestamp_s": f"{timestamp:.3f}",
            "split": "tune", "orange_gt": "", "purple_gt": "",
            "orange_tp": "", "orange_fp": "", "orange_fn": "",
            "purple_tp": "", "purple_fp": "", "purple_fn": "",
            "scene_notes": "",
        })
        timestamp += args.interval
    capture.release()
    manifest = output / "annotations.csv"
    with manifest.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=rows[0].keys() if rows else [])
        if rows:
            writer.writeheader()
            writer.writerows(rows)
    print(f"Extracted {len(rows)} frames to {output.resolve()}")
    print(f"Annotation manifest: {manifest.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
