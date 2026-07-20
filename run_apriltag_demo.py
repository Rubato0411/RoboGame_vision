from __future__ import annotations

import argparse
from pathlib import Path
from time import monotonic

import cv2

from src.apriltag_detector import AprilTagDetector
from src.image_source import CameraConfig, ImageSource
from src.temporal_tracker import TemporalObjectTracker, observations_from_apriltags
from src.stream_health import StreamHealthMonitor, StreamStatus


ROOT = Path(__file__).resolve().parent
WINDOW_NAME = "AprilTag Detector"


def parse_source(value: str):
    return int(value) if value.isdigit() else value


def fit_for_display(image, max_width: int, max_height: int):
    """Resize only the preview while keeping detection at source resolution."""
    height, width = image.shape[:2]
    scale = min(max_width / width, max_height / height, 1.0)
    if scale >= 1.0:
        return image
    target = (max(1, int(round(width * scale))), max(1, int(round(height * scale))))
    return cv2.resize(image, target, interpolation=cv2.INTER_AREA)


def main() -> int:
    parser = argparse.ArgumentParser(description="AprilTag tag36h11 detector")
    parser.add_argument("--source", required=True, help="Image/video path or camera index")
    parser.add_argument("--config", default=str(ROOT / "configs" / "apriltag_detector.json"))
    parser.add_argument("--calibration", help="Camera calibration JSON; enables metric pose")
    parser.add_argument("--temporal-config",
                        default=str(ROOT / "configs" / "temporal_tracker.json"),
                        help="Continuous-frame tracker JSON; use an empty value to disable")
    parser.add_argument("--stream-health-config",
                        default=str(ROOT / "configs" / "stream_health.json"),
                        help="Detector-independent camera watchdog JSON")
    parser.add_argument("--backend", choices=["auto", "dshow", "msmf", "v4l2"], default="auto")
    parser.add_argument("--fourcc", default="MJPG")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--loop", action="store_true")
    parser.add_argument("--save", help="Save latest annotated frame")
    parser.add_argument("--no-display", action="store_true")
    parser.add_argument("--display-width", type=int, default=1280,
                        help="Maximum preview width; does not affect detection")
    parser.add_argument("--display-height", type=int, default=720,
                        help="Maximum preview height; does not affect detection")
    args = parser.parse_args()
    if args.display_width <= 0 or args.display_height <= 0:
        parser.error("display width and height must be positive")

    detector = AprilTagDetector.from_json(args.config, args.calibration)
    tracker = TemporalObjectTracker.from_json(args.temporal_config) if args.temporal_config else None
    stream_monitor = StreamHealthMonitor.from_json(args.stream_health_config)
    camera = CameraConfig(width=args.width, height=args.height, fps=args.fps,
                          backend=args.backend, fourcc=args.fourcc)
    latest = None
    try:
        with ImageSource(parse_source(args.source), camera_config=camera,
                         loop_video=args.loop) as source:
            while True:
                packet = source.read()
                if packet is None:
                    health = stream_monitor.observe_failure()
                    print(f"STREAM {health.status.value}: {health.reason}")
                    break
                health = stream_monitor.observe(packet)
                started = monotonic()
                result = detector.process(packet.image)
                unsafe_stream = health.status in {
                    StreamStatus.FROZEN, StreamStatus.STALE_FRAME,
                    StreamStatus.INVALID_FRAME, StreamStatus.TIMEOUT,
                    StreamStatus.DISCONNECTED,
                }
                if tracker and unsafe_stream:
                    tracker.reset()
                temporal = (tracker.update([] if unsafe_stream else
                                           observations_from_apriltags(result.detections))
                            if tracker and not source.is_still_image else None)
                display_detections = ([track.detection for track in temporal.tracks]
                                      if temporal is not None else result.detections)
                elapsed = (monotonic() - started) * 1000
                latest = detector.annotate(packet.image, display_detections)
                cv2.putText(latest, f"raw={len(result.detections)} stable={len(display_detections)} {elapsed:.1f}ms",
                            (12, 28), cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 255, 0), 2)
                health_color = ((0, 200, 0) if health.healthy else
                                (0, 180, 255) if health.status == StreamStatus.RECOVERING else (0, 0, 255))
                cv2.putText(latest, f"stream={health.status.value} fps={health.fps:.1f}",
                            (12, 56), cv2.FONT_HERSHEY_SIMPLEX, .60, health_color, 2)
                print(" ".join(
                    f"ID={d.tag_id} center=({d.center_px[0]:.1f},{d.center_px[1]:.1f})" +
                    (f" xyz={tuple(round(v, 3) for v in d.tvec_m)}m" if d.tvec_m is not None else "")
                    for d in display_detections
                ) or "No allowed AprilTag detected")
                if args.no_display:
                    if source.is_still_image:
                        break
                    continue
                preview = fit_for_display(latest, args.display_width, args.display_height)
                cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
                cv2.resizeWindow(WINDOW_NAME, preview.shape[1], preview.shape[0])
                cv2.imshow(WINDOW_NAME, preview)
                key = cv2.waitKey(0 if source.is_still_image else 1) & 0xFF
                if key in (27, ord("q"), ord("Q")):
                    break
                if key in (ord("s"), ord("S")):
                    output = Path(args.save or "apriltag_result.jpg")
                    cv2.imwrite(str(output), latest)
                    print(f"Saved: {output.resolve()}")
    finally:
        cv2.destroyAllWindows()
    if args.save and latest is not None:
        cv2.imwrite(str(args.save), latest)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
