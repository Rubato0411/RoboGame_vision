from __future__ import annotations

import argparse
from pathlib import Path
from time import monotonic

import cv2

from src.black_line_detector import BlackLineDetector
from src.image_source import CameraConfig, ImageSource


ROOT = Path(__file__).resolve().parent


def parse_source(value: str):
    return int(value) if value.isdigit() else value


def fit_display(image, max_width, max_height):
    height, width = image.shape[:2]
    scale = min(max_width/width, max_height/height, 1.0)
    return image if scale >= 1 else cv2.resize(
        image, (int(width*scale), int(height*scale)), interpolation=cv2.INTER_AREA)


def main() -> int:
    parser = argparse.ArgumentParser(description="Black floor-line detector")
    parser.add_argument("--source", required=True)
    parser.add_argument("--config", default=str(ROOT / "configs" / "black_line_detector.json"))
    parser.add_argument("--backend", choices=["auto", "dshow", "msmf", "v4l2", "picamera2"], default="auto")
    parser.add_argument("--fourcc", default="MJPG")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=float, default=30)
    parser.add_argument("--loop", action="store_true")
    parser.add_argument("--save")
    parser.add_argument("--no-display", action="store_true")
    parser.add_argument("--display-width", type=int, default=1280)
    parser.add_argument("--display-height", type=int, default=720)
    args = parser.parse_args()

    detector = BlackLineDetector.from_json(args.config)
    camera = CameraConfig(width=args.width, height=args.height, fps=args.fps,
                          backend=args.backend, fourcc=args.fourcc)
    latest = None
    with ImageSource(parse_source(args.source), camera, loop_video=args.loop) as source:
        while True:
            packet = source.read()
            if packet is None:
                break
            started = monotonic()
            result = detector.process(packet.image)
            latest = detector.annotate(packet.image, result)
            elapsed = (monotonic()-started)*1000
            cv2.putText(latest, f"{elapsed:.1f}ms", (12, 58),
                        cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 255, 0), 2)
            print((f"found={result.found} offset={result.lateral_offset_px} "
                   f"heading={result.heading_error_deg} confidence={result.confidence:.3f}"))
            if args.no_display:
                if source.is_still_image:
                    break
                continue
            preview = fit_display(latest, args.display_width, args.display_height)
            cv2.namedWindow("Black Line Detector", cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
            cv2.resizeWindow("Black Line Detector", preview.shape[1], preview.shape[0])
            cv2.imshow("Black Line Detector", preview)
            key = cv2.waitKey(0 if source.is_still_image else 1) & 0xff
            if key in (27, ord('q'), ord('Q')):
                break
    cv2.destroyAllWindows()
    if args.save and latest is not None:
        cv2.imwrite(args.save, latest)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
