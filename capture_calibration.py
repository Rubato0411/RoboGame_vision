from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path

import cv2

from src.camera_calibration import CameraCalibrator, ChessboardSpec
from src.image_source import CameraConfig, ImageSource


def main() -> int:
    parser = argparse.ArgumentParser(description="Capture chessboard images from a real camera")
    parser.add_argument("--source", default="0")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--columns", type=int, default=9)
    parser.add_argument("--rows", type=int, default=6)
    parser.add_argument("--square-mm", type=float, default=25.0)
    parser.add_argument("--backend", choices=["auto", "dshow", "msmf", "v4l2"], default="auto")
    parser.add_argument("--fourcc", default="MJPG")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=float, default=30)
    args = parser.parse_args()

    source_value = int(args.source) if args.source.isdigit() else args.source
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    calibrator = CameraCalibrator(ChessboardSpec(args.columns, args.rows, args.square_mm / 1000.0))
    config = CameraConfig(width=args.width, height=args.height, fps=args.fps,
                          backend=args.backend, fourcc=args.fourcc)
    saved = len(list(output_dir.glob("calib_*.jpg")))

    try:
        with ImageSource(source_value, camera_config=config) as source:
            while True:
                packet = source.read()
                if packet is None:
                    print("ERROR: camera stream ended")
                    return 1
                observation = calibrator.detect_corners(packet.image)
                display = packet.image.copy()
                if observation is not None:
                    display = calibrator.draw_detection(display, observation)
                status = "FOUND" if observation is not None else "NOT FOUND"
                cv2.putText(display, f"Corners: {status}  Saved: {saved}", (12, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            (0, 255, 0) if observation is not None else (0, 0, 255), 2)
                cv2.putText(display, "SPACE save | Q quit", (12, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
                cv2.imshow("Calibration capture", display)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), ord("Q"), 27):
                    break
                if key == ord(" "):
                    if observation is None:
                        print("Not saved: chessboard corners were not detected")
                        continue
                    path = output_dir / f"calib_{datetime.now():%Y%m%d_%H%M%S_%f}.jpg"
                    if cv2.imwrite(str(path), packet.image):
                        saved += 1
                        print(f"Saved {saved}: {path}")
    finally:
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
