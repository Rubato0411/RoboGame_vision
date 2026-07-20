from __future__ import annotations

import argparse
from collections import deque
from datetime import datetime
from pathlib import Path
from time import monotonic
from typing import Optional

import cv2

from src.image_source import CameraConfig, ImageSource


def parse_source(value: str):
    return int(value) if value.isdigit() else value


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Robust image/video/UVC camera viewer")
    parser.add_argument("--source", default="0", help="Camera index or image/video path")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--backend", choices=["auto", "dshow", "msmf", "v4l2"], default="auto")
    parser.add_argument("--fourcc", default="MJPG", help="Camera format, commonly MJPG or YUYV")
    parser.add_argument("--buffer-size", type=int, default=1)
    parser.add_argument("--warmup-frames", type=int, default=10)
    parser.add_argument("--open-retries", type=int, default=3)
    parser.add_argument("--max-read-failures", type=int, default=5)
    parser.add_argument("--reconnect-retries", type=int, default=5)
    parser.add_argument("--reconnect-delay", type=float, default=0.5)
    parser.add_argument("--exposure", type=float)
    parser.add_argument("--auto-exposure", type=float)
    parser.add_argument("--gain", type=float)
    parser.add_argument("--brightness", type=float)
    parser.add_argument("--contrast", type=float)
    parser.add_argument("--saturation", type=float)
    parser.add_argument("--sharpness", type=float)
    parser.add_argument("--white-balance", type=float)
    parser.add_argument("--auto-white-balance", type=float)
    parser.add_argument("--focus", type=float)
    parser.add_argument("--auto-focus", type=float)
    parser.add_argument("--loop", action="store_true")
    parser.add_argument("--record", action="store_true")
    return parser


def timestamped_path(folder: Path, prefix: str, suffix: str) -> Path:
    folder.mkdir(parents=True, exist_ok=True)
    return folder / f"{prefix}_{datetime.now():%Y%m%d_%H%M%S_%f}{suffix}"


def create_writer(frame, fps: float) -> tuple[cv2.VideoWriter, Path]:
    height, width = frame.shape[:2]
    path = timestamped_path(Path("recordings"), "video", ".mp4")
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), max(fps, 1.0), (width, height))
    if not writer.isOpened():
        raise RuntimeError("Could not create MP4 recording")
    return writer, path


def draw_status(frame, packet, display_fps: float, paused: bool, recording: bool, age_ms: float):
    lines = [
        f"Source: {packet.source_name}",
        f"Frame: {packet.frame_id}  FPS: {display_fps:.1f}  Age: {age_ms:.1f} ms",
        f"Reconnects: {packet.reconnect_count}",
        f"State: {'PAUSED' if paused else 'RUNNING'}  Recording: {'ON' if recording else 'OFF'}",
        "Q/Esc quit | SPACE pause | S screenshot | R record | P properties",
    ]
    for index, line in enumerate(lines, start=1):
        origin = (12, 28 * index)
        cv2.putText(frame, line, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(frame, line, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1, cv2.LINE_AA)


def main() -> int:
    args = build_parser().parse_args()
    config = CameraConfig(
        width=args.width, height=args.height, fps=args.fps, backend=args.backend,
        fourcc=args.fourcc, buffer_size=args.buffer_size, warmup_frames=args.warmup_frames,
        open_retries=args.open_retries, max_read_failures=args.max_read_failures,
        reconnect_retries=args.reconnect_retries, reconnect_delay=args.reconnect_delay,
        exposure=args.exposure, auto_exposure=args.auto_exposure, gain=args.gain,
        brightness=args.brightness, contrast=args.contrast, saturation=args.saturation,
        sharpness=args.sharpness, white_balance=args.white_balance,
        auto_white_balance=args.auto_white_balance, focus=args.focus, auto_focus=args.auto_focus,
    )
    paused = False
    recording_requested = args.record
    writer: Optional[cv2.VideoWriter] = None
    last_frame = last_packet = None
    frame_times: deque[float] = deque(maxlen=30)

    try:
        with ImageSource(parse_source(args.source), camera_config=config, loop_video=args.loop) as source:
            props = source.properties()
            print("Opened source with actual properties:")
            for name, value in props.items():
                print(f"  {name}: {value}")

            while True:
                if not paused or last_frame is None:
                    packet = source.read()
                    if packet is None:
                        print("ERROR: input ended or camera recovery failed")
                        break
                    last_packet, last_frame = packet, packet.image
                    frame_times.append(monotonic())

                assert last_frame is not None and last_packet is not None
                elapsed = frame_times[-1] - frame_times[0] if len(frame_times) >= 2 else 0.0
                display_fps = (len(frame_times) - 1) / elapsed if elapsed > 0 else 0.0

                if recording_requested and writer is None:
                    writer, path = create_writer(last_frame, float(props.get("fps", 0)) or args.fps)
                    print(f"Recording started: {path.resolve()}")
                elif not recording_requested and writer is not None:
                    writer.release()
                    writer = None
                    print("Recording stopped")
                if writer is not None and not paused:
                    writer.write(last_frame)

                display = last_frame.copy()
                draw_status(display, last_packet, display_fps, paused, writer is not None,
                            (monotonic() - last_packet.timestamp) * 1000.0)
                cv2.imshow("RoboGame Camera Input", display)
                key = cv2.waitKey(0 if source.is_still_image else 1) & 0xFF
                if key in (ord("q"), ord("Q"), 27):
                    break
                if key == ord(" "):
                    paused = not paused
                elif key in (ord("s"), ord("S")):
                    path = timestamped_path(Path("captures"), "frame", ".jpg")
                    cv2.imwrite(str(path), last_frame)
                    print(f"Screenshot saved: {path.resolve()}")
                elif key in (ord("r"), ord("R")):
                    recording_requested = not recording_requested
                elif key in (ord("p"), ord("P")):
                    print("Current properties:")
                    for name, value in source.properties().items():
                        print(f"  {name}: {value}")
    except (FileNotFoundError, RuntimeError, ValueError) as exc:
        print(f"ERROR: {exc}")
        return 1
    finally:
        if writer is not None:
            writer.release()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
