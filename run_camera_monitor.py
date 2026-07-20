from __future__ import annotations

import argparse
import json
from pathlib import Path
from time import monotonic, sleep

import cv2
import numpy as np

from src.image_source import CameraConfig, ImageSource
from src.realtime_camera_monitor import CameraMonitorWorker, RealtimeCameraMonitor
from src.stream_health import StreamHealthMonitor


ROOT = Path(__file__).resolve().parent
WINDOW = "RoboGame Camera Monitor"


def parse_source(value: str):
    return int(value) if value.isdigit() else value


def render_camera(snapshot, panel_size=(640, 360)):
    width, height = panel_size
    if snapshot.packet is None:
        panel = np.full((height, width, 3), 35, np.uint8)
        cv2.putText(panel, "WAITING FOR FRAME", (45, height//2),
                    cv2.FONT_HERSHEY_SIMPLEX, .9, (0, 180, 255), 2, cv2.LINE_AA)
    else:
        panel = cv2.resize(snapshot.packet.image, panel_size, interpolation=cv2.INTER_AREA)
    healthy = snapshot.health.healthy
    color = (0, 220, 0) if healthy else (0, 0, 255)
    cv2.rectangle(panel, (0, 0), (width-1, height-1), color, 3)
    cv2.putText(panel, f"{snapshot.role.upper()}  {snapshot.health.status.value}",
                (12, 27), cv2.FONT_HERSHEY_SIMPLEX, .68, color, 2, cv2.LINE_AA)
    cv2.putText(panel, f"fps={snapshot.health.fps:.1f} age={snapshot.health.frame_age_s:.2f}s "
                f"reconnect={snapshot.health.reconnect_count}",
                (12, 54), cv2.FONT_HERSHEY_SIMPLEX, .50, color, 2, cv2.LINE_AA)
    if snapshot.error:
        cv2.putText(panel, snapshot.error[:80], (12, height-16),
                    cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 0, 255), 2, cv2.LINE_AA)
    return panel


def compose_dashboard(snapshots, panel_size=(640, 360)):
    panels = [render_camera(snapshot, panel_size) for snapshot in snapshots]
    if len(panels) == 1:
        return panels[0]
    return cv2.hconcat(panels)


def build_parser():
    parser = argparse.ArgumentParser(description="Realtime front/gripper camera monitor")
    parser.add_argument("--front", required=True, help="Front camera index/path")
    parser.add_argument("--gripper", help="Optional gripper camera index/path")
    parser.add_argument("--backend", choices=["auto", "dshow", "msmf", "v4l2"], default="auto")
    parser.add_argument("--fourcc", default="MJPG")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=float, default=30)
    parser.add_argument("--display-panel-width", type=int, default=640)
    parser.add_argument("--display-panel-height", type=int, default=360)
    parser.add_argument("--record", help="Record the composed dashboard MP4")
    parser.add_argument("--health-log", help="Write camera health as JSONL")
    parser.add_argument("--save-dir", default=str(ROOT / "captures" / "monitor"))
    parser.add_argument("--duration", type=float, default=0, help="Stop after N seconds; 0 means manual")
    parser.add_argument("--no-display", action="store_true")
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    if min(args.width, args.height, args.fps,
           args.display_panel_width, args.display_panel_height) <= 0 or args.duration < 0:
        parser.error("dimensions/fps must be positive and duration cannot be negative")
    if args.no_display and args.duration <= 0:
        parser.error("--no-display requires a positive --duration")

    camera = CameraConfig(width=args.width, height=args.height, fps=args.fps,
                          backend=args.backend, fourcc=args.fourcc, buffer_size=1)
    workers = []
    try:
        for role, source_value in (("front", args.front), ("gripper", args.gripper)):
            if source_value is None:
                continue
            source = ImageSource(parse_source(source_value), camera)
            health = StreamHealthMonitor.from_json(ROOT / "configs" / "stream_health.json")
            workers.append(CameraMonitorWorker(role, source, health))
    except Exception:
        for worker in workers:
            worker.source.release()
        raise

    save_dir = Path(args.save_dir)
    save_dir.mkdir(parents=True, exist_ok=True)
    log_file = open(args.health_log, "w", encoding="utf-8") if args.health_log else None
    writer = None
    started, last_log = monotonic(), 0.0
    try:
        with RealtimeCameraMonitor(workers) as monitor:
            while True:
                now = monotonic()
                snapshots = monitor.snapshots()
                dashboard = compose_dashboard(
                    snapshots, (args.display_panel_width, args.display_panel_height))
                if args.record and writer is None:
                    Path(args.record).parent.mkdir(parents=True, exist_ok=True)
                    writer = cv2.VideoWriter(args.record, cv2.VideoWriter_fourcc(*"mp4v"),
                                             args.fps, (dashboard.shape[1], dashboard.shape[0]))
                    if not writer.isOpened():
                        raise RuntimeError(f"could not create recording: {args.record}")
                if writer is not None:
                    writer.write(dashboard)
                if log_file and now-last_log >= 0.2:
                    row = {item.role: {
                        "status": item.health.status.value, "healthy": item.health.healthy,
                        "fps": item.health.fps, "frame_age_s": item.health.frame_age_s,
                        "reconnect_count": item.health.reconnect_count, "error": item.error,
                    } for item in snapshots}
                    log_file.write(json.dumps({"monotonic_s": now, "cameras": row},
                                              ensure_ascii=False) + "\n")
                    log_file.flush()
                    last_log = now
                if not args.no_display:
                    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
                    cv2.imshow(WINDOW, dashboard)
                    key = cv2.waitKey(1) & 0xff
                    if key in (27, ord("q"), ord("Q")):
                        break
                    if key in (ord("s"), ord("S")):
                        stamp = int(now*1000)
                        for item in snapshots:
                            if item.packet is not None:
                                cv2.imwrite(str(save_dir / f"{item.role}_{stamp}.jpg"),
                                            item.packet.image)
                else:
                    sleep(.01)
                if args.duration and now-started >= args.duration:
                    break
    finally:
        if writer is not None:
            writer.release()
        if log_file is not None:
            log_file.close()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
