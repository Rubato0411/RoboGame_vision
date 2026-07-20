from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2


ROOT = Path(__file__).resolve().parents[1]
import sys
sys.path.insert(0, str(ROOT))

from src.block_detector_robust import BlockDetector, BlockDetectorConfig  # noqa: E402
from src.image_source import CameraConfig, ImageSource  # noqa: E402


CONTROL_WINDOW = "Block detector controls"
RESULT_WINDOW = "Result | active mask | combined mask"


def noop(_value: int) -> None:
    pass


def odd(value: int) -> int:
    value = max(value, 1)
    return value if value % 2 == 1 else value + 1


def parse_source(value: str):
    return int(value) if value.isdigit() else value


def create_trackbar(name: str, value: int, maximum: int) -> None:
    cv2.createTrackbar(name, CONTROL_WINDOW, int(value), maximum, noop)


def set_trackbar(name: str, value: int) -> None:
    cv2.setTrackbarPos(name, CONTROL_WINDOW, int(value))


def get_trackbar(name: str) -> int:
    return cv2.getTrackbarPos(name, CONTROL_WINDOW)


def load_raw_config(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def create_controls(raw: dict, active_color: str) -> None:
    cv2.namedWindow(CONTROL_WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(CONTROL_WINDOW, 560, 760)
    rule = raw[active_color]
    create_trackbar("H min", rule["hsv_lower"][0], 179)
    create_trackbar("H max", rule["hsv_upper"][0], 179)
    create_trackbar("S min", rule["hsv_lower"][1], 255)
    create_trackbar("S max", rule["hsv_upper"][1], 255)
    create_trackbar("V min", rule["hsv_lower"][2], 255)
    create_trackbar("V max", rule["hsv_upper"][2], 255)
    create_trackbar("blur kernel", raw["blur_kernel"], 31)
    create_trackbar("morph kernel", raw["morph_kernel"], 31)
    create_trackbar("open iterations", raw["open_iterations"], 8)
    create_trackbar("close iterations", raw["close_iterations"], 8)
    create_trackbar("min area px", raw["min_area_px"], 30000)
    create_trackbar("max area %", round(raw["max_area_ratio"] * 100), 100)
    create_trackbar("min aspect x100", round(raw["min_aspect_ratio"] * 100), 300)
    create_trackbar("max aspect x100", round(raw["max_aspect_ratio"] * 100), 500)
    create_trackbar("min rectangular %", round(raw["min_rectangularity"] * 100), 100)
    create_trackbar("min solidity %", round(raw["min_solidity"] * 100), 100)


def load_color_controls(raw: dict, color: str) -> None:
    rule = raw[color]
    set_trackbar("H min", rule["hsv_lower"][0])
    set_trackbar("H max", rule["hsv_upper"][0])
    set_trackbar("S min", rule["hsv_lower"][1])
    set_trackbar("S max", rule["hsv_upper"][1])
    set_trackbar("V min", rule["hsv_lower"][2])
    set_trackbar("V max", rule["hsv_upper"][2])


def update_raw_from_controls(raw: dict, active_color: str) -> None:
    raw[active_color]["hsv_lower"] = [
        get_trackbar("H min"), get_trackbar("S min"), get_trackbar("V min")
    ]
    raw[active_color]["hsv_upper"] = [
        get_trackbar("H max"), get_trackbar("S max"), get_trackbar("V max")
    ]
    raw["blur_kernel"] = odd(get_trackbar("blur kernel"))
    raw["morph_kernel"] = odd(get_trackbar("morph kernel"))
    raw["open_iterations"] = get_trackbar("open iterations")
    raw["close_iterations"] = get_trackbar("close iterations")
    raw["min_area_px"] = max(get_trackbar("min area px"), 1)
    raw["max_area_ratio"] = max(get_trackbar("max area %") / 100.0, 0.01)
    raw["min_aspect_ratio"] = max(get_trackbar("min aspect x100") / 100.0, 0.01)
    raw["max_aspect_ratio"] = max(get_trackbar("max aspect x100") / 100.0, 0.01)
    raw["min_rectangularity"] = get_trackbar("min rectangular %") / 100.0
    raw["min_solidity"] = get_trackbar("min solidity %") / 100.0


def make_detector(raw: dict) -> BlockDetector:
    colors = {}
    for name in ("orange", "purple"):
        rule = raw[name]
        from src.block_detector_robust import ColorRule
        colors[name] = ColorRule(
            hsv_lower=tuple(rule["hsv_lower"]),
            hsv_upper=tuple(rule["hsv_upper"]),
            draw_color_bgr=tuple(rule["draw_color_bgr"]),
        )
    return BlockDetector(BlockDetectorConfig(
        colors=colors,
        blur_kernel=raw["blur_kernel"],
        morph_kernel=raw["morph_kernel"],
        open_iterations=raw["open_iterations"],
        close_iterations=raw["close_iterations"],
        min_area_px=raw["min_area_px"],
        max_area_ratio=raw["max_area_ratio"],
        min_aspect_ratio=raw["min_aspect_ratio"],
        max_aspect_ratio=raw["max_aspect_ratio"],
        min_rectangularity=raw["min_rectangularity"],
        min_rotated_rectangularity=raw.get("min_rotated_rectangularity", 0.60),
        min_solidity=raw["min_solidity"],
        min_color_coverage=raw.get("min_color_coverage", 0.55),
        roi_normalized=tuple(raw.get("roi_normalized", (0, 0, 1, 1))),
        gamma=raw.get("gamma", 1.0),
        clahe_enabled=raw.get("clahe_enabled", False),
        clahe_clip_limit=raw.get("clahe_clip_limit", 2.0),
        clahe_grid_size=raw.get("clahe_grid_size", 8),
        border_margin_px=raw.get("border_margin_px", 2),
        reject_border_touching=raw.get("reject_border_touching", False),
        nms_iou_threshold=raw.get("nms_iou_threshold", 0.35),
        split_touching_enabled=raw.get("split_touching_enabled", True),
        split_peak_ratio=raw.get("split_peak_ratio", 0.52),
        split_erosion_ratio=raw.get("split_erosion_ratio", 0.10),
        split_max_erosion_iterations=raw.get("split_max_erosion_iterations", 48),
        split_min_seed_area_px=raw.get("split_min_seed_area_px", 80),
        split_min_region_area_px=raw.get("split_min_region_area_px", 300),
    ))


def make_panel(annotated, active_mask, combined_mask, active_color: str):
    height, width = annotated.shape[:2]
    small_size = (width // 2, height // 2)
    active_bgr = cv2.cvtColor(active_mask, cv2.COLOR_GRAY2BGR)
    combined_bgr = cv2.cvtColor(combined_mask, cv2.COLOR_GRAY2BGR)
    active_bgr = cv2.resize(active_bgr, small_size)
    combined_bgr = cv2.resize(combined_bgr, small_size)
    cv2.putText(active_bgr, f"active mask: {active_color}", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.putText(combined_bgr, "combined mask", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    lower = cv2.hconcat([active_bgr, combined_bgr])
    if lower.shape[1] != width:
        lower = cv2.resize(lower, (width, height // 2))
    return cv2.vconcat([annotated, lower])


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Interactive block detector parameter tuner")
    parser.add_argument("--source", default=str(ROOT / "data" / "synthetic_blocks.jpg"))
    parser.add_argument("--config", default=str(ROOT / "configs" / "block_detector_robust.json"))
    parser.add_argument("--backend", choices=["auto", "dshow", "msmf", "v4l2"], default="auto")
    parser.add_argument("--fourcc", default="MJPG")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=float, default=30)
    parser.add_argument("--loop", action="store_true")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    config_path = Path(args.config)
    raw = load_raw_config(config_path)
    active_color = "orange"
    create_controls(raw, active_color)
    cv2.namedWindow(RESULT_WINDOW, cv2.WINDOW_NORMAL)
    camera = CameraConfig(width=args.width, height=args.height, fps=args.fps,
                          backend=args.backend, fourcc=args.fourcc)
    latest_frame = None

    try:
        with ImageSource(parse_source(args.source), camera_config=camera, loop_video=args.loop) as source:
            while True:
                if source.is_still_image or latest_frame is None:
                    packet = source.read()
                    if packet is None:
                        break
                    latest_frame = packet.image
                elif not source.is_still_image:
                    packet = source.read()
                    if packet is None:
                        break
                    latest_frame = packet.image

                update_raw_from_controls(raw, active_color)
                detector = make_detector(raw)
                result = detector.process(latest_frame)
                annotated = detector.annotate(latest_frame, result.detections)
                cv2.putText(annotated,
                            f"active={active_color} detections={len(result.detections)} | C switch W save R reload Q quit",
                            (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 255, 0), 2)
                panel = make_panel(annotated, result.masks[active_color],
                                   detector.combined_mask(result.masks), active_color)
                cv2.imshow(RESULT_WINDOW, panel)
                key = cv2.waitKey(20) & 0xFF
                if key in (ord("q"), ord("Q"), 27):
                    break
                if key in (ord("c"), ord("C")):
                    update_raw_from_controls(raw, active_color)
                    active_color = "purple" if active_color == "orange" else "orange"
                    load_color_controls(raw, active_color)
                    print(f"Active color: {active_color}")
                elif key in (ord("w"), ord("W")):
                    update_raw_from_controls(raw, active_color)
                    config_path.write_text(json.dumps(raw, ensure_ascii=False, indent=2), encoding="utf-8")
                    print(f"Saved: {config_path}")
                elif key in (ord("r"), ord("R")):
                    raw = load_raw_config(config_path)
                    load_color_controls(raw, active_color)
                    print("Reloaded saved configuration")
    finally:
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
