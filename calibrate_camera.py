from __future__ import annotations

import argparse
from pathlib import Path

import cv2

from src.camera_calibration import CameraCalibrator, ChessboardSpec


def find_images(folder: Path, recursive: bool) -> list[Path]:
    patterns = ("*.jpg", "*.jpeg", "*.png", "*.bmp", "*.tif", "*.tiff")
    paths = []
    for pattern in patterns:
        paths.extend(folder.rglob(pattern) if recursive else folder.glob(pattern))
    return sorted(set(paths))


def main() -> int:
    parser = argparse.ArgumentParser(description="Calibrate a camera from chessboard images")
    parser.add_argument("--input", required=True, help="Directory containing calibration images")
    parser.add_argument("--output", required=True, help="Output calibration JSON")
    parser.add_argument("--columns", type=int, default=9, help="Inner corner columns")
    parser.add_argument("--rows", type=int, default=6, help="Inner corner rows")
    parser.add_argument("--square-mm", type=float, default=25.0, help="Measured physical square side")
    parser.add_argument("--min-views", type=int, default=12)
    parser.add_argument("--max-view-error", type=float, default=1.5)
    parser.add_argument("--recursive", action="store_true")
    parser.add_argument("--preview-dir", help="Optional directory for corner-detection previews")
    args = parser.parse_args()

    folder = Path(args.input)
    if not folder.is_dir():
        print(f"ERROR: input directory does not exist: {folder}")
        return 1
    paths = find_images(folder, args.recursive)
    if not paths:
        print("ERROR: no calibration images found")
        return 1

    calibrator = CameraCalibrator(ChessboardSpec(args.columns, args.rows, args.square_mm / 1000.0))
    observations, failed = calibrator.collect_from_paths(paths)
    print(f"Images: total={len(paths)} corners_found={len(observations)} failed={len(failed)}")

    if args.preview_dir:
        preview_dir = Path(args.preview_dir)
        preview_dir.mkdir(parents=True, exist_ok=True)
        for index, observation in enumerate(observations):
            image = cv2.imread(observation.source_name)
            preview = calibrator.draw_detection(image, observation)
            cv2.imwrite(str(preview_dir / f"corners_{index:03d}.jpg"), preview)

    try:
        result = calibrator.calibrate(
            observations,
            min_views=args.min_views,
            max_view_error_px=args.max_view_error,
        )
    except ValueError as exc:
        print(f"ERROR: {exc}")
        return 1

    result.save_json(args.output)
    print(f"Saved: {Path(args.output).resolve()}")
    print(f"Resolution: {result.image_width}x{result.image_height}")
    print(f"RMS: {result.rms_error:.4f} px")
    print(f"Mean reprojection error: {result.mean_reprojection_error:.4f} px")
    print(f"Used views: {len(result.used_images)}  Rejected views: {len(result.rejected_images)}")
    for name, error in sorted(zip(result.used_images, result.per_view_errors), key=lambda item: item[1], reverse=True):
        print(f"  {error:.4f} px  {name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
