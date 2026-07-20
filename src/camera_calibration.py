from __future__ import annotations

from dataclasses import asdict, dataclass
import json
from pathlib import Path
from typing import Iterable, Optional, Sequence

import cv2
import numpy as np


@dataclass(frozen=True)
class ChessboardSpec:
    columns: int = 9
    rows: int = 6
    square_size_m: float = 0.025

    def validate(self) -> None:
        if self.columns < 2 or self.rows < 2:
            raise ValueError("Chessboard must have at least 2x2 inner corners")
        if self.square_size_m <= 0:
            raise ValueError("square_size_m must be positive")

    def object_points(self) -> np.ndarray:
        self.validate()
        points = np.zeros((self.rows * self.columns, 3), dtype=np.float32)
        grid = np.mgrid[0:self.columns, 0:self.rows].T.reshape(-1, 2)
        points[:, :2] = grid * self.square_size_m
        return points


@dataclass(frozen=True)
class CalibrationObservation:
    source_name: str
    image_size: tuple[int, int]
    image_points: np.ndarray


@dataclass(frozen=True)
class CalibrationResult:
    image_width: int
    image_height: int
    camera_matrix: np.ndarray
    distortion_coefficients: np.ndarray
    rms_error: float
    mean_reprojection_error: float
    per_view_errors: tuple[float, ...]
    used_images: tuple[str, ...]
    rejected_images: tuple[str, ...]
    board: ChessboardSpec

    def to_dict(self) -> dict:
        return {
            "format_version": 1,
            "image_width": self.image_width,
            "image_height": self.image_height,
            "camera_matrix": self.camera_matrix.tolist(),
            "distortion_coefficients": self.distortion_coefficients.reshape(-1).tolist(),
            "rms_error": self.rms_error,
            "mean_reprojection_error": self.mean_reprojection_error,
            "per_view_errors": list(self.per_view_errors),
            "used_images": list(self.used_images),
            "rejected_images": list(self.rejected_images),
            "board": asdict(self.board),
        }

    def save_json(self, path: str | Path) -> None:
        output = Path(path)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(self.to_dict(), ensure_ascii=False, indent=2), encoding="utf-8")

    @classmethod
    def load_json(cls, path: str | Path) -> "CalibrationResult":
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        return cls(
            image_width=int(data["image_width"]),
            image_height=int(data["image_height"]),
            camera_matrix=np.asarray(data["camera_matrix"], dtype=np.float64),
            distortion_coefficients=np.asarray(data["distortion_coefficients"], dtype=np.float64),
            rms_error=float(data["rms_error"]),
            mean_reprojection_error=float(data["mean_reprojection_error"]),
            per_view_errors=tuple(float(v) for v in data["per_view_errors"]),
            used_images=tuple(data["used_images"]),
            rejected_images=tuple(data.get("rejected_images", [])),
            board=ChessboardSpec(**data["board"]),
        )


class CameraCalibrator:
    def __init__(self, board: ChessboardSpec) -> None:
        board.validate()
        self.board = board
        self.criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            50,
            1e-4,
        )

    def detect_corners(
        self,
        image_bgr: np.ndarray,
        source_name: str = "frame",
    ) -> Optional[CalibrationObservation]:
        if image_bgr is None or image_bgr.size == 0:
            raise ValueError("Input image is empty")
        gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        pattern = (self.board.columns, self.board.rows)

        # SB is more robust on difficult views. Fall back for older builds.
        if hasattr(cv2, "findChessboardCornersSB"):
            found, corners = cv2.findChessboardCornersSB(
                gray,
                pattern,
                flags=cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE,
            )
        else:
            found, corners = cv2.findChessboardCorners(
                gray,
                pattern,
                flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE,
            )
        if not found or corners is None:
            return None

        corners = cv2.cornerSubPix(gray, corners.astype(np.float32), (5, 5), (-1, -1), self.criteria)
        return CalibrationObservation(
            source_name=source_name,
            image_size=(image_bgr.shape[1], image_bgr.shape[0]),
            image_points=corners.reshape(-1, 2).astype(np.float32),
        )

    def collect_from_paths(self, paths: Iterable[str | Path]) -> tuple[list[CalibrationObservation], list[str]]:
        observations, failed = [], []
        expected_size = None
        for value in paths:
            path = Path(value)
            image = cv2.imread(str(path), cv2.IMREAD_COLOR)
            if image is None:
                failed.append(str(path))
                continue
            observation = self.detect_corners(image, str(path))
            if observation is None:
                failed.append(str(path))
                continue
            if expected_size is None:
                expected_size = observation.image_size
            if observation.image_size != expected_size:
                failed.append(str(path))
                continue
            observations.append(observation)
        return observations, failed

    def calibrate(
        self,
        observations: Sequence[CalibrationObservation],
        min_views: int = 12,
        max_view_error_px: Optional[float] = 1.5,
        max_rejection_rounds: int = 3,
    ) -> CalibrationResult:
        if len(observations) < min_views:
            raise ValueError(f"Need at least {min_views} valid views, got {len(observations)}")
        image_size = observations[0].image_size
        if any(item.image_size != image_size for item in observations):
            raise ValueError("All calibration images must use the same resolution")

        active = list(observations)
        rejected_names: list[str] = []
        solution = None
        for _ in range(max_rejection_rounds + 1):
            solution = self._calibrate_once(active, image_size)
            _, _, _, _, per_view = solution
            if max_view_error_px is None or len(active) <= min_views:
                break
            worst_index = int(np.argmax(per_view))
            if per_view[worst_index] <= max_view_error_px:
                break
            rejected_names.append(active[worst_index].source_name)
            active.pop(worst_index)
            if len(active) < min_views:
                raise ValueError("Too many poor views were rejected; capture more calibration images")

        assert solution is not None
        rms, camera_matrix, distortion, _, per_view = solution
        return CalibrationResult(
            image_width=image_size[0],
            image_height=image_size[1],
            camera_matrix=camera_matrix,
            distortion_coefficients=distortion,
            rms_error=float(rms),
            mean_reprojection_error=float(np.mean(per_view)),
            per_view_errors=tuple(float(v) for v in per_view),
            used_images=tuple(item.source_name for item in active),
            rejected_images=tuple(rejected_names),
            board=self.board,
        )

    def _calibrate_once(self, observations, image_size):
        object_points = [self.board.object_points() for _ in observations]
        image_points = [item.image_points.reshape(-1, 1, 2) for item in observations]
        rms, matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
            object_points, image_points, image_size, None, None
        )
        errors = []
        for object_view, image_view, rvec, tvec in zip(object_points, image_points, rvecs, tvecs):
            projected, _ = cv2.projectPoints(object_view, rvec, tvec, matrix, distortion)
            error = cv2.norm(image_view, projected, cv2.NORM_L2) / len(projected) ** 0.5
            errors.append(float(error))
        return rms, matrix, distortion, (rvecs, tvecs), errors

    @staticmethod
    def undistort(image_bgr: np.ndarray, result: CalibrationResult, crop: bool = False) -> np.ndarray:
        height, width = image_bgr.shape[:2]
        if (width, height) != (result.image_width, result.image_height):
            raise ValueError("Image resolution differs from calibration resolution")
        new_matrix, roi = cv2.getOptimalNewCameraMatrix(
            result.camera_matrix, result.distortion_coefficients, (width, height), 1.0, (width, height)
        )
        output = cv2.undistort(image_bgr, result.camera_matrix,
                               result.distortion_coefficients, None, new_matrix)
        if crop:
            x, y, roi_width, roi_height = roi
            output = output[y:y + roi_height, x:x + roi_width]
        return output

    def draw_detection(self, image_bgr: np.ndarray, observation: CalibrationObservation) -> np.ndarray:
        output = image_bgr.copy()
        cv2.drawChessboardCorners(
            output,
            (self.board.columns, self.board.rows),
            observation.image_points.reshape(-1, 1, 2),
            True,
        )
        return output
