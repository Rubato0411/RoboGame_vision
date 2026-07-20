from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Iterable, Optional

import cv2
import numpy as np

from .camera_calibration import CalibrationResult


@dataclass
class AprilTagConfig:
    family: str = "tag36h11"
    allowed_ids: tuple[int, ...] = (1, 2, 3, 4, 5, 6)
    tag_size_m: float = 0.15
    upper_edge_height_m: float = 0.40
    adaptive_thresh_win_size_min: int = 3
    adaptive_thresh_win_size_max: int = 23
    adaptive_thresh_win_size_step: int = 10
    adaptive_thresh_constant: float = 7.0
    min_marker_perimeter_rate: float = 0.03
    max_marker_perimeter_rate: float = 4.0
    polygonal_approx_accuracy_rate: float = 0.03
    corner_refinement: str = "subpix"
    corner_refinement_win_size: int = 5
    corner_refinement_max_iterations: int = 30
    corner_refinement_min_accuracy: float = 0.01
    error_correction_rate: float = 0.6

    @classmethod
    def from_json(cls, path: str | Path) -> "AprilTagConfig":
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        data["allowed_ids"] = tuple(int(v) for v in data.get("allowed_ids", (1, 2, 3, 4, 5, 6)))
        return cls(**data)


@dataclass(frozen=True)
class AprilTagDetection:
    tag_id: int
    corners_px: tuple[tuple[float, float], ...]
    center_px: tuple[float, float]
    area_px: float
    perimeter_px: float
    rvec: Optional[np.ndarray] = None
    tvec_m: Optional[np.ndarray] = None
    distance_m: Optional[float] = None
    reprojection_error_px: Optional[float] = None


@dataclass(frozen=True)
class AprilTagResult:
    detections: tuple[AprilTagDetection, ...]
    rejected_candidates: int
    ignored_ids: tuple[int, ...]


class AprilTagDetector:
    """Detect tag36h11 markers and optionally estimate camera-relative pose."""

    DICTIONARIES = {"tag36h11": cv2.aruco.DICT_APRILTAG_36h11}

    def __init__(self, config: AprilTagConfig, calibration: CalibrationResult | None = None) -> None:
        if config.family not in self.DICTIONARIES:
            raise ValueError(f"Unsupported tag family: {config.family}")
        if config.tag_size_m <= 0:
            raise ValueError("tag_size_m must be positive")
        self.config = config
        self.calibration = calibration
        self.dictionary = cv2.aruco.getPredefinedDictionary(self.DICTIONARIES[config.family])
        parameters = cv2.aruco.DetectorParameters()
        parameters.adaptiveThreshWinSizeMin = config.adaptive_thresh_win_size_min
        parameters.adaptiveThreshWinSizeMax = config.adaptive_thresh_win_size_max
        parameters.adaptiveThreshWinSizeStep = config.adaptive_thresh_win_size_step
        parameters.adaptiveThreshConstant = config.adaptive_thresh_constant
        parameters.minMarkerPerimeterRate = config.min_marker_perimeter_rate
        parameters.maxMarkerPerimeterRate = config.max_marker_perimeter_rate
        parameters.polygonalApproxAccuracyRate = config.polygonal_approx_accuracy_rate
        parameters.cornerRefinementMethod = self._corner_refinement(config.corner_refinement)
        parameters.cornerRefinementWinSize = config.corner_refinement_win_size
        parameters.cornerRefinementMaxIterations = config.corner_refinement_max_iterations
        parameters.cornerRefinementMinAccuracy = config.corner_refinement_min_accuracy
        parameters.errorCorrectionRate = config.error_correction_rate
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, parameters)

    @classmethod
    def from_json(cls, config_path: str | Path,
                  calibration_path: str | Path | None = None) -> "AprilTagDetector":
        calibration = CalibrationResult.load_json(calibration_path) if calibration_path else None
        return cls(AprilTagConfig.from_json(config_path), calibration)

    @staticmethod
    def _corner_refinement(value: str) -> int:
        choices = {
            "none": cv2.aruco.CORNER_REFINE_NONE,
            "subpix": cv2.aruco.CORNER_REFINE_SUBPIX,
            "contour": cv2.aruco.CORNER_REFINE_CONTOUR,
            "apriltag": cv2.aruco.CORNER_REFINE_APRILTAG,
        }
        if value.lower() not in choices:
            raise ValueError(f"Unknown corner refinement method: {value}")
        return choices[value.lower()]

    def process(self, image_bgr: np.ndarray) -> AprilTagResult:
        if image_bgr is None or image_bgr.size == 0:
            raise ValueError("Input image is empty")
        if image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
            raise ValueError("Input must be a BGR image")
        gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.detector.detectMarkers(gray)
        if ids is None:
            return AprilTagResult((), len(rejected), ())

        allowed = set(self.config.allowed_ids)
        detections, ignored = [], []
        for marker_corners, raw_id in zip(corners, ids.reshape(-1)):
            tag_id = int(raw_id)
            if tag_id not in allowed:
                ignored.append(tag_id)
                continue
            points = marker_corners.reshape(4, 2).astype(np.float32)
            pose = self._estimate_pose(points, image_bgr.shape[:2])
            center = tuple(np.mean(points, axis=0).tolist())
            area = abs(float(cv2.contourArea(points)))
            perimeter = float(cv2.arcLength(points.reshape(-1, 1, 2), True))
            detections.append(AprilTagDetection(
                tag_id, tuple(map(tuple, points.tolist())), center, area, perimeter,
                *(pose if pose is not None else (None, None, None, None)),
            ))
        detections.sort(key=lambda item: item.tag_id)
        return AprilTagResult(tuple(detections), len(rejected), tuple(sorted(set(ignored))))

    def _estimate_pose(self, image_points: np.ndarray, image_shape: tuple[int, int]):
        if self.calibration is None:
            return None
        height, width = image_shape
        if (width, height) != (self.calibration.image_width, self.calibration.image_height):
            raise ValueError("Image resolution differs from calibration resolution")
        half = self.config.tag_size_m / 2.0
        object_points = np.array([
            [-half, half, 0], [half, half, 0],
            [half, -half, 0], [-half, -half, 0],
        ], dtype=np.float32)
        ok, rvec, tvec = cv2.solvePnP(
            object_points, image_points, self.calibration.camera_matrix,
            self.calibration.distortion_coefficients, flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not ok:
            return None
        projected, _ = cv2.projectPoints(object_points, rvec, tvec,
                                         self.calibration.camera_matrix,
                                         self.calibration.distortion_coefficients)
        error = float(np.sqrt(np.mean(np.sum((projected.reshape(-1, 2) - image_points) ** 2, axis=1))))
        vector = tvec.reshape(3).astype(np.float64)
        return rvec.reshape(3), vector, float(np.linalg.norm(vector)), error

    def annotate(self, image_bgr: np.ndarray, detections: Iterable[AprilTagDetection]) -> np.ndarray:
        output = image_bgr.copy()
        for item in detections:
            points = np.asarray(item.corners_px, np.int32)
            cv2.polylines(output, [points], True, (0, 255, 0), 3, cv2.LINE_AA)
            for index, point in enumerate(points):
                cv2.circle(output, tuple(point), 4, (0, 255, 255), -1)
                cv2.putText(output, str(index), tuple(point + (5, -5)),
                            cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 255, 255), 1, cv2.LINE_AA)
            cx, cy = map(lambda v: int(round(v)), item.center_px)
            label = f"ID={item.tag_id}"
            if item.distance_m is not None:
                label += f" d={item.distance_m:.2f}m err={item.reprojection_error_px:.2f}px"
            cv2.putText(output, label, (cx - 30, cy - 12), cv2.FONT_HERSHEY_SIMPLEX,
                        .62, (0, 255, 0), 2, cv2.LINE_AA)
            if item.rvec is not None:
                cv2.drawFrameAxes(output, self.calibration.camera_matrix,
                                  self.calibration.distortion_coefficients,
                                  item.rvec, item.tvec_m, self.config.tag_size_m * 0.5, 2)
        return output
