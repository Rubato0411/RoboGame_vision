from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path

import cv2
import numpy as np


@dataclass
class BlackLineConfig:
    roi_normalized: tuple[float, float, float, float] = (0.0, 0.45, 1.0, 1.0)
    grayscale_max: int = 140
    saturation_max: int = 110
    blur_kernel: int = 5
    morph_kernel: int = 5
    open_iterations: int = 1
    close_iterations: int = 2
    scan_row_count: int = 11
    scan_top_ratio: float = 0.12
    scan_bottom_ratio: float = 0.92
    scan_band_height_px: int = 5
    min_line_width_px: int = 5
    max_line_width_ratio: float = 0.18
    max_center_step_ratio: float = 0.16
    min_valid_scan_rows: int = 4
    max_bottom_gap_ratio: float = 0.18
    intersection_width_ratio: float = 0.35
    center_reference_ratio: float = 0.5

    @classmethod
    def from_json(cls, path: str | Path) -> "BlackLineConfig":
        raw = json.loads(Path(path).read_text(encoding="utf-8"))
        raw["roi_normalized"] = tuple(raw.get("roi_normalized", (0, .45, 1, 1)))
        return cls(**raw)

    def validate(self):
        for name in ("blur_kernel", "morph_kernel"):
            value = int(getattr(self, name))
            if value <= 0 or value % 2 == 0:
                raise ValueError(f"{name} must be a positive odd number")
        x1, y1, x2, y2 = self.roi_normalized
        if not (0 <= x1 < x2 <= 1 and 0 <= y1 < y2 <= 1):
            raise ValueError("invalid normalized ROI")
        if not 0 <= self.grayscale_max <= 255:
            raise ValueError("grayscale_max must be in [0, 255]")
        if not 0 <= self.saturation_max <= 255:
            raise ValueError("saturation_max must be in [0, 255]")
        if self.scan_row_count < 2 or self.min_valid_scan_rows < 2:
            raise ValueError("scan row counts must be at least 2")


@dataclass(frozen=True)
class BlackLineDetection:
    found: bool
    center_px: tuple[float, float] | None
    lateral_offset_px: float | None
    lateral_offset_normalized: float | None
    heading_error_deg: float | None
    confidence: float
    centerline_points_px: tuple[tuple[float, float], ...]
    valid_scan_rows: int
    total_scan_rows: int
    intersection_detected: bool
    roi_px: tuple[int, int, int, int]
    mask: np.ndarray


class BlackLineDetector:
    """Track a dark floor line with linked horizontal scan bands."""

    def __init__(self, config: BlackLineConfig) -> None:
        config.validate()
        self.config = config

    @classmethod
    def from_json(cls, path: str | Path) -> "BlackLineDetector":
        return cls(BlackLineConfig.from_json(path))

    def _roi(self, image):
        height, width = image.shape[:2]
        x1, y1, x2, y2 = self.config.roi_normalized
        left, top = int(x1 * width), int(y1 * height)
        right, bottom = max(left + 1, int(x2 * width)), max(top + 1, int(y2 * height))
        return image[top:bottom, left:right], (left, top, right-left, bottom-top)

    def process(self, image_bgr: np.ndarray) -> BlackLineDetection:
        if image_bgr is None or image_bgr.size == 0:
            raise ValueError("input image is empty")
        if image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
            raise ValueError("input must be a BGR image")
        roi, roi_px = self._roi(image_bgr)
        ox, oy, width, height = roi_px
        blurred = cv2.GaussianBlur(roi, (self.config.blur_kernel, self.config.blur_kernel), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        # Black floor tape is dark and weakly saturated. Requiring both avoids
        # treating dark red/orange block faces as a black line.
        mask = cv2.inRange(hsv, np.array((0, 0, 0), np.uint8),
                           np.array((179, self.config.saturation_max,
                                     self.config.grayscale_max), np.uint8))
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT,
                                           (self.config.morph_kernel, self.config.morph_kernel))
        if self.config.open_iterations:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel,
                                    iterations=self.config.open_iterations)
        if self.config.close_iterations:
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel,
                                    iterations=self.config.close_iterations)

        rows = np.linspace(self.config.scan_bottom_ratio * (height-1),
                           self.config.scan_top_ratio * (height-1),
                           self.config.scan_row_count).astype(int)
        expected_x = width * self.config.center_reference_ratio
        max_width = width * self.config.max_line_width_ratio
        max_step = width * self.config.max_center_step_ratio
        points = []
        wide_rows = 0
        previous_x = None
        half_band = max(0, self.config.scan_band_height_px // 2)

        for row in rows:
            y1, y2 = max(0, row-half_band), min(height, row+half_band+1)
            profile = np.max(mask[y1:y2], axis=0) > 0
            runs = self._runs(profile)
            if any((end-start) >= width*self.config.intersection_width_ratio for start, end in runs):
                wide_rows += 1
            candidates = [(start, end) for start, end in runs
                          if self.config.min_line_width_px <= end-start <= max_width]
            if not candidates:
                continue
            target = expected_x if previous_x is None else previous_x
            start, end = min(candidates, key=lambda run: abs((run[0]+run[1]-1)/2-target))
            center_x = (start + end - 1) / 2.0
            if previous_x is not None and abs(center_x - previous_x) > max_step:
                continue
            points.append((center_x + ox, float(row + oy)))
            previous_x = center_x

        valid = len(points)
        bottom_gap = (oy + self.config.scan_bottom_ratio*(height-1) -
                      max((point[1] for point in points), default=float("-inf")))
        found = (valid >= self.config.min_valid_scan_rows and
                 bottom_gap <= height*self.config.max_bottom_gap_ratio)
        if not found:
            return BlackLineDetection(False, None, None, None, None, 0.0,
                                      tuple(points), valid, len(rows), wide_rows > 0,
                                      roi_px, self._full_mask(mask, image_bgr.shape[:2], roi_px))

        array = np.asarray(points, np.float64)
        # x = slope*y + intercept; use the fitted bottom position for steering.
        slope, intercept = np.polyfit(array[:, 1], array[:, 0], 1)
        reference_y = oy + self.config.scan_bottom_ratio * (height-1)
        center_x = float(slope * reference_y + intercept)
        reference_x = ox + width * self.config.center_reference_ratio
        offset = center_x - reference_x
        x_top = float(slope * array[:, 1].min() + intercept)
        x_bottom = float(slope * array[:, 1].max() + intercept)
        vertical = max(float(array[:, 1].max() - array[:, 1].min()), 1.0)
        heading = float(np.degrees(np.arctan2(x_top-x_bottom, vertical)))
        coverage = valid / len(rows)
        residual = float(np.sqrt(np.mean((array[:, 0] - (slope*array[:, 1]+intercept))**2)))
        straightness = float(np.exp(-residual / max(width*.03, 1.0)))
        confidence = float(np.clip(.7*coverage + .3*straightness, 0, 1))
        return BlackLineDetection(
            True, (center_x, reference_y), offset, offset/(width/2), heading,
            confidence, tuple(map(tuple, array)), valid, len(rows), wide_rows > 0,
            roi_px, self._full_mask(mask, image_bgr.shape[:2], roi_px),
        )

    @staticmethod
    def _runs(profile):
        padded = np.pad(profile.astype(np.int8), (1, 1))
        changes = np.diff(padded)
        starts, ends = np.flatnonzero(changes == 1), np.flatnonzero(changes == -1)
        return list(zip(starts.tolist(), ends.tolist()))

    @staticmethod
    def _full_mask(mask, image_shape, roi_px):
        output = np.zeros(image_shape, np.uint8)
        x, y, width, height = roi_px
        output[y:y+height, x:x+width] = mask
        return output

    def annotate(self, image_bgr: np.ndarray, result: BlackLineDetection) -> np.ndarray:
        output = image_bgr.copy()
        x, y, width, height = result.roi_px
        cv2.rectangle(output, (x, y), (x+width-1, y+height-1), (160, 160, 160), 1)
        for point in result.centerline_points_px:
            cv2.circle(output, tuple(map(lambda value: int(round(value)), point)), 5,
                       (0, 255, 255), -1)
        if result.found:
            center = tuple(map(lambda value: int(round(value)), result.center_px))
            cv2.circle(output, center, 8, (0, 255, 0), -1)
            cv2.line(output, (int(x+width*self.config.center_reference_ratio), y+height-1),
                     center, (255, 0, 0), 2)
            label = (f"LINE off={result.lateral_offset_px:+.0f}px "
                     f"heading={result.heading_error_deg:+.1f} conf={result.confidence:.2f}")
            color = (0, 255, 0)
        else:
            label, color = "LINE LOST", (0, 0, 255)
        if result.intersection_detected:
            label += " INTERSECTION"
        cv2.putText(output, label, (12, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    .65, color, 2, cv2.LINE_AA)
        return output
