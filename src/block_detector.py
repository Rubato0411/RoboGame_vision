from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Dict, Iterable, Optional

import cv2
import numpy as np


@dataclass(frozen=True)
class BlockDetection:
    """One detected block in image coordinates."""

    class_name: str
    center_px: tuple[float, float]
    bbox: tuple[int, int, int, int]
    contour_area: float
    rectangularity: float
    solidity: float
    confidence: float


@dataclass
class ColorRule:
    hsv_lower: tuple[int, int, int]
    hsv_upper: tuple[int, int, int]
    draw_color_bgr: tuple[int, int, int]


@dataclass
class BlockDetectorConfig:
    colors: Dict[str, ColorRule]
    blur_kernel: int = 5
    morph_kernel: int = 5
    open_iterations: int = 1
    close_iterations: int = 2
    min_area_px: float = 500.0
    max_area_ratio: float = 0.7
    min_aspect_ratio: float = 0.45
    max_aspect_ratio: float = 2.2
    min_rectangularity: float = 0.55
    min_solidity: float = 0.75

    @classmethod
    def from_json(cls, path: str | Path) -> "BlockDetectorConfig":
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        colors = {}
        for name in ("orange", "purple"):
            rule = data[name]
            colors[name] = ColorRule(
                hsv_lower=tuple(rule["hsv_lower"]),
                hsv_upper=tuple(rule["hsv_upper"]),
                draw_color_bgr=tuple(rule["draw_color_bgr"]),
            )
        common = {key: value for key, value in data.items() if key not in colors}
        return cls(colors=colors, **common)


@dataclass(frozen=True)
class DetectionResult:
    detections: tuple[BlockDetection, ...]
    masks: Dict[str, np.ndarray]


class BlockDetector:
    """HSV and geometry based orange/purple block detector.

    This baseline deliberately has no ROS or camera dependency. It accepts one
    BGR image and returns structured detections, making it easy to unit-test and
    later wrap in a ROS2 node.
    """

    def __init__(self, config: BlockDetectorConfig) -> None:
        self.config = config
        self._validate_config()

    @classmethod
    def from_json(cls, path: str | Path) -> "BlockDetector":
        return cls(BlockDetectorConfig.from_json(path))

    def _validate_config(self) -> None:
        for name in ("blur_kernel", "morph_kernel"):
            value = int(getattr(self.config, name))
            if value <= 0 or value % 2 == 0:
                raise ValueError(f"{name} must be a positive odd number")
        if not 0 < self.config.max_area_ratio <= 1:
            raise ValueError("max_area_ratio must be in (0, 1]")

    def process(self, image_bgr: np.ndarray) -> DetectionResult:
        if image_bgr is None or image_bgr.size == 0:
            raise ValueError("Input image is empty")
        if image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
            raise ValueError("Input image must be a BGR image with three channels")

        blurred = cv2.GaussianBlur(
            image_bgr,
            (self.config.blur_kernel, self.config.blur_kernel),
            0,
        )
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT,
            (self.config.morph_kernel, self.config.morph_kernel),
        )

        all_detections: list[BlockDetection] = []
        masks: Dict[str, np.ndarray] = {}
        for class_name, rule in self.config.colors.items():
            mask = cv2.inRange(
                hsv,
                np.array(rule.hsv_lower, dtype=np.uint8),
                np.array(rule.hsv_upper, dtype=np.uint8),
            )
            if self.config.open_iterations > 0:
                mask = cv2.morphologyEx(
                    mask,
                    cv2.MORPH_OPEN,
                    kernel,
                    iterations=self.config.open_iterations,
                )
            if self.config.close_iterations > 0:
                mask = cv2.morphologyEx(
                    mask,
                    cv2.MORPH_CLOSE,
                    kernel,
                    iterations=self.config.close_iterations,
                )
            masks[class_name] = mask
            all_detections.extend(self._extract_detections(mask, class_name, image_bgr.shape))

        # Stable ordering is useful for tests and later target selection.
        all_detections.sort(key=lambda item: (-item.confidence, -item.contour_area, item.class_name))
        return DetectionResult(tuple(all_detections), masks)

    def _extract_detections(
        self,
        mask: np.ndarray,
        class_name: str,
        image_shape: tuple[int, ...],
    ) -> Iterable[BlockDetection]:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        image_area = float(image_shape[0] * image_shape[1])

        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < self.config.min_area_px or area > image_area * self.config.max_area_ratio:
                continue

            x, y, width, height = cv2.boundingRect(contour)
            if width <= 0 or height <= 0:
                continue
            aspect_ratio = width / float(height)
            if not self.config.min_aspect_ratio <= aspect_ratio <= self.config.max_aspect_ratio:
                continue

            bbox_area = float(width * height)
            rectangularity = area / bbox_area
            if rectangularity < self.config.min_rectangularity:
                continue

            hull = cv2.convexHull(contour)
            hull_area = float(cv2.contourArea(hull))
            solidity = area / hull_area if hull_area > 0 else 0.0
            if solidity < self.config.min_solidity:
                continue

            moments = cv2.moments(contour)
            if abs(moments["m00"]) > 1e-6:
                center_x = moments["m10"] / moments["m00"]
                center_y = moments["m01"] / moments["m00"]
            else:
                center_x, center_y = x + width / 2.0, y + height / 2.0

            # This is a heuristic quality score, not a calibrated probability.
            shape_score = np.clip(
                0.55 * rectangularity + 0.35 * solidity + 0.10 * min(area / (4 * self.config.min_area_px), 1.0),
                0.0,
                1.0,
            )
            yield BlockDetection(
                class_name=class_name,
                center_px=(float(center_x), float(center_y)),
                bbox=(x, y, width, height),
                contour_area=area,
                rectangularity=rectangularity,
                solidity=solidity,
                confidence=float(shape_score),
            )

    def annotate(self, image_bgr: np.ndarray, detections: Iterable[BlockDetection]) -> np.ndarray:
        output = image_bgr.copy()
        for item in detections:
            x, y, width, height = item.bbox
            color = self.config.colors[item.class_name].draw_color_bgr
            cv2.rectangle(output, (x, y), (x + width, y + height), color, 2)
            center = (int(round(item.center_px[0])), int(round(item.center_px[1])))
            cv2.circle(output, center, 4, color, -1)
            label = f"{item.class_name} {item.confidence:.2f}"
            cv2.putText(output, label, (x, max(y - 8, 18)), cv2.FONT_HERSHEY_SIMPLEX, 0.58, color, 2, cv2.LINE_AA)
        return output

    @staticmethod
    def combined_mask(masks: Dict[str, np.ndarray]) -> np.ndarray:
        if not masks:
            raise ValueError("No masks to combine")
        values = list(masks.values())
        combined = values[0].copy()
        for mask in values[1:]:
            combined = cv2.bitwise_or(combined, mask)
        return combined
