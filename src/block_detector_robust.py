from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
import json
from pathlib import Path
from typing import Dict, Iterable

import cv2
import numpy as np


@dataclass(frozen=True)
class BlockDetection:
    class_name: str
    center_px: tuple[float, float]
    bbox: tuple[int, int, int, int]
    rotated_box: tuple[tuple[float, float], ...]
    angle_deg: float
    contour_area: float
    rectangularity: float
    rotated_rectangularity: float
    solidity: float
    color_coverage: float
    confidence: float
    touches_border: bool


@dataclass
class ColorRule:
    hsv_lower: tuple[int, int, int]
    hsv_upper: tuple[int, int, int]
    draw_color_bgr: tuple[int, int, int]


@dataclass
class BlockDetectorConfig:
    colors: Dict[str, ColorRule]
    roi_normalized: tuple[float, float, float, float] = (0.0, 0.0, 1.0, 1.0)
    gamma: float = 1.0
    clahe_enabled: bool = False
    clahe_clip_limit: float = 2.0
    clahe_grid_size: int = 8
    blur_kernel: int = 5
    morph_kernel: int = 5
    open_iterations: int = 1
    close_iterations: int = 2
    min_area_px: float = 500.0
    max_area_ratio: float = 0.7
    min_aspect_ratio: float = 0.45
    max_aspect_ratio: float = 2.2
    min_rectangularity: float = 0.55
    min_rotated_rectangularity: float = 0.60
    min_solidity: float = 0.75
    min_color_coverage: float = 0.55
    border_margin_px: int = 2
    reject_border_touching: bool = False
    nms_iou_threshold: float = 0.35
    split_touching_enabled: bool = True
    split_peak_ratio: float = 0.52
    split_erosion_ratio: float = 0.10
    split_max_erosion_iterations: int = 48
    split_min_seed_area_px: int = 80
    split_min_region_area_px: int = 300

    @classmethod
    def from_json(cls, path: str | Path) -> "BlockDetectorConfig":
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        colors = {}
        for name in ("orange", "purple"):
            rule = data.pop(name)
            colors[name] = ColorRule(tuple(rule["hsv_lower"]), tuple(rule["hsv_upper"]),
                                     tuple(rule["draw_color_bgr"]))
        data["roi_normalized"] = tuple(data.get("roi_normalized", (0, 0, 1, 1)))
        return cls(colors=colors, **data)


@dataclass(frozen=True)
class DetectionResult:
    detections: tuple[BlockDetection, ...]
    masks: Dict[str, np.ndarray]
    rejection_counts: Dict[str, int]
    roi_px: tuple[int, int, int, int]


class BlockDetector:
    """HSV baseline hardened with ROI, illumination preprocessing and diagnostics."""

    def __init__(self, config: BlockDetectorConfig) -> None:
        self.config = config
        self._validate()
        inv_gamma = 1.0 / config.gamma
        self._gamma_lut = np.array([((v / 255.0) ** inv_gamma) * 255 for v in range(256)], np.uint8)

    @classmethod
    def from_json(cls, path: str | Path) -> "BlockDetector":
        return cls(BlockDetectorConfig.from_json(path))

    def _validate(self) -> None:
        for name in ("blur_kernel", "morph_kernel"):
            value = int(getattr(self.config, name))
            if value <= 0 or value % 2 == 0:
                raise ValueError(f"{name} must be a positive odd number")
        if self.config.gamma <= 0:
            raise ValueError("gamma must be positive")
        x1, y1, x2, y2 = self.config.roi_normalized
        if not (0 <= x1 < x2 <= 1 and 0 <= y1 < y2 <= 1):
            raise ValueError("Invalid normalized ROI")

    def _get_roi(self, image: np.ndarray):
        h, w = image.shape[:2]
        x1, y1, x2, y2 = self.config.roi_normalized
        left, top = int(x1 * w), int(y1 * h)
        right, bottom = max(left + 1, int(x2 * w)), max(top + 1, int(y2 * h))
        return image[top:bottom, left:right], (left, top, right - left, bottom - top)

    def _preprocess(self, image: np.ndarray) -> np.ndarray:
        output = cv2.LUT(image, self._gamma_lut) if abs(self.config.gamma - 1) > 1e-3 else image
        if self.config.clahe_enabled:
            lab = cv2.cvtColor(output, cv2.COLOR_BGR2LAB)
            l_chan, a_chan, b_chan = cv2.split(lab)
            clahe = cv2.createCLAHE(self.config.clahe_clip_limit,
                                    (self.config.clahe_grid_size, self.config.clahe_grid_size))
            output = cv2.cvtColor(cv2.merge((clahe.apply(l_chan), a_chan, b_chan)), cv2.COLOR_LAB2BGR)
        return cv2.GaussianBlur(output, (self.config.blur_kernel, self.config.blur_kernel), 0)

    def process(self, image: np.ndarray) -> DetectionResult:
        if image is None or image.size == 0:
            raise ValueError("Input image is empty")
        if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
            raise ValueError("Input must be an 8-bit three-channel BGR image")
        roi, roi_px = self._get_roi(image)
        ox, oy, _, _ = roi_px
        hsv = cv2.cvtColor(self._preprocess(roi), cv2.COLOR_BGR2HSV)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                           (self.config.morph_kernel, self.config.morph_kernel))
        masks, candidates, rejected = {}, [], Counter()
        for class_name, rule in self.config.colors.items():
            local = cv2.inRange(hsv, np.array(rule.hsv_lower, np.uint8),
                                np.array(rule.hsv_upper, np.uint8))
            if self.config.open_iterations:
                local = cv2.morphologyEx(local, cv2.MORPH_OPEN, kernel,
                                         iterations=self.config.open_iterations)
            if self.config.close_iterations:
                local = cv2.morphologyEx(local, cv2.MORPH_CLOSE, kernel,
                                         iterations=self.config.close_iterations)
            full = np.zeros(image.shape[:2], np.uint8)
            full[oy:oy + local.shape[0], ox:ox + local.shape[1]] = local
            masks[class_name] = full
            candidates.extend(self._extract(local, class_name, roi_px, rejected))
        detections = self._nms(candidates)
        detections.sort(key=lambda d: (-d.confidence, -d.contour_area, d.class_name))
        return DetectionResult(tuple(detections), masks, dict(rejected), roi_px)

    def _split_touching_contours(self, mask, rejected):
        """Split one connected color region using distance peaks and watershed."""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not self.config.split_touching_enabled:
            return contours

        output = []
        for contour in contours:
            x, y, width, height = cv2.boundingRect(contour)
            component = np.zeros((height + 4, width + 4), np.uint8)
            shifted = contour - np.array([[[x - 2, y - 2]]], dtype=contour.dtype)
            cv2.drawContours(component, [shifted], -1, 255, -1)
            distance = cv2.distanceTransform(component, cv2.DIST_L2, 5)
            maximum = float(distance.max())
            if maximum <= 0:
                output.append(contour)
                continue

            # Erosion removes narrow contact bridges while retaining one solid
            # core per block. It works well for stacked blocks of one colour.
            erosion_steps = int(round(maximum * self.config.split_erosion_ratio))
            erosion_steps = max(1, min(erosion_steps, self.config.split_max_erosion_iterations))
            seed_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
            seeds = cv2.erode(component, seed_kernel, iterations=erosion_steps)

            def valid_seed_labels(seed_image):
                count, labels, stats, _ = cv2.connectedComponentsWithStats(seed_image, 8)
                valid = [label for label in range(1, count)
                         if stats[label, cv2.CC_STAT_AREA] >= self.config.split_min_seed_area_px]
                return labels, valid

            projection_used = False
            seed_labels, valid_labels = valid_seed_labels(seeds)
            if len(valid_labels) < 2:
                seeds = np.uint8(distance >= maximum * self.config.split_peak_ratio) * 255
                seed_labels, valid_labels = valid_seed_labels(seeds)
            projection_seeds = self._projection_seeds(component)
            projection_labels, projection_valid = valid_seed_labels(projection_seeds)
            if len(projection_valid) > len(valid_labels):
                seeds, seed_labels, valid_labels = projection_seeds, projection_labels, projection_valid
                projection_used = True
            if len(valid_labels) < 2:
                output.append(contour)
                continue

            markers = np.zeros(component.shape, np.int32)
            markers[component == 0] = 1
            for marker_id, label in enumerate(valid_labels, start=2):
                markers[seed_labels == label] = marker_id

            normalized = cv2.normalize(distance, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            # Projection seeds already encode the expected stack layout. A flat
            # surface grows them by nearest distance instead of pulling regions
            # toward an unrelated global distance maximum.
            elevation = np.zeros_like(normalized) if projection_used else 255 - normalized
            elevation[component == 0] = 255
            cv2.watershed(cv2.cvtColor(elevation, cv2.COLOR_GRAY2BGR), markers)

            regions = []
            for marker_id in range(2, 2 + len(valid_labels)):
                region = np.uint8((markers == marker_id) & (component > 0)) * 255
                region_contours, _ = cv2.findContours(region, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if not region_contours:
                    continue
                region_contour = max(region_contours, key=cv2.contourArea)
                if cv2.contourArea(region_contour) < self.config.split_min_region_area_px:
                    continue
                region_contour = cv2.convexHull(region_contour)
                region_contour = region_contour + np.array([[[x - 2, y - 2]]], dtype=region_contour.dtype)
                regions.append(region_contour)

            if len(regions) >= 2:
                output.extend(regions)
                rejected["connected_regions_split"] += len(regions) - 1
            else:
                output.append(contour)
        return output

    def _projection_seeds(self, component):
        """Infer block cores from width changes in a stacked silhouette."""
        binary = component > 0
        ys = np.flatnonzero(binary.any(axis=1))
        if len(ys) < 12:
            return np.zeros_like(component)
        top, bottom = int(ys[0]), int(ys[-1]) + 1
        xs = np.flatnonzero(binary.any(axis=0))
        silhouette_width = int(xs[-1] - xs[0] + 1)
        silhouette_height = bottom - top
        silhouette_ratio = silhouette_width / max(silhouette_height, 1)
        if not 0.50 <= silhouette_ratio <= 2.0:
            return np.zeros_like(component)
        widths = binary[top:bottom].sum(axis=1).astype(np.float32)
        window = max(3, (bottom - top) // 30)
        smooth = np.convolve(widths, np.ones(window) / window, mode="same")
        lo, hi = max(window, len(smooth) // 5), min(len(smooth) - window, len(smooth) * 4 // 5)
        if hi <= lo:
            return np.zeros_like(component)
        offset = max(2, window)
        changes = np.zeros_like(smooth)
        changes[offset:-offset] = np.abs(smooth[2 * offset:] - smooth[:-2 * offset])
        split_local = lo + int(np.argmax(changes[lo:hi]))
        typical_width = max(float(np.percentile(widths, 75)), 1.0)
        if changes[split_local] < 0.18 * typical_width:
            return np.zeros_like(component)
        split_y = top + split_local

        seeds = np.zeros_like(component)
        band_regions = []
        for y1, y2 in ((top, split_y), (split_y, bottom)):
            band = np.uint8(binary[y1:y2]) * 255
            band_contours, _ = cv2.findContours(band, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for band_contour in band_contours:
                x, y, w, h = cv2.boundingRect(band_contour)
                if h < 5 or w < 5:
                    continue
                band_regions.append((y1, x, y, w, h))

        if len(band_regions) < 2:
            return seeds
        reference_width = min(region[3] for region in band_regions)
        for y1, x, y, w, h in band_regions:
                block_count = max(1, min(4, int(round(w / max(reference_width, 1)))))
                for index in range(block_count):
                    xa = x + int(round(index * w / block_count))
                    xb = x + int(round((index + 1) * w / block_count))
                    inset = max(2, min((xb - xa) // 8, h // 8))
                    core = binary[y1 + y:y1 + y + h, xa:xb].copy()
                    core[:inset, :] = False; core[-inset:, :] = False
                    core[:, :inset] = False; core[:, -inset:] = False
                    if int(core.sum()) < self.config.split_min_seed_area_px:
                        continue
                    seeds[y1 + y:y1 + y + h, xa:xb][core] = 255
        return seeds

    def _extract(self, mask, class_name, roi_px, rejected):
        contours = self._split_touching_contours(mask, rejected)
        ox, oy, roi_w, roi_h = roi_px
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < self.config.min_area_px:
                rejected["area_too_small"] += 1; continue
            if area > roi_w * roi_h * self.config.max_area_ratio:
                rejected["area_too_large"] += 1; continue
            x, y, w, h = cv2.boundingRect(contour)
            border = (x <= self.config.border_margin_px or y <= self.config.border_margin_px or
                      x + w >= roi_w - self.config.border_margin_px or
                      y + h >= roi_h - self.config.border_margin_px)
            if border and self.config.reject_border_touching:
                rejected["touches_border"] += 1; continue
            rotated = cv2.minAreaRect(contour)
            rw, rh = rotated[1]
            if min(rw, rh) <= 1e-6:
                rejected["degenerate"] += 1; continue
            ratio = max(rw, rh) / min(rw, rh)
            ratio_limit = max(self.config.max_aspect_ratio, 1 / self.config.min_aspect_ratio)
            if ratio > ratio_limit:
                rejected["aspect_ratio"] += 1; continue
            bbox_rect = area / (w * h)
            rotated_rect = area / (rw * rh)
            hull_area = float(cv2.contourArea(cv2.convexHull(contour)))
            solidity = area / hull_area if hull_area else 0
            coverage = cv2.countNonZero(mask[y:y + h, x:x + w]) / float(w * h)
            checks = ((bbox_rect, self.config.min_rectangularity, "rectangularity"),
                      (rotated_rect, self.config.min_rotated_rectangularity, "rotated_rectangularity"),
                      (solidity, self.config.min_solidity, "solidity"),
                      (coverage, self.config.min_color_coverage, "color_coverage"))
            failed = next((name for value, limit, name in checks if value < limit), None)
            if failed:
                rejected[failed] += 1; continue
            moments = cv2.moments(contour)
            cx = moments["m10"] / moments["m00"] if moments["m00"] else x + w / 2
            cy = moments["m01"] / moments["m00"] if moments["m00"] else y + h / 2
            points = cv2.boxPoints(rotated) + np.array((ox, oy), np.float32)
            angle = rotated[2] + (90 if rw < rh else 0)
            confidence = float(np.clip(.28*bbox_rect + .27*rotated_rect + .25*solidity +
                                       .15*coverage + .05*min(area/(4*self.config.min_area_px), 1) -
                                       (.05 if border else 0), 0, 1))
            yield BlockDetection(class_name, (cx + ox, cy + oy), (x + ox, y + oy, w, h),
                                 tuple(map(tuple, points)), float(angle), area, bbox_rect,
                                 rotated_rect, solidity, coverage, confidence, border)

    def _nms(self, items):
        kept = []
        for item in sorted(items, key=lambda d: d.confidence, reverse=True):
            if all(self._iou(item.bbox, old.bbox) < self.config.nms_iou_threshold for old in kept):
                kept.append(item)
        return kept

    @staticmethod
    def _iou(a, b):
        ax, ay, aw, ah = a; bx, by, bw, bh = b
        inter = max(0, min(ax+aw, bx+bw)-max(ax, bx)) * max(0, min(ay+ah, by+bh)-max(ay, by))
        union = aw*ah + bw*bh - inter
        return inter / union if union else 0

    def annotate(self, image, detections: Iterable[BlockDetection], draw_roi=True):
        output = image.copy()
        if draw_roi:
            _, (x, y, w, h) = self._get_roi(image)
            cv2.rectangle(output, (x, y), (x+w-1, y+h-1), (180, 180, 180), 1)
        for item in detections:
            color = self.config.colors[item.class_name].draw_color_bgr
            cv2.polylines(output, [np.array(item.rotated_box, np.int32)], True, color, 2)
            cv2.circle(output, tuple(map(lambda v: int(round(v)), item.center_px)), 4, color, -1)
            x, y, _, _ = item.bbox
            cv2.putText(output, f"{item.class_name} {item.confidence:.2f} a={item.angle_deg:.0f}",
                        (x, max(y-8, 18)), cv2.FONT_HERSHEY_SIMPLEX, .55, color, 2, cv2.LINE_AA)
        return output

    @staticmethod
    def combined_mask(masks):
        combined = np.zeros_like(next(iter(masks.values())))
        for mask in masks.values():
            combined = cv2.bitwise_or(combined, mask)
        return combined
