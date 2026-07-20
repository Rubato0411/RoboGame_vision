from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path


@dataclass
class GripperAlignmentConfig:
    grasp_reference_px: tuple[float, float] = (640.0, 360.0)
    max_dx_px: float = 12.0
    max_dy_px: float = 12.0
    max_angle_error_deg: float = 5.0
    desired_angle_deg: float = 0.0
    angle_symmetry_period_deg: float = 90.0
    minimum_confidence: float = 0.70
    allow_predicted: bool = False

    @classmethod
    def from_json(cls, path: str | Path):
        raw = json.loads(Path(path).read_text(encoding="utf-8"))
        raw["grasp_reference_px"] = tuple(raw["grasp_reference_px"])
        return cls(**raw)


@dataclass(frozen=True)
class GripperAlignmentResult:
    valid: bool
    target_track_id: int | None
    dx_px: float | None
    dy_px: float | None
    angle_error_deg: float | None
    centered: bool
    angle_ready: bool
    aligned: bool
    confidence: float
    predicted: bool
    reason: str


class GripperAligner:
    def __init__(self, config: GripperAlignmentConfig) -> None:
        if (config.max_dx_px <= 0 or config.max_dy_px <= 0 or
                config.max_angle_error_deg <= 0 or config.angle_symmetry_period_deg <= 0):
            raise ValueError("alignment tolerances must be positive")
        self.config = config

    @classmethod
    def from_json(cls, path: str | Path):
        return cls(GripperAlignmentConfig.from_json(path))

    def align(self, center_px, angle_deg: float, confidence: float,
              track_id: int | None = None, predicted: bool = False) -> GripperAlignmentResult:
        dx = float(center_px[0] - self.config.grasp_reference_px[0])
        dy = float(center_px[1] - self.config.grasp_reference_px[1])
        centered = abs(dx) <= self.config.max_dx_px and abs(dy) <= self.config.max_dy_px
        period = self.config.angle_symmetry_period_deg
        raw_angle = float(angle_deg) - self.config.desired_angle_deg
        angle_error = ((raw_angle + period/2.0) % period) - period/2.0
        angle_ready = abs(angle_error) <= self.config.max_angle_error_deg
        valid = confidence >= self.config.minimum_confidence and (
            self.config.allow_predicted or not predicted)
        aligned = valid and centered and angle_ready
        reason = "ok" if aligned else (
            "predicted target" if predicted and not self.config.allow_predicted else
            "low confidence" if confidence < self.config.minimum_confidence else
            "position not centered" if not centered else "angle not aligned")
        return GripperAlignmentResult(valid, track_id, dx, dy, angle_error,
                                      centered, angle_ready, aligned, confidence,
                                      predicted, reason)

    def no_target(self) -> GripperAlignmentResult:
        return GripperAlignmentResult(False, None, None, None, None, False,
                                      False, False, 0.0, False, "no target")
