from __future__ import annotations

from dataclasses import dataclass
from math import hypot
from typing import Iterable

from .vision_output import BlockOutput


@dataclass
class TargetSelectionConfig:
    desired_color: str = "orange"
    minimum_confidence: float = 0.65
    maximum_distance_m: float = 2.5
    confidence_weight: float = 0.55
    distance_weight: float = 0.35
    center_weight: float = 0.10
    predicted_penalty: float = 0.40


@dataclass(frozen=True)
class TargetSelection:
    valid: bool
    track_id: int | None
    score: float
    reason: str


class TargetSelector:
    def __init__(self, config: TargetSelectionConfig) -> None:
        self.config = config

    def select(self, blocks: Iterable[BlockOutput], image_width: int = 1280,
               desired_color: str | None = None) -> TargetSelection:
        selected_color = (desired_color or self.config.desired_color).lower()
        if selected_color not in ("orange", "purple"):
            raise ValueError(f"unsupported desired block color: {selected_color}")
        scored = []
        for block in blocks:
            if block.color != selected_color or not block.valid:
                continue
            if block.confidence < self.config.minimum_confidence:
                continue
            distance_score = 0.5
            if block.position_robot_m is not None:
                distance = hypot(block.position_robot_m[0], block.position_robot_m[1])
                if distance > self.config.maximum_distance_m:
                    continue
                distance_score = max(0.0, 1.0-distance/self.config.maximum_distance_m)
            center_score = max(0.0, 1.0-abs(block.center_px[0]-image_width/2)/(image_width/2))
            score = (self.config.confidence_weight*block.confidence +
                     self.config.distance_weight*distance_score +
                     self.config.center_weight*center_score -
                     (self.config.predicted_penalty if block.predicted else 0.0))
            scored.append((score, block.track_id))
        if not scored:
            return TargetSelection(False, None, 0.0, "no reachable stable target")
        score, track_id = max(scored)
        return TargetSelection(True, track_id, float(score), "highest selection score")
