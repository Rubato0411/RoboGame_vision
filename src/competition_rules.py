from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Iterable


@dataclass(frozen=True)
class CompetitionRules:
    """Rule values that are known before robot hardware is available."""

    rules_release_date: str = "2026-06-15"
    preparation_duration_s: float = 60.0
    match_duration_s: float = 360.0
    build_stability_s: float = 3.0
    abnormal_restart_wait_s: float = 6.0
    major_win_hold_s: float = 10.0
    major_win_score: float = 14.0
    major_win_lead: float = 10.0
    block_edge_m: float = 0.10
    block_size_tolerance_ratio: float = 0.05
    orange_slot_width_m: float = 0.11
    purple_slot_width_m: float = 0.11
    line_width_m: float = 0.05
    tag_edge_m: float = 0.15
    tag_upper_edge_height_m: float = 0.40
    max_carried_blocks: int = 3
    max_carried_purple_blocks: int = 1
    max_available_purple_blocks: int = 3
    max_scoring_buildings: int = 3

    @classmethod
    def from_json(cls, path: str | Path) -> "CompetitionRules":
        raw = json.loads(Path(path).read_text(encoding="utf-8"))
        values = raw.get("rules", raw)
        result = cls(**values)
        result.validate()
        return result

    def validate(self) -> None:
        positive = {
            "preparation_duration_s": self.preparation_duration_s,
            "match_duration_s": self.match_duration_s,
            "build_stability_s": self.build_stability_s,
            "abnormal_restart_wait_s": self.abnormal_restart_wait_s,
            "block_edge_m": self.block_edge_m,
            "orange_slot_width_m": self.orange_slot_width_m,
            "purple_slot_width_m": self.purple_slot_width_m,
            "line_width_m": self.line_width_m,
            "tag_edge_m": self.tag_edge_m,
        }
        invalid = [name for name, value in positive.items() if value <= 0]
        if invalid:
            raise ValueError(f"competition rule values must be positive: {', '.join(invalid)}")
        if not 0 <= self.block_size_tolerance_ratio < 1:
            raise ValueError("block_size_tolerance_ratio must be in [0, 1)")
        if self.max_carried_blocks < 1:
            raise ValueError("max_carried_blocks must be at least one")
        if not 0 <= self.max_carried_purple_blocks <= self.max_carried_blocks:
            raise ValueError("purple carrying limit must fit the total carrying limit")
        if self.max_available_purple_blocks < self.max_carried_purple_blocks:
            raise ValueError("available purple blocks cannot be below carrying limit")
        if self.max_scoring_buildings < 1:
            raise ValueError("max_scoring_buildings must be at least one")

    def carrying_allowed(self, carried_colors: Iterable[str], next_color: str) -> bool:
        colors = tuple(str(value).lower() for value in carried_colors)
        selected = str(next_color).lower()
        if selected not in ("orange", "purple"):
            return False
        if len(colors) >= self.max_carried_blocks:
            return False
        return not (selected == "purple" and
                    colors.count("purple") >= self.max_carried_purple_blocks)

    @staticmethod
    def building_score(colors_bottom_to_top: Iterable[str]) -> float:
        """Score one vertical stack using the rulebook's layer examples.

        A purple block acts as a roof. Blocks above the first roof do not score.
        """
        scored = []
        for raw_color in colors_bottom_to_top:
            color = str(raw_color).lower()
            if color not in ("orange", "purple"):
                raise ValueError(f"unsupported block color: {raw_color}")
            scored.append(color)
            if color == "purple":
                break
        base = float(max(0, len(scored) - 1))
        return base * 1.5 if scored and scored[-1] == "purple" else base

    def total_building_score(self, buildings: Iterable[Iterable[str]]) -> float:
        scores = sorted((self.building_score(value) for value in buildings), reverse=True)
        return float(sum(scores[:self.max_scoring_buildings]))
