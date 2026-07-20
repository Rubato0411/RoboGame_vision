from __future__ import annotations

from dataclasses import dataclass, replace
import json
from pathlib import Path
from typing import Any, Iterable

import numpy as np

from .apriltag_detector import AprilTagDetection
from .block_detector_robust import BlockDetection


@dataclass
class TemporalTrackerConfig:
    confirmation_hits: int = 3
    max_missed_frames: int = 5
    smoothing_alpha: float = 0.35
    max_center_jump_px: float = 180.0
    min_iou_for_blocks: float = 0.0
    show_tentative: bool = False

    @classmethod
    def from_json(cls, path: str | Path) -> "TemporalTrackerConfig":
        return cls(**json.loads(Path(path).read_text(encoding="utf-8")))

    def validate(self) -> None:
        if self.confirmation_hits < 1:
            raise ValueError("confirmation_hits must be at least 1")
        if self.max_missed_frames < 0:
            raise ValueError("max_missed_frames cannot be negative")
        if not 0 < self.smoothing_alpha <= 1:
            raise ValueError("smoothing_alpha must be in (0, 1]")
        if self.max_center_jump_px <= 0:
            raise ValueError("max_center_jump_px must be positive")


@dataclass(frozen=True)
class TemporalObservation:
    category: str
    center_px: tuple[float, float]
    detection: Any
    identity: int | None = None


@dataclass(frozen=True)
class TrackedDetection:
    track_id: int
    category: str
    identity: int | None
    center_px: tuple[float, float]
    detection: Any
    age_frames: int
    hit_streak: int
    missed_frames: int
    confirmed: bool
    predicted: bool


@dataclass(frozen=True)
class TemporalResult:
    tracks: tuple[TrackedDetection, ...]
    appeared_track_ids: tuple[int, ...]
    lost_track_ids: tuple[int, ...]
    rejected_jumps: int


@dataclass
class _TrackState:
    track_id: int
    category: str
    identity: int | None
    center: np.ndarray
    detection: Any
    age: int = 1
    hits: int = 1
    missed: int = 0
    confirmed: bool = False


class TemporalObjectTracker:
    """Frame-to-frame confirmation and smoothing for tags and colour blocks."""

    def __init__(self, config: TemporalTrackerConfig) -> None:
        config.validate()
        self.config = config
        self._tracks: dict[int, _TrackState] = {}
        self._next_id = 1

    @classmethod
    def from_json(cls, path: str | Path) -> "TemporalObjectTracker":
        return cls(TemporalTrackerConfig.from_json(path))

    def reset(self) -> None:
        self._tracks.clear()
        self._next_id = 1

    def update(self, observations: Iterable[TemporalObservation]) -> TemporalResult:
        items = list(observations)
        unmatched_tracks = set(self._tracks)
        unmatched_observations = set(range(len(items)))
        matches: list[tuple[int, int]] = []
        rejected_jumps = 0

        # Tags and any future identified objects use exact identity matching.
        for index, observation in enumerate(items):
            if observation.identity is None:
                continue
            candidates = [state for state in self._tracks.values()
                          if state.category == observation.category and
                          state.identity == observation.identity]
            if not candidates:
                continue
            state = min(candidates, key=lambda value: np.linalg.norm(value.center - observation.center_px))
            distance = float(np.linalg.norm(state.center - observation.center_px))
            if distance > self.config.max_center_jump_px:
                rejected_jumps += 1
                unmatched_observations.discard(index)
                continue
            matches.append((state.track_id, index))
            unmatched_tracks.discard(state.track_id)
            unmatched_observations.discard(index)

        # Unidentified blocks use greedy nearest-neighbour association by class.
        candidates = []
        for track_id in unmatched_tracks:
            state = self._tracks[track_id]
            if state.identity is not None:
                continue
            for index in unmatched_observations:
                observation = items[index]
                if observation.identity is None and observation.category == state.category:
                    distance = float(np.linalg.norm(state.center - observation.center_px))
                    if distance <= self.config.max_center_jump_px:
                        candidates.append((distance, track_id, index))
        for _, track_id, index in sorted(candidates):
            if track_id not in unmatched_tracks or index not in unmatched_observations:
                continue
            matches.append((track_id, index))
            unmatched_tracks.remove(track_id)
            unmatched_observations.remove(index)

        appeared = []
        alpha = self.config.smoothing_alpha
        for track_id, index in matches:
            state, observation = self._tracks[track_id], items[index]
            state.center = (1 - alpha) * state.center + alpha * np.asarray(observation.center_px)
            state.detection = self._smooth_detection(state.detection, observation.detection, alpha, state.center)
            state.age += 1
            state.hits += 1
            state.missed = 0
            if not state.confirmed and state.hits >= self.config.confirmation_hits:
                state.confirmed = True
                appeared.append(track_id)

        for track_id in unmatched_tracks:
            state = self._tracks[track_id]
            state.age += 1
            state.missed += 1
            state.hits = 0

        for index in unmatched_observations:
            observation = items[index]
            track_id = self._next_id
            self._next_id += 1
            confirmed = self.config.confirmation_hits == 1
            self._tracks[track_id] = _TrackState(
                track_id, observation.category, observation.identity,
                np.asarray(observation.center_px, np.float64), observation.detection,
                confirmed=confirmed,
            )
            if confirmed:
                appeared.append(track_id)

        lost = []
        for track_id, state in list(self._tracks.items()):
            if state.missed > self.config.max_missed_frames:
                if state.confirmed:
                    lost.append(track_id)
                del self._tracks[track_id]

        visible = []
        for state in self._tracks.values():
            if not state.confirmed and not self.config.show_tentative:
                continue
            visible.append(TrackedDetection(
                state.track_id, state.category, state.identity, tuple(state.center),
                state.detection, state.age, state.hits, state.missed,
                state.confirmed, state.missed > 0,
            ))
        visible.sort(key=lambda item: item.track_id)
        return TemporalResult(tuple(visible), tuple(sorted(appeared)),
                              tuple(sorted(lost)), rejected_jumps)

    @staticmethod
    def _smooth_detection(old, new, alpha, center):
        if isinstance(new, AprilTagDetection) and isinstance(old, AprilTagDetection):
            old_corners = np.asarray(old.corners_px)
            new_corners = np.asarray(new.corners_px)
            corners = (1 - alpha) * old_corners + alpha * new_corners
            tvec = None
            if old.tvec_m is not None and new.tvec_m is not None:
                tvec = (1 - alpha) * old.tvec_m + alpha * new.tvec_m
            distance = None if tvec is None else float(np.linalg.norm(tvec))
            return replace(new, center_px=tuple(center), corners_px=tuple(map(tuple, corners)),
                           tvec_m=tvec, distance_m=distance)
        if isinstance(new, BlockDetection):
            return replace(new, center_px=tuple(center))
        return new


def observations_from_apriltags(detections: Iterable[AprilTagDetection]):
    return [TemporalObservation("apriltag", item.center_px, item, item.tag_id)
            for item in detections]


def observations_from_blocks(detections: Iterable[BlockDetection]):
    return [TemporalObservation(f"block:{item.class_name}", item.center_px, item)
            for item in detections]
