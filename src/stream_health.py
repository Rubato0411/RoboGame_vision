from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import Enum
import json
from pathlib import Path
from time import monotonic

import cv2
import numpy as np

from .image_source import FramePacket


class StreamStatus(str, Enum):
    STARTING = "STARTING"
    HEALTHY = "HEALTHY"
    LOW_FPS = "LOW_FPS"
    FROZEN = "FROZEN"
    STALE_FRAME = "STALE_FRAME"
    INVALID_FRAME = "INVALID_FRAME"
    TIMEOUT = "TIMEOUT"
    DISCONNECTED = "DISCONNECTED"
    RECOVERING = "RECOVERING"


@dataclass
class StreamHealthConfig:
    expected_fps: float = 30.0
    minimum_fps_ratio: float = 0.50
    fps_window_frames: int = 30
    startup_grace_frames: int = 10
    frame_timeout_s: float = 1.0
    disconnect_timeout_s: float = 3.0
    max_consecutive_failures: int = 3
    freeze_threshold: float = 0.35
    freeze_timeout_s: float = 2.0
    thumbnail_width: int = 64
    thumbnail_height: int = 36
    recovery_frames: int = 10

    @classmethod
    def from_json(cls, path: str | Path) -> "StreamHealthConfig":
        return cls(**json.loads(Path(path).read_text(encoding="utf-8")))

    def validate(self):
        if self.expected_fps <= 0 or self.fps_window_frames < 2:
            raise ValueError("expected_fps must be positive and fps_window_frames >= 2")
        if not 0 < self.minimum_fps_ratio <= 1:
            raise ValueError("minimum_fps_ratio must be in (0, 1]")
        if not 0 < self.frame_timeout_s < self.disconnect_timeout_s:
            raise ValueError("timeouts must satisfy 0 < frame_timeout_s < disconnect_timeout_s")
        if self.freeze_timeout_s <= 0 or self.freeze_threshold < 0:
            raise ValueError("freeze settings must be non-negative")


@dataclass(frozen=True)
class StreamHealth:
    status: StreamStatus
    healthy: bool
    fps: float
    frame_age_s: float
    consecutive_failures: int
    frozen_duration_s: float
    reconnect_count: int
    reason: str


class StreamHealthMonitor:
    """Connection watchdog independent of all target detectors."""

    def __init__(self, config: StreamHealthConfig, clock=monotonic) -> None:
        config.validate()
        self.config = config
        self.clock = clock
        self._timestamps = deque(maxlen=config.fps_window_frames)
        self._last_packet_time: float | None = None
        self._last_frame_id: int | None = None
        self._last_thumbnail: np.ndarray | None = None
        self._frozen_since: float | None = None
        self._failures = 0
        self._reconnect_count = 0
        self._recovery_remaining = 0
        self._last_problem: tuple[StreamStatus, str] | None = None

    @classmethod
    def from_json(cls, path: str | Path) -> "StreamHealthMonitor":
        return cls(StreamHealthConfig.from_json(path))

    def observe(self, packet: FramePacket, now: float | None = None) -> StreamHealth:
        now = self.clock() if now is None else now
        image = packet.image
        if image is None or image.size == 0 or image.ndim not in (2, 3):
            self._failures += 1
            self._last_problem = (StreamStatus.INVALID_FRAME, "empty or malformed image")
            return self.check(now)

        stale = (self._last_frame_id is not None and packet.frame_id <= self._last_frame_id) or (
            self._last_packet_time is not None and packet.timestamp <= self._last_packet_time)
        if stale:
            self._last_problem = (StreamStatus.STALE_FRAME, "frame id or timestamp did not advance")
        else:
            self._last_problem = None

        if packet.reconnect_count > self._reconnect_count:
            self._reconnect_count = packet.reconnect_count
            self._recovery_remaining = self.config.recovery_frames

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
        thumb = cv2.resize(gray, (self.config.thumbnail_width, self.config.thumbnail_height),
                           interpolation=cv2.INTER_AREA)
        if self._last_thumbnail is not None:
            difference = float(np.mean(cv2.absdiff(thumb, self._last_thumbnail)))
            if difference <= self.config.freeze_threshold:
                if self._frozen_since is None:
                    self._frozen_since = now
            else:
                self._frozen_since = None
        self._last_thumbnail = thumb
        self._last_packet_time = now
        self._last_frame_id = packet.frame_id
        self._timestamps.append(now)
        self._failures = 0
        if self._recovery_remaining > 0:
            self._recovery_remaining -= 1
        return self.check(now)

    def observe_failure(self, now: float | None = None) -> StreamHealth:
        self._failures += 1
        return self.check(self.clock() if now is None else now)

    def check(self, now: float | None = None) -> StreamHealth:
        now = self.clock() if now is None else now
        age = float("inf") if self._last_packet_time is None else max(0.0, now - self._last_packet_time)
        fps = self._fps()
        frozen_for = 0.0 if self._frozen_since is None else max(0.0, now - self._frozen_since)

        if age >= self.config.disconnect_timeout_s or self._failures >= self.config.max_consecutive_failures:
            status, reason = StreamStatus.DISCONNECTED, "no usable frames"
        elif age >= self.config.frame_timeout_s:
            status, reason = StreamStatus.TIMEOUT, "latest frame is too old"
        elif self._last_problem is not None:
            status, reason = self._last_problem
        elif frozen_for >= self.config.freeze_timeout_s:
            status, reason = StreamStatus.FROZEN, "image content has not changed"
        elif self._recovery_remaining > 0:
            status, reason = StreamStatus.RECOVERING, "validating frames after reconnect"
        elif len(self._timestamps) < self.config.startup_grace_frames:
            status, reason = StreamStatus.STARTING, "collecting frame statistics"
        elif fps < self.config.expected_fps * self.config.minimum_fps_ratio:
            status, reason = StreamStatus.LOW_FPS, "measured fps below threshold"
        else:
            status, reason = StreamStatus.HEALTHY, "stream is continuous"
        healthy = status in (StreamStatus.HEALTHY, StreamStatus.STARTING)
        return StreamHealth(status, healthy, fps, age, self._failures, frozen_for,
                            self._reconnect_count, reason)

    def _fps(self) -> float:
        if len(self._timestamps) < 2:
            return 0.0
        elapsed = self._timestamps[-1] - self._timestamps[0]
        return (len(self._timestamps) - 1) / elapsed if elapsed > 0 else 0.0
