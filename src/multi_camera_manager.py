from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import json
from pathlib import Path
from typing import Mapping

from .image_source import CameraConfig, FramePacket, ImageSource


class CameraRole(str, Enum):
    FRONT = "front"
    GRIPPER = "gripper"


@dataclass(frozen=True)
class CameraChannel:
    role: CameraRole
    source: ImageSource


class MultiCameraManager:
    """Own independent front/gripper inputs without sharing frame state."""

    def __init__(self, channels: Mapping[CameraRole | str, ImageSource]) -> None:
        self.channels = {CameraRole(role): source for role, source in channels.items()}
        if not self.channels:
            raise ValueError("at least one camera channel is required")

    @classmethod
    def from_json(cls, path: str | Path):
        raw = json.loads(Path(path).read_text(encoding="utf-8"))
        channels = {}
        for name, item in raw.get("cameras", {}).items():
            if not item.get("configured", False):
                continue
            source_value = item["source"]
            source = int(source_value) if isinstance(source_value, str) and source_value.isdigit() else source_value
            camera = CameraConfig(**item.get("camera", {}))
            channels[CameraRole(name)] = ImageSource(source, camera)
        return cls(channels)

    def read(self, role: CameraRole | str) -> FramePacket | None:
        selected = CameraRole(role)
        if selected not in self.channels:
            raise KeyError(f"camera role is not configured: {selected.value}")
        return self.channels[selected].read()

    def read_all(self) -> dict[CameraRole, FramePacket | None]:
        return {role: source.read() for role, source in self.channels.items()}

    def properties(self) -> dict[str, dict]:
        return {role.value: source.properties() for role, source in self.channels.items()}

    def release(self) -> None:
        for source in self.channels.values():
            source.release()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.release()
