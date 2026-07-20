from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import Enum
import json
import math
from typing import Any


SCHEMA_VERSION = "1.1"


class VisionMode(str, Enum):
    IDLE = "IDLE"
    LOCALIZATION = "LOCALIZATION"
    FIND_BLOCKS = "FIND_BLOCKS"
    FOLLOW_LINE = "FOLLOW_LINE"
    GRAB_ASSIST = "GRAB_ASSIST"
    PLACE_ASSIST = "PLACE_ASSIST"
    SAFE_STOP = "SAFE_STOP"
    DEBUG_ALL = "DEBUG_ALL"


@dataclass(frozen=True)
class StreamOutput:
    status: str
    healthy: bool
    fps: float
    frame_age_s: float
    reconnect_count: int
    reason: str


@dataclass(frozen=True)
class RobotPoseOutput:
    valid: bool
    position_field_m: tuple[float, float, float] | None = None
    rpy_field_deg: tuple[float, float, float] | None = None
    confidence: float = 0.0
    source_tag_ids: tuple[int, ...] = ()
    rejected_tag_ids: tuple[int, ...] = ()


@dataclass(frozen=True)
class TagOutput:
    tag_id: int
    track_id: int | None
    valid: bool
    predicted: bool
    center_px: tuple[float, float]
    corners_px: tuple[tuple[float, float], ...]
    position_camera_m: tuple[float, float, float] | None
    distance_m: float | None
    reprojection_error_px: float | None
    confidence: float


@dataclass(frozen=True)
class BlockOutput:
    track_id: int | None
    color: str
    valid: bool
    predicted: bool
    center_px: tuple[float, float]
    bbox_px: tuple[int, int, int, int]
    position_robot_m: tuple[float, float, float] | None
    confidence: float


@dataclass(frozen=True)
class LineOutput:
    valid: bool
    lateral_offset_px: float | None
    lateral_offset_normalized: float | None
    heading_error_deg: float | None
    confidence: float
    intersection_detected: bool


@dataclass(frozen=True)
class ProcessingOutput:
    total_ms: float
    apriltag_ms: float = 0.0
    blocks_ms: float = 0.0
    line_ms: float = 0.0
    coordinate_ms: float = 0.0


@dataclass(frozen=True)
class SelectedTargetOutput:
    valid: bool = False
    track_id: int | None = None
    score: float = 0.0
    reason: str = "not requested"


@dataclass(frozen=True)
class GripperAlignmentOutput:
    valid: bool = False
    target_track_id: int | None = None
    dx_px: float | None = None
    dy_px: float | None = None
    angle_error_deg: float | None = None
    aligned: bool = False
    confidence: float = 0.0
    predicted: bool = False
    reason: str = "not requested"


@dataclass(frozen=True)
class PlacementOutput:
    valid: bool = False
    slot_id: str | None = None
    reference_tag_id: int | None = None
    position_robot_m: tuple[float, float, float] | None = None
    rpy_robot_deg: tuple[float, float, float] | None = None
    layer: int | None = None
    reason: str = "not requested"


@dataclass(frozen=True)
class ManipulationOutput:
    valid: bool = False
    phase: str | None = None
    success: bool = False
    confidence: float = 0.0
    reason: str = "not requested"


@dataclass(frozen=True)
class VisionOutput:
    frame_id: int
    timestamp_s: float
    source_name: str
    mode: str
    stream: StreamOutput
    robot_pose: RobotPoseOutput = field(default_factory=lambda: RobotPoseOutput(False))
    tags: tuple[TagOutput, ...] = ()
    blocks: tuple[BlockOutput, ...] = ()
    line: LineOutput = field(default_factory=lambda: LineOutput(False, None, None, None, 0.0, False))
    processing: ProcessingOutput = field(default_factory=lambda: ProcessingOutput(0.0))
    errors: tuple[str, ...] = ()
    selected_target: SelectedTargetOutput = field(default_factory=SelectedTargetOutput)
    gripper_alignment: GripperAlignmentOutput = field(default_factory=GripperAlignmentOutput)
    placement: PlacementOutput = field(default_factory=PlacementOutput)
    manipulation: ManipulationOutput = field(default_factory=ManipulationOutput)
    schema_version: str = SCHEMA_VERSION

    @property
    def valid(self) -> bool:
        return self.stream.healthy and not self.errors

    def to_dict(self) -> dict[str, Any]:
        data = asdict(self)
        data["valid"] = self.valid
        return _json_safe(data)

    def to_json(self, *, indent: int | None = None) -> str:
        return json.dumps(self.to_dict(), ensure_ascii=False, separators=None if indent else (",", ":"),
                          indent=indent, allow_nan=False)


def _json_safe(value):
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    return value
