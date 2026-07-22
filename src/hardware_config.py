from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any, Iterable


DEFAULT_REQUIRED_MEASUREMENTS = (
    "platform.os_release",
    "platform.python_version",
    "platform.opencv_version",
    "cameras.front.device_path",
    "cameras.front.calibration_file",
    "cameras.front.coordinate_geometry_file",
    "cameras.front.actual_width_px",
    "cameras.front.actual_height_px",
    "cameras.front.actual_fps",
    "cameras.front.intrinsics.camera_matrix",
    "cameras.front.intrinsics.distortion_coefficients",
    "cameras.front.robot_from_camera.translation_m",
    "cameras.front.robot_from_camera.rpy_deg",
    "cameras.gripper.device_path",
    "cameras.gripper.calibration_file",
    "cameras.gripper.coordinate_geometry_file",
    "cameras.gripper.actual_width_px",
    "cameras.gripper.actual_height_px",
    "cameras.gripper.actual_fps",
    "cameras.gripper.intrinsics.camera_matrix",
    "cameras.gripper.intrinsics.distortion_coefficients",
    "cameras.gripper.robot_from_camera.translation_m",
    "cameras.gripper.robot_from_camera.rpy_deg",
    "cameras.gripper.grasp_reference_px",
    "cameras.gripper.alignment_tolerance_px",
    "cameras.gripper.alignment_tolerance_deg",
    "field_tags.1.field_from_tag.translation_m",
    "field_tags.1.field_from_tag.rpy_deg",
    "field_tags.2.field_from_tag.translation_m",
    "field_tags.2.field_from_tag.rpy_deg",
    "field_tags.3.field_from_tag.translation_m",
    "field_tags.3.field_from_tag.rpy_deg",
    "field_tags.4.field_from_tag.translation_m",
    "field_tags.4.field_from_tag.rpy_deg",
    "field_tags.5.field_from_tag.translation_m",
    "field_tags.5.field_from_tag.rpy_deg",
    "field_tags.6.field_from_tag.translation_m",
    "field_tags.6.field_from_tag.rpy_deg",
    "lower_controller.transport",
    "lower_controller.device",
    "lower_controller.baudrate",
    "safety.heartbeat_timeout_s",
    "safety.command_timeout_s",
    "safety.physical_estop_cuts_all_actuators",
    "placement.building_zone_reference_tag_id",
    "placement.slot_transforms_tag_m",
    "placement.approach_offset_m",
    "placement.release_offset_m",
    "placement.stable_observation_roi",
    "performance.single_camera_pipeline_p95_ms",
    "performance.dual_camera_pipeline_p95_ms",
    "performance.end_to_end_control_latency_p95_ms",
    "performance.continuous_run_duration_min",
    "vision_acceptance.orange_precision",
    "vision_acceptance.orange_recall",
    "vision_acceptance.purple_precision",
    "vision_acceptance.purple_recall",
    "vision_acceptance.line_loss_rate",
    "vision_acceptance.grasp_success_rate",
    "vision_acceptance.placement_success_rate",
)


def _lookup(data: dict[str, Any], dotted_path: str) -> Any:
    value: Any = data
    for part in dotted_path.split("."):
        if not isinstance(value, dict) or part not in value:
            return None
        value = value[part]
    return value


@dataclass(frozen=True)
class HardwareReadiness:
    ready: bool
    missing: tuple[str, ...]
    not_verified: tuple[str, ...]


class HardwareMeasurementConfig:
    """Load measured values while refusing to treat placeholders as calibration."""

    def __init__(self, data: dict[str, Any], source_path: Path | None = None) -> None:
        self.data = data
        self.source_path = source_path

    @classmethod
    def from_json(cls, path: str | Path) -> "HardwareMeasurementConfig":
        source = Path(path)
        raw = json.loads(source.read_text(encoding="utf-8"))
        if not isinstance(raw, dict):
            raise ValueError("hardware measurement file must contain a JSON object")
        return cls(raw, source)

    def readiness(self, required: Iterable[str] = DEFAULT_REQUIRED_MEASUREMENTS) -> HardwareReadiness:
        missing = []
        not_verified = []
        for path in required:
            value = _lookup(self.data, path)
            if value is None or value == "":
                missing.append(path)
        for section in ("platform", "cameras", "field_tags", "placement",
                        "lower_controller", "safety", "performance", "vision_acceptance"):
            value = self.data.get(section)
            if isinstance(value, dict) and value.get("verified") is False:
                not_verified.append(section)
        ready = not missing and not not_verified and self.data.get("status") == "MEASURED_AND_VERIFIED"
        return HardwareReadiness(ready, tuple(missing), tuple(not_verified))

    def get(self, dotted_path: str, default: Any = None) -> Any:
        value = _lookup(self.data, dotted_path)
        return default if value is None else value

    def require_ready(self) -> None:
        result = self.readiness()
        if result.ready:
            return
        details = []
        if self.data.get("status") != "MEASURED_AND_VERIFIED":
            details.append("status is not MEASURED_AND_VERIFIED")
        if result.missing:
            details.append("missing: " + ", ".join(result.missing))
        if result.not_verified:
            details.append("unverified sections: " + ", ".join(result.not_verified))
        raise RuntimeError("hardware configuration is not competition-ready; " + "; ".join(details))
