from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Iterable

from .apriltag_detector import AprilTagDetection
from .coordinate_transform import RigidTransform, rpy_from_rotation


@dataclass(frozen=True)
class PlacementSlot:
    slot_id: str
    reference_tag_id: int
    transform_tag_slot: RigidTransform
    layer: int = 0
    priority: int = 0


@dataclass(frozen=True)
class PlacementTarget:
    valid: bool
    slot_id: str | None
    reference_tag_id: int | None
    position_robot_m: tuple[float, float, float] | None
    rpy_robot_deg: tuple[float, float, float] | None
    layer: int | None
    reason: str


class PlacementTagLocator:
    """Locate configured building slots relative to observed AprilTags."""

    def __init__(self, slots: Iterable[PlacementSlot]) -> None:
        self.slots = tuple(slots)
        if len({slot.slot_id for slot in self.slots}) != len(self.slots):
            raise ValueError("placement slot ids must be unique")

    @classmethod
    def from_json(cls, path: str | Path):
        raw = json.loads(Path(path).read_text(encoding="utf-8"))
        slots = []
        for item in raw.get("slots", []):
            if not item.get("configured", False):
                continue
            slots.append(PlacementSlot(
                str(item["slot_id"]), int(item["reference_tag_id"]),
                RigidTransform.from_xyz_rpy(item["translation_tag_m"], item["rpy_tag_deg"]),
                int(item.get("layer", 0)), int(item.get("priority", 0)),
            ))
        return cls(slots)

    def locate(self, detections: Iterable[AprilTagDetection],
               transform_robot_camera: RigidTransform,
               occupied_slot_ids: Iterable[str] = ()) -> tuple[PlacementTarget, ...]:
        observed = {item.tag_id: item for item in detections
                    if item.rvec is not None and item.tvec_m is not None}
        occupied = set(occupied_slot_ids)
        targets = []
        for slot in sorted(self.slots, key=lambda value: (value.layer, value.priority, value.slot_id)):
            if slot.slot_id in occupied or slot.reference_tag_id not in observed:
                continue
            detection = observed[slot.reference_tag_id]
            transform_camera_tag = RigidTransform.from_rvec_tvec(detection.rvec, detection.tvec_m)
            transform_robot_slot = transform_robot_camera.compose(transform_camera_tag).compose(
                slot.transform_tag_slot)
            targets.append(PlacementTarget(
                True, slot.slot_id, slot.reference_tag_id,
                tuple(float(v) for v in transform_robot_slot.translation),
                rpy_from_rotation(transform_robot_slot.rotation, degrees=True),
                slot.layer, "tag and slot configured",
            ))
        return tuple(targets)

    def select_next(self, detections: Iterable[AprilTagDetection],
                    transform_robot_camera: RigidTransform,
                    occupied_slot_ids: Iterable[str] = ()) -> PlacementTarget:
        targets = self.locate(detections, transform_robot_camera, occupied_slot_ids)
        return targets[0] if targets else PlacementTarget(
            False, None, None, None, None, None,
            "no visible configured tag with an unoccupied slot")
