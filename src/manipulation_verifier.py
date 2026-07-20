from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class ManipulationPhase(str, Enum):
    VERIFY_GRASP = "VERIFY_GRASP"
    VERIFY_RELEASE = "VERIFY_RELEASE"
    VERIFY_BUILD = "VERIFY_BUILD"


@dataclass(frozen=True)
class ManipulationEvidence:
    gripper_closed: bool = False
    gripper_open: bool = False
    contact_detected: bool = False
    pressure_value: float | None = None
    target_visible: bool = False
    target_moved_with_gripper: bool | None = None
    target_in_placement_slot: bool | None = None
    structure_stable: bool | None = None


@dataclass(frozen=True)
class ManipulationVerification:
    valid: bool
    success: bool
    phase: str
    confidence: float
    reason: str


class ManipulationVerifier:
    """Fuse visual evidence with gripper sensors; never relies on disappearance alone."""

    def verify(self, phase: ManipulationPhase | str,
               evidence: ManipulationEvidence) -> ManipulationVerification:
        selected = ManipulationPhase(phase)
        if selected == ManipulationPhase.VERIFY_GRASP:
            sensor_ok = evidence.gripper_closed and evidence.contact_detected
            motion_ok = evidence.target_moved_with_gripper is True
            success = sensor_ok and motion_ok
            confidence = (0.45 if evidence.gripper_closed else 0.0) + (
                0.30 if evidence.contact_detected else 0.0) + (0.25 if motion_ok else 0.0)
            reason = "sensor contact and visual motion agree" if success else (
                "waiting for contact" if not sensor_ok else "visual lift not confirmed")
        elif selected == ManipulationPhase.VERIFY_RELEASE:
            released = evidence.gripper_open and not evidence.contact_detected
            placed = evidence.target_in_placement_slot is True
            success = released and placed
            confidence = (0.55 if released else 0.0) + (0.45 if placed else 0.0)
            reason = "released inside placement slot" if success else (
                "gripper still holding target" if not released else "placement not visually confirmed")
        else:
            placed = evidence.target_in_placement_slot is True
            stable = evidence.structure_stable is True
            success = placed and stable
            confidence = (0.5 if placed else 0.0) + (0.5 if stable else 0.0)
            reason = "placement and structure stable" if success else "build verification incomplete"
        return ManipulationVerification(True, success, selected.value,
                                        float(confidence), reason)
