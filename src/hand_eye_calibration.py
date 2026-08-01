from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Iterable, Mapping

import cv2
import numpy as np

from .coordinate_transform import RigidTransform, rpy_from_rotation


_METHODS = {
    "TSAI": cv2.CALIB_HAND_EYE_TSAI,
    "PARK": cv2.CALIB_HAND_EYE_PARK,
    "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
    "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
    "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def _rotation_error_deg(left: np.ndarray, right: np.ndarray) -> float:
    relative = left.T @ right
    cosine = np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.rad2deg(np.arccos(cosine)))


def _average_rotation(rotations: Iterable[np.ndarray]) -> np.ndarray:
    matrix = sum(np.asarray(item, np.float64) for item in rotations)
    u, _, vt = np.linalg.svd(matrix)
    result = u @ vt
    if np.linalg.det(result) < 0:
        u[:, -1] *= -1
        result = u @ vt
    return result


def transform_from_json(value: Mapping) -> RigidTransform:
    if value.get("translation_m") is not None and value.get("rpy_deg") is not None:
        return RigidTransform.from_xyz_rpy(value["translation_m"], value["rpy_deg"])
    if value.get("tvec_m") is not None and value.get("rvec") is not None:
        return RigidTransform.from_rvec_tvec(value["rvec"], value["tvec_m"])
    raise ValueError("transform requires translation_m+rpy_deg or tvec_m+rvec")


@dataclass(frozen=True)
class HandEyeSample:
    sample_id: str
    transform_base_gripper: RigidTransform
    transform_camera_target: RigidTransform


@dataclass(frozen=True)
class HandEyeCalibrationResult:
    method: str
    transform_gripper_camera: RigidTransform
    sample_count: int
    translation_rms_m: float
    translation_max_m: float
    rotation_rms_deg: float
    rotation_max_deg: float
    gripper_translation_span_m: float
    gripper_rotation_span_deg: float

    def to_dict(self) -> dict:
        transform = self.transform_gripper_camera
        return {
            "format_version": 1,
            "calibration_type": "eye_in_hand",
            "convention": "T_gripper_camera maps camera coordinates into gripper coordinates",
            "method": self.method,
            "gripper_from_camera": {
                "configured": True,
                "translation_m": [float(v) for v in transform.translation],
                "rpy_deg": list(rpy_from_rotation(transform.rotation, degrees=True)),
                "matrix_4x4": transform.as_matrix().tolist(),
            },
            "quality": {
                "sample_count": self.sample_count,
                "fixed_target_translation_rms_m": self.translation_rms_m,
                "fixed_target_translation_max_m": self.translation_max_m,
                "fixed_target_rotation_rms_deg": self.rotation_rms_deg,
                "fixed_target_rotation_max_deg": self.rotation_max_deg,
                "gripper_translation_span_m": self.gripper_translation_span_m,
                "gripper_rotation_span_deg": self.gripper_rotation_span_deg,
            },
        }

    def save_json(self, path: str | Path) -> None:
        output = Path(path)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(self.to_dict(), ensure_ascii=False, indent=2) + "\n",
                          encoding="utf-8")


def load_hand_eye_samples(path: str | Path) -> tuple[HandEyeSample, ...]:
    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    if raw.get("mode") != "eye_in_hand":
        raise ValueError("sample file mode must be eye_in_hand")
    if raw.get("length_unit", "m") != "m":
        raise ValueError("sample file length_unit must be m")
    samples = []
    for index, item in enumerate(raw.get("samples", [])):
        samples.append(HandEyeSample(
            str(item.get("id", f"sample_{index + 1:02d}")),
            transform_from_json(item["base_from_gripper"]),
            transform_from_json(item["camera_from_target"]),
        ))
    if len(samples) < 5:
        raise ValueError("hand-eye calibration requires at least 5 paired poses; 12-20 recommended")
    return tuple(samples)


def load_gripper_from_camera(path: str | Path) -> RigidTransform:
    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    if raw.get("calibration_type") != "eye_in_hand":
        raise ValueError("hand-eye result calibration_type must be eye_in_hand")
    value = raw.get("gripper_from_camera")
    if not isinstance(value, dict) or not value.get("configured", False):
        raise ValueError("gripper_from_camera is not configured")
    return transform_from_json(value)


def calibrate_eye_in_hand(samples: Iterable[HandEyeSample],
                          method: str = "PARK") -> HandEyeCalibrationResult:
    values = tuple(samples)
    if len(values) < 5:
        raise ValueError("hand-eye calibration requires at least 5 paired poses; 12-20 recommended")
    selected_method = method.upper()
    if selected_method not in _METHODS:
        raise ValueError(f"unknown hand-eye method: {method}")

    gripper_translations = np.stack(
        [sample.transform_base_gripper.translation for sample in values])
    translation_span = float(np.max(np.linalg.norm(
        gripper_translations[:, None, :] - gripper_translations[None, :, :], axis=2)))
    rotation_span = max(
        _rotation_error_deg(left.transform_base_gripper.rotation,
                            right.transform_base_gripper.rotation)
        for left in values for right in values)
    if translation_span < 0.03:
        raise ValueError("gripper translation span is below 0.03 m; collect more diverse poses")
    if rotation_span < 15.0:
        raise ValueError("gripper rotation span is below 15 deg; collect more diverse poses")

    rotation, translation = cv2.calibrateHandEye(
        [item.transform_base_gripper.rotation for item in values],
        [item.transform_base_gripper.translation.reshape(3, 1) for item in values],
        [item.transform_camera_target.rotation for item in values],
        [item.transform_camera_target.translation.reshape(3, 1) for item in values],
        method=_METHODS[selected_method],
    )
    transform_gripper_camera = RigidTransform(rotation, translation)

    # With a fixed target, every sample should produce the same T_base_target.
    target_estimates = [
        item.transform_base_gripper.compose(transform_gripper_camera).compose(
            item.transform_camera_target)
        for item in values
    ]
    average_translation = np.mean(
        np.stack([item.translation for item in target_estimates]), axis=0)
    average_target_rotation = _average_rotation(item.rotation for item in target_estimates)
    translation_errors = np.asarray([
        np.linalg.norm(item.translation - average_translation) for item in target_estimates])
    rotation_errors = np.asarray([
        _rotation_error_deg(average_target_rotation, item.rotation) for item in target_estimates])
    return HandEyeCalibrationResult(
        selected_method, transform_gripper_camera, len(values),
        float(np.sqrt(np.mean(translation_errors ** 2))),
        float(np.max(translation_errors)),
        float(np.sqrt(np.mean(rotation_errors ** 2))),
        float(np.max(rotation_errors)), translation_span, rotation_span,
    )
