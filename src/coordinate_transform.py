from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Iterable, Mapping

import cv2
import numpy as np

from .apriltag_detector import AprilTagDetection
from .camera_calibration import CalibrationResult


def _vector3(value) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64).reshape(3)
    if not np.all(np.isfinite(result)):
        raise ValueError("vector contains NaN or infinity")
    return result


def rotation_from_rpy(roll: float, pitch: float, yaw: float,
                      degrees: bool = False) -> np.ndarray:
    """Return Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    values = np.deg2rad([roll, pitch, yaw]) if degrees else np.array([roll, pitch, yaw])
    roll, pitch, yaw = values
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], np.float64)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], np.float64)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], np.float64)
    return rz @ ry @ rx


def rpy_from_rotation(rotation: np.ndarray, degrees: bool = False) -> tuple[float, float, float]:
    matrix = np.asarray(rotation, np.float64).reshape(3, 3)
    pitch = np.arctan2(-matrix[2, 0], np.hypot(matrix[0, 0], matrix[1, 0]))
    if abs(np.cos(pitch)) > 1e-8:
        roll = np.arctan2(matrix[2, 1], matrix[2, 2])
        yaw = np.arctan2(matrix[1, 0], matrix[0, 0])
    else:
        roll = 0.0
        yaw = np.arctan2(-matrix[0, 1], matrix[1, 1])
    values = np.array([roll, pitch, yaw])
    if degrees:
        values = np.rad2deg(values)
    return tuple(float(v) for v in values)


@dataclass(frozen=True)
class RigidTransform:
    """Rigid transform T_target_source: source coordinates -> target coordinates."""

    rotation: np.ndarray
    translation: np.ndarray

    def __post_init__(self):
        rotation = np.asarray(self.rotation, np.float64).reshape(3, 3)
        translation = _vector3(self.translation)
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6):
            raise ValueError("rotation matrix is not orthonormal")
        if np.linalg.det(rotation) < 0.999999:
            raise ValueError("rotation matrix must have determinant +1")
        object.__setattr__(self, "rotation", rotation)
        object.__setattr__(self, "translation", translation)

    @classmethod
    def identity(cls) -> "RigidTransform":
        return cls(np.eye(3), np.zeros(3))

    @classmethod
    def from_rvec_tvec(cls, rvec, tvec) -> "RigidTransform":
        rotation, _ = cv2.Rodrigues(_vector3(rvec))
        return cls(rotation, _vector3(tvec))

    @classmethod
    def from_xyz_rpy(cls, xyz, rpy, degrees: bool = True) -> "RigidTransform":
        return cls(rotation_from_rpy(*rpy, degrees=degrees), _vector3(xyz))

    @classmethod
    def from_matrix(cls, matrix) -> "RigidTransform":
        value = np.asarray(matrix, np.float64).reshape(4, 4)
        if not np.allclose(value[3], [0, 0, 0, 1], atol=1e-8):
            raise ValueError("invalid homogeneous matrix")
        return cls(value[:3, :3], value[:3, 3])

    def as_matrix(self) -> np.ndarray:
        result = np.eye(4, dtype=np.float64)
        result[:3, :3] = self.rotation
        result[:3, 3] = self.translation
        return result

    def inverse(self) -> "RigidTransform":
        inverse_rotation = self.rotation.T
        return RigidTransform(inverse_rotation, -inverse_rotation @ self.translation)

    def compose(self, other: "RigidTransform") -> "RigidTransform":
        """Return self @ other, e.g. T_A_B.compose(T_B_C) -> T_A_C."""
        return RigidTransform(self.rotation @ other.rotation,
                              self.rotation @ other.translation + self.translation)

    def apply(self, points) -> np.ndarray:
        values = np.asarray(points, np.float64)
        if values.shape[-1] != 3:
            raise ValueError("points must end with three coordinates")
        return values @ self.rotation.T + self.translation


@dataclass(frozen=True)
class RobotPoseEstimate:
    transform_field_robot: RigidTransform
    used_tag_ids: tuple[int, ...]
    rejected_tag_ids: tuple[int, ...]

    @property
    def xyz_m(self) -> tuple[float, float, float]:
        return tuple(float(v) for v in self.transform_field_robot.translation)

    @property
    def rpy_deg(self) -> tuple[float, float, float]:
        return rpy_from_rotation(self.transform_field_robot.rotation, degrees=True)


class CoordinateTransformer:
    def __init__(self, calibration: CalibrationResult,
                 transform_robot_camera: RigidTransform,
                 transforms_field_tag: Mapping[int, RigidTransform]) -> None:
        self.calibration = calibration
        self.transform_robot_camera = transform_robot_camera
        self.transforms_field_tag = dict(transforms_field_tag)

    @classmethod
    def from_json(cls, calibration_path: str | Path, geometry_path: str | Path):
        calibration = CalibrationResult.load_json(calibration_path)
        raw = json.loads(Path(geometry_path).read_text(encoding="utf-8"))
        camera = raw["robot_from_camera"]
        if not camera.get("configured", False):
            raise ValueError("robot_from_camera is not configured")
        transform_robot_camera = RigidTransform.from_xyz_rpy(
            camera["translation_m"], camera["rpy_deg"], degrees=True)
        tags = {}
        for key, value in raw.get("field_from_tags", {}).items():
            if value.get("configured", False):
                tags[int(key)] = RigidTransform.from_xyz_rpy(
                    value["translation_m"], value["rpy_deg"], degrees=True)
        if not tags:
            raise ValueError("no field tag poses are configured")
        return cls(calibration, transform_robot_camera, tags)

    def robot_pose_from_tag(self, detection: AprilTagDetection) -> RigidTransform:
        """T_field_robot from one detection containing T_camera_tag."""
        if detection.rvec is None or detection.tvec_m is None:
            raise ValueError("AprilTag detection has no pose; provide camera calibration")
        if detection.tag_id not in self.transforms_field_tag:
            raise KeyError(f"field pose for tag {detection.tag_id} is not configured")
        transform_camera_tag = RigidTransform.from_rvec_tvec(detection.rvec, detection.tvec_m)
        transform_field_camera = self.transforms_field_tag[detection.tag_id].compose(
            transform_camera_tag.inverse())
        return transform_field_camera.compose(self.transform_robot_camera.inverse())

    def estimate_robot_pose(self, detections: Iterable[AprilTagDetection],
                            max_translation_residual_m: float = 0.35) -> RobotPoseEstimate:
        candidates = []
        for detection in detections:
            if (detection.tag_id in self.transforms_field_tag and
                    detection.rvec is not None and detection.tvec_m is not None):
                weight = 1.0 / max((detection.reprojection_error_px or 1.0) ** 2, 0.01)
                candidates.append((detection.tag_id, self.robot_pose_from_tag(detection), weight))
        if not candidates:
            raise ValueError("no usable configured AprilTag poses")
        translations = np.stack([item[1].translation for item in candidates])
        median = np.median(translations, axis=0)
        residuals = np.linalg.norm(translations - median, axis=1)
        keep = residuals <= max_translation_residual_m
        if not np.any(keep):
            keep[int(np.argmin(residuals))] = True
        accepted = [item for item, use in zip(candidates, keep) if use]
        rejected = [item[0] for item, use in zip(candidates, keep) if not use]
        weights = np.asarray([item[2] for item in accepted], np.float64)
        weights /= weights.sum()
        translation = np.sum(np.stack([item[1].translation for item in accepted]) * weights[:, None], axis=0)
        matrix = sum(weight * item[1].rotation for weight, item in zip(weights, accepted))
        u, _, vt = np.linalg.svd(matrix)
        rotation = u @ vt
        if np.linalg.det(rotation) < 0:
            u[:, -1] *= -1
            rotation = u @ vt
        return RobotPoseEstimate(RigidTransform(rotation, translation),
                                 tuple(item[0] for item in accepted), tuple(rejected))

    def pixel_to_camera_ray(self, pixel_xy) -> np.ndarray:
        pixel = np.asarray(pixel_xy, np.float64).reshape(1, 1, 2)
        normalized = cv2.undistortPoints(pixel, self.calibration.camera_matrix,
                                         self.calibration.distortion_coefficients).reshape(2)
        ray = np.array([normalized[0], normalized[1], 1.0])
        return ray / np.linalg.norm(ray)

    def pixel_to_plane(self, pixel_xy, transform_target_camera: RigidTransform,
                       plane_normal_target=(0, 0, 1), plane_offset: float = 0.0) -> np.ndarray:
        """Intersect a camera pixel ray with n·X + offset = 0 in target frame."""
        ray_camera = self.pixel_to_camera_ray(pixel_xy)
        origin = transform_target_camera.translation
        direction = transform_target_camera.rotation @ ray_camera
        normal = _vector3(plane_normal_target)
        denominator = float(normal @ direction)
        if abs(denominator) < 1e-9:
            raise ValueError("pixel ray is parallel to plane")
        distance = -(float(normal @ origin) + plane_offset) / denominator
        if distance <= 0:
            raise ValueError("plane intersection is behind the camera")
        return origin + distance * direction
