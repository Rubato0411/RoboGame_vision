from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .block_detector_robust import BlockDetection
from .coordinate_transform import CoordinateTransformer, RigidTransform


@dataclass(frozen=True)
class BlockPoseEstimate:
    valid: bool
    center_robot_m: tuple[float, float, float] | None
    grasp_point_robot_m: tuple[float, float, float] | None
    contact_pixel: tuple[float, float]
    yaw_image_deg: float | None
    reason: str


class BlockPoseEstimator:
    """Estimate a cube center from its image contact point and a support plane."""

    def __init__(self, coordinates: CoordinateTransformer, cube_size_m: float = 0.10,
                 support_plane_height_m: float = 0.0) -> None:
        if cube_size_m <= 0:
            raise ValueError("cube_size_m must be positive")
        self.coordinates = coordinates
        self.cube_size_m = cube_size_m
        self.support_plane_height_m = support_plane_height_m

    def estimate(self, detection: BlockDetection,
                 transform_robot_camera: RigidTransform | None = None) -> BlockPoseEstimate:
        x, y, width, height = detection.bbox
        contact_pixel = (x + width/2.0, y + height - 1.0)
        try:
            contact = self.coordinates.pixel_to_plane(
                contact_pixel, transform_robot_camera or self.coordinates.transform_robot_camera,
                plane_normal_target=(0, 0, 1),
                plane_offset=-self.support_plane_height_m,
            )
        except ValueError as exc:
            return BlockPoseEstimate(False, None, None, contact_pixel, None, str(exc))
        center = np.asarray(contact, np.float64)
        center[2] += self.cube_size_m/2.0
        grasp = center.copy()
        grasp[2] += self.cube_size_m/2.0
        return BlockPoseEstimate(
            True, tuple(float(v) for v in center), tuple(float(v) for v in grasp),
            contact_pixel, float(detection.angle_deg), "ok",
        )
