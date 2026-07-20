from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.apriltag_detector import AprilTagDetection  # noqa: E402
from src.camera_calibration import CalibrationResult, ChessboardSpec  # noqa: E402
from src.coordinate_transform import (CoordinateTransformer, RigidTransform,
                                      rotation_from_rpy, rpy_from_rotation)  # noqa: E402


def calibration():
    return CalibrationResult(640, 480, np.array([[500., 0., 320.], [0., 500., 240.], [0., 0., 1.]]),
                             np.zeros(5), 0., 0., (), (), (), ChessboardSpec())


class CoordinateTransformTests(unittest.TestCase):
    def test_inverse_and_compose_identity(self):
        transform = RigidTransform.from_xyz_rpy([1, 2, 3], [10, 20, 30])
        identity = transform.compose(transform.inverse()).as_matrix()
        np.testing.assert_allclose(identity, np.eye(4), atol=1e-9)

    def test_point_round_trip(self):
        transform = RigidTransform.from_xyz_rpy([.2, -.1, 1], [5, -20, 35])
        point = np.array([.4, .3, -.2])
        np.testing.assert_allclose(transform.inverse().apply(transform.apply(point)), point, atol=1e-9)

    def test_rpy_round_trip(self):
        rotation = rotation_from_rpy(10, -15, 30, degrees=True)
        np.testing.assert_allclose(rpy_from_rotation(rotation, degrees=True), [10, -15, 30], atol=1e-8)

    def test_robot_pose_from_single_tag(self):
        field_tag = RigidTransform.from_xyz_rpy([2, 1, .325], [0, 0, 180])
        expected_field_robot = RigidTransform.from_xyz_rpy([1.2, .5, 0], [0, 0, 20])
        robot_camera = RigidTransform.from_xyz_rpy([.1, 0, .4], [0, 0, 0])
        field_camera = expected_field_robot.compose(robot_camera)
        camera_tag = field_camera.inverse().compose(field_tag)
        rvec, _ = __import__('cv2').Rodrigues(camera_tag.rotation)
        detection = AprilTagDetection(1, ((0, 0),)*4, (0, 0), 1, 1,
                                      rvec.reshape(3), camera_tag.translation, 1, .2)
        transformer = CoordinateTransformer(calibration(), robot_camera, {1: field_tag})
        actual = transformer.robot_pose_from_tag(detection)
        np.testing.assert_allclose(actual.as_matrix(), expected_field_robot.as_matrix(), atol=1e-7)

    def test_pixel_to_ground_plane(self):
        transformer = CoordinateTransformer(calibration(), RigidTransform.identity(), {})
        # Camera at z=1 m, optical z axis points downward in the target frame.
        target_camera = RigidTransform(rotation_from_rpy(180, 0, 0, degrees=True), [0, 0, 1])
        point = transformer.pixel_to_plane((320, 240), target_camera)
        np.testing.assert_allclose(point, [0, 0, 0], atol=1e-8)

    def test_multi_tag_rejects_translation_outlier(self):
        transformer = CoordinateTransformer(calibration(), RigidTransform.identity(), {})
        poses = [RigidTransform.from_xyz_rpy([1, 2, 0], [0, 0, 10]),
                 RigidTransform.from_xyz_rpy([1.02, 1.98, 0], [0, 0, 11]),
                 RigidTransform.from_xyz_rpy([5, 8, 0], [0, 0, 80])]
        transformer.robot_pose_from_tag = lambda detection: poses[detection.tag_id - 1]
        transformer.transforms_field_tag = {1: poses[0], 2: poses[1], 3: poses[2]}
        detections = [AprilTagDetection(i, ((0, 0),)*4, (0, 0), 1, 1,
                                       np.zeros(3), np.ones(3), 1, 1) for i in (1, 2, 3)]
        result = transformer.estimate_robot_pose(detections, max_translation_residual_m=.2)
        self.assertEqual(result.used_tag_ids, (1, 2))
        self.assertEqual(result.rejected_tag_ids, (3,))


if __name__ == "__main__":
    unittest.main()
