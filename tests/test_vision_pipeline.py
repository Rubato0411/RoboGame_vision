from pathlib import Path
import sys
import unittest

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.apriltag_detector import AprilTagConfig, AprilTagDetector  # noqa: E402
from src.black_line_detector import BlackLineConfig, BlackLineDetector  # noqa: E402
from src.block_pose_estimator import BlockPoseEstimate  # noqa: E402
from src.coordinate_transform import RigidTransform  # noqa: E402
from src.block_pose_estimator import BlockPoseEstimate  # noqa: E402
from src.coordinate_transform import RigidTransform  # noqa: E402
from src.block_detector_robust import BlockDetector  # noqa: E402
from src.image_source import FramePacket  # noqa: E402
from src.stream_health import StreamHealthConfig, StreamHealthMonitor  # noqa: E402
from src.temporal_tracker import TemporalObjectTracker, TemporalTrackerConfig  # noqa: E402
from src.vision_output import VisionMode  # noqa: E402
from src.vision_pipeline import VisionPipeline  # noqa: E402


class VisionPipelineTests(unittest.TestCase):
    def make_pipeline(self):
        tracker_config = TemporalTrackerConfig(confirmation_hits=1)
        return VisionPipeline(
            AprilTagDetector(AprilTagConfig()),
            BlockDetector.from_json(ROOT / "configs" / "block_detector_robust.json"),
            BlackLineDetector(BlackLineConfig(roi_normalized=(0, 0, 1, 1))),
            TemporalObjectTracker(tracker_config), TemporalObjectTracker(tracker_config),
            StreamHealthMonitor(StreamHealthConfig(expected_fps=30)), None,
        )

    def frame(self, image, frame_id=0):
        return FramePacket(image, frame_id/30, frame_id, "synthetic")

    def test_idle_runs_no_detectors_and_serializes(self):
        image = np.full((480, 640, 3), 180, np.uint8)
        output = self.make_pipeline().process(self.frame(image), VisionMode.IDLE)
        self.assertEqual(output.tags, ())
        self.assertEqual(output.blocks, ())
        self.assertFalse(output.line.valid)
        self.assertIn('"schema_version":"1.2"', output.to_json())

    def test_find_blocks_outputs_stable_block(self):
        image = np.full((480, 640, 3), 180, np.uint8)
        cv2.rectangle(image, (200, 180), (340, 330), (0, 140, 255), -1)
        output = self.make_pipeline().process(self.frame(image), VisionMode.FIND_BLOCKS)
        self.assertEqual(len(output.blocks), 1)
        self.assertEqual(output.blocks[0].color, "orange")
        self.assertIsNone(output.blocks[0].position_robot_m)

    def test_dynamic_gripper_pose_gates_robot_coordinates(self):
        class FakePoseEstimator:
            def estimate(self, detection, transform_robot_camera=None):
                self_transform = transform_robot_camera
                return BlockPoseEstimate(
                    True, tuple(self_transform.translation),
                    tuple(self_transform.translation), (0.0, 0.0), 0.0, "ok")

        image = np.full((480, 640, 3), 180, np.uint8)
        cv2.rectangle(image, (200, 180), (340, 330), (0, 140, 255), -1)
        pipeline = self.make_pipeline()
        pipeline.block_pose_estimator = FakePoseEstimator()
        pipeline.dynamic_camera_transform_required = True
        without_pose = pipeline.process(self.frame(image), VisionMode.FIND_BLOCKS)
        self.assertIsNone(without_pose.blocks[0].position_robot_m)

        pipeline.reset()
        transform = RigidTransform.from_xyz_rpy([0.2, -0.1, 0.4], [0, 0, 0])
        with_pose = pipeline.process(
            self.frame(image, 1), VisionMode.FIND_BLOCKS,
            transform_robot_camera=transform)
        np.testing.assert_allclose(with_pose.blocks[0].position_robot_m,
                                   transform.translation)

    def test_dynamic_gripper_pose_gates_robot_coordinates(self):
        class FakePoseEstimator:
            def estimate(self, detection, transform_robot_camera=None):
                self_transform = transform_robot_camera
                return BlockPoseEstimate(
                    True, tuple(self_transform.translation),
                    tuple(self_transform.translation), (0.0, 0.0), 0.0, "ok")

        image = np.full((480, 640, 3), 180, np.uint8)
        cv2.rectangle(image, (200, 180), (340, 330), (0, 140, 255), -1)
        pipeline = self.make_pipeline()
        pipeline.block_pose_estimator = FakePoseEstimator()
        pipeline.dynamic_camera_transform_required = True
        without_pose = pipeline.process(self.frame(image), VisionMode.FIND_BLOCKS)
        self.assertIsNone(without_pose.blocks[0].position_robot_m)

        pipeline.reset()
        transform = RigidTransform.from_xyz_rpy([0.2, -0.1, 0.4], [0, 0, 0])
        with_pose = pipeline.process(
            self.frame(image, 1), VisionMode.FIND_BLOCKS,
            transform_robot_camera=transform)
        np.testing.assert_allclose(with_pose.blocks[0].position_robot_m,
                                   transform.translation)

    def test_localization_outputs_tag_without_pose_when_unconfigured(self):
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        marker = cv2.aruco.generateImageMarker(dictionary, 3, 180)
        image = np.full((480, 640, 3), 220, np.uint8)
        image[140:320, 230:410] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
        output = self.make_pipeline().process(self.frame(image), VisionMode.LOCALIZATION)
        self.assertEqual([item.tag_id for item in output.tags], [3])
        self.assertFalse(output.robot_pose.valid)

    def test_follow_line_only_outputs_line(self):
        image = np.full((480, 640, 3), 190, np.uint8)
        cv2.line(image, (320, 479), (320, 0), (20, 20, 20), 24)
        output = self.make_pipeline().process(self.frame(image), VisionMode.FOLLOW_LINE)
        self.assertTrue(output.line.valid)
        self.assertEqual(output.tags, ())
        self.assertEqual(output.blocks, ())


if __name__ == "__main__":
    unittest.main()
