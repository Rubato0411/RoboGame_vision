from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.image_source import CameraConfig, ImageSource  # noqa: E402


class FakePicamera2:
    instances = []

    def __init__(self, camera_num):
        self.camera_num = camera_num
        self.configuration = None
        self.started = False
        self.closed = False
        self.frames = 0
        self.camera_controls = {"AfMode": None, "LensPosition": None}
        self.__class__.instances.append(self)

    def create_video_configuration(self, **kwargs):
        return kwargs

    def configure(self, configuration):
        self.configuration = configuration

    def start(self):
        self.started = True

    def capture_array(self, stream):
        self.frames += 1
        return np.full((48, 64, 3), self.camera_num, dtype=np.uint8)

    def capture_metadata(self):
        return {"ExposureTime": 8000, "AnalogueGain": 2.0,
                "ColourGains": (1.5, 1.4), "LensPosition": 2.5}

    def stop(self):
        self.started = False

    def close(self):
        self.closed = True


class Picamera2ImageSourceTests(unittest.TestCase):
    def setUp(self):
        FakePicamera2.instances.clear()

    @patch.object(ImageSource, "_picamera2_class", return_value=FakePicamera2)
    def test_csi_camera_controls_frame_and_release(self, _loader):
        config = CameraConfig(
            width=64, height=48, fps=30, backend="picamera2", warmup_frames=1,
            exposure_time_us=8000, analogue_gain=2.0,
            colour_gains=(1.5, 1.4), lens_position=2.5,
        )
        with ImageSource(1, config) as source:
            packet = source.read()
            self.assertEqual(packet.image.shape, (48, 64, 3))
            self.assertEqual(packet.source_name, "camera:1")
            self.assertEqual(source.properties()["backend"], "picamera2")
            camera = FakePicamera2.instances[0]
            controls = camera.configuration["controls"]
            self.assertFalse(controls["AeEnable"])
            self.assertFalse(controls["AwbEnable"])
            self.assertEqual(controls["ExposureTime"], 8000)
            self.assertEqual(controls["LensPosition"], 2.5)
        self.assertTrue(camera.closed)

    @patch.object(ImageSource, "_picamera2_class", return_value=FakePicamera2)
    def test_two_csi_indices_are_independent(self, _loader):
        config = CameraConfig(width=64, height=48, backend="picamera2", warmup_frames=0)
        with ImageSource(0, config) as front, ImageSource(1, config) as gripper:
            self.assertEqual(int(front.read().image[0, 0, 0]), 0)
            self.assertEqual(int(gripper.read().image[0, 0, 0]), 1)


if __name__ == "__main__":
    unittest.main()
