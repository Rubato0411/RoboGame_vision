from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from run_camera_monitor import compose_dashboard, render_camera  # noqa: E402
from src.realtime_camera_monitor import CameraSnapshot  # noqa: E402
from src.stream_health import StreamHealth, StreamStatus  # noqa: E402


class CameraMonitorTests(unittest.TestCase):
    def snapshot(self, role):
        health = StreamHealth(StreamStatus.STARTING, True, 0, 0, 0, 0, 0, "start")
        return CameraSnapshot(role, None, health, None)

    def test_waiting_panel_has_expected_size(self):
        self.assertEqual(render_camera(self.snapshot("front"), (320, 180)).shape,
                         (180, 320, 3))

    def test_two_camera_dashboard_is_horizontal(self):
        dashboard = compose_dashboard((self.snapshot("front"), self.snapshot("gripper")),
                                      (320, 180))
        self.assertEqual(dashboard.shape, (180, 640, 3))


if __name__ == "__main__": unittest.main()
