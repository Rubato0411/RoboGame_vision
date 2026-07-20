from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.image_source import FramePacket  # noqa: E402
from src.stream_health import StreamHealthConfig, StreamHealthMonitor, StreamStatus  # noqa: E402


def packet(frame_id, timestamp, value=0, reconnect=0):
    image = np.full((120, 160, 3), value, np.uint8)
    return FramePacket(image, timestamp, frame_id, "test", reconnect)


class StreamHealthTests(unittest.TestCase):
    def make_monitor(self, **changes):
        values = dict(expected_fps=10, minimum_fps_ratio=.5, fps_window_frames=10,
                      startup_grace_frames=3, frame_timeout_s=.5,
                      disconnect_timeout_s=1.5, max_consecutive_failures=3,
                      freeze_threshold=.1, freeze_timeout_s=.4, recovery_frames=2)
        values.update(changes)
        return StreamHealthMonitor(StreamHealthConfig(**values), clock=lambda: 0)

    def test_health_does_not_depend_on_targets(self):
        monitor = self.make_monitor()
        monitor.observe(packet(0, 0, 0), now=0)
        monitor.observe(packet(1, .1, 1), now=.1)
        health = monitor.observe(packet(2, .2, 2), now=.2)
        self.assertEqual(health.status, StreamStatus.HEALTHY)

    def test_detects_timeout_and_disconnect(self):
        monitor = self.make_monitor()
        monitor.observe(packet(0, 0), now=0)
        self.assertEqual(monitor.check(.6).status, StreamStatus.TIMEOUT)
        self.assertEqual(monitor.check(1.6).status, StreamStatus.DISCONNECTED)

    def test_detects_frozen_frames(self):
        monitor = self.make_monitor()
        for index in range(6):
            health = monitor.observe(packet(index, index * .1, 20), now=index * .1)
        self.assertEqual(health.status, StreamStatus.FROZEN)

    def test_detects_low_fps(self):
        monitor = self.make_monitor(freeze_threshold=0)
        for index in range(3):
            health = monitor.observe(packet(index, index, index), now=index)
        self.assertEqual(health.status, StreamStatus.LOW_FPS)

    def test_stale_frame_metadata_is_rejected(self):
        monitor = self.make_monitor()
        monitor.observe(packet(2, 1), now=0)
        health = monitor.observe(packet(2, 1), now=.1)
        self.assertEqual(health.status, StreamStatus.STALE_FRAME)

    def test_reports_recovery_after_reconnect(self):
        monitor = self.make_monitor()
        monitor.observe(packet(0, 0), now=0)
        health = monitor.observe(packet(1, .1, 1, reconnect=1), now=.1)
        self.assertEqual(health.status, StreamStatus.RECOVERING)


if __name__ == "__main__":
    unittest.main()
