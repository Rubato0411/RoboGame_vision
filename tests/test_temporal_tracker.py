from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.apriltag_detector import AprilTagDetection  # noqa: E402
from src.temporal_tracker import (TemporalObjectTracker, TemporalTrackerConfig,
                                  observations_from_apriltags)  # noqa: E402


def tag(x, y, tag_id=1):
    corners = ((x-5, y-5), (x+5, y-5), (x+5, y+5), (x-5, y+5))
    return AprilTagDetection(tag_id, corners, (x, y), 100, 40)


class TemporalTrackerTests(unittest.TestCase):
    def make_tracker(self, **changes):
        values = dict(confirmation_hits=3, max_missed_frames=2,
                      smoothing_alpha=.5, max_center_jump_px=50)
        values.update(changes)
        return TemporalObjectTracker(TemporalTrackerConfig(**values))

    def test_requires_consecutive_hits(self):
        tracker = self.make_tracker()
        self.assertEqual(tracker.update(observations_from_apriltags([tag(100, 100)])).tracks, ())
        self.assertEqual(tracker.update(observations_from_apriltags([tag(102, 100)])).tracks, ())
        result = tracker.update(observations_from_apriltags([tag(104, 100)]))
        self.assertEqual(len(result.tracks), 1)
        self.assertEqual(result.appeared_track_ids, (1,))

    def test_smooths_center_jitter(self):
        tracker = self.make_tracker(confirmation_hits=1)
        tracker.update(observations_from_apriltags([tag(100, 100)]))
        result = tracker.update(observations_from_apriltags([tag(110, 100)]))
        self.assertAlmostEqual(result.tracks[0].center_px[0], 105)

    def test_keeps_confirmed_track_through_short_occlusion(self):
        tracker = self.make_tracker(confirmation_hits=1)
        tracker.update(observations_from_apriltags([tag(100, 100)]))
        result = tracker.update([])
        self.assertEqual(len(result.tracks), 1)
        self.assertTrue(result.tracks[0].predicted)

    def test_removes_track_after_miss_limit(self):
        tracker = self.make_tracker(confirmation_hits=1, max_missed_frames=1)
        tracker.update(observations_from_apriltags([tag(100, 100)]))
        tracker.update([])
        result = tracker.update([])
        self.assertEqual(result.tracks, ())
        self.assertEqual(result.lost_track_ids, (1,))

    def test_rejects_impossible_same_id_jump(self):
        tracker = self.make_tracker(confirmation_hits=1, max_center_jump_px=20)
        tracker.update(observations_from_apriltags([tag(100, 100)]))
        result = tracker.update(observations_from_apriltags([tag(300, 300)]))
        self.assertEqual(result.rejected_jumps, 1)
        self.assertEqual(len(result.tracks), 1)
        self.assertTrue(result.tracks[0].predicted)


if __name__ == "__main__":
    unittest.main()
