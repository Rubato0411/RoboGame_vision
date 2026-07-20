from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.multi_camera_manager import CameraRole, MultiCameraManager  # noqa: E402


class FakeSource:
    def __init__(self, value): self.value, self.released = value, False
    def read(self): return self.value
    def properties(self): return {"name": self.value}
    def release(self): self.released = True


class MultiCameraManagerTests(unittest.TestCase):
    def test_roles_have_independent_sources(self):
        front, gripper = FakeSource("front-frame"), FakeSource("gripper-frame")
        manager = MultiCameraManager({CameraRole.FRONT: front, CameraRole.GRIPPER: gripper})
        self.assertEqual(manager.read("front"), "front-frame")
        self.assertEqual(manager.read("gripper"), "gripper-frame")
        manager.release()
        self.assertTrue(front.released and gripper.released)


if __name__ == "__main__": unittest.main()
