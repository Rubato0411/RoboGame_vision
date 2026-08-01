from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.hardware_config import HardwareMeasurementConfig  # noqa: E402


class HardwareConfigTests(unittest.TestCase):
    def test_repository_measurements_remain_blocked_by_unfinished_3d_hardware(self):
        config = HardwareMeasurementConfig.from_json(
            ROOT / "configs" / "hardware_measurements.json")
        result = config.readiness()
        self.assertFalse(result.ready)
        self.assertIn("cameras.gripper.calibration_file", result.missing)
        self.assertIn("cameras.gripper.coordinate_geometry_file", result.missing)
        self.assertIn("cameras.gripper.robot_from_camera.translation_m", result.missing)
        self.assertIn("cameras.gripper.robot_from_camera.rpy_deg", result.missing)
        with self.assertRaises(RuntimeError):
            config.require_ready()

    def test_explicit_measured_status_can_pass_custom_required_subset(self):
        config = HardwareMeasurementConfig({
            "status": "MEASURED_AND_VERIFIED",
            "platform": {"verified": True, "python_version": "3.11"},
        })
        result = config.readiness(("platform.python_version",))
        self.assertTrue(result.ready)

