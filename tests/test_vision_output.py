import json
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.vision_output import StreamOutput, VisionMode, VisionOutput  # noqa: E402


class VisionOutputTests(unittest.TestCase):
    def test_json_has_schema_and_validity(self):
        output = VisionOutput(7, 12.5, "test", VisionMode.IDLE.value,
                              StreamOutput("HEALTHY", True, 30, 0, 0, "ok"))
        data = json.loads(output.to_json())
        self.assertEqual(data["schema_version"], "1.2")
        self.assertTrue(data["valid"])
        self.assertEqual(data["frame_id"], 7)

    def test_non_finite_values_become_null(self):
        output = VisionOutput(0, 0, "test", "IDLE",
                              StreamOutput("STARTING", True, 0, float("inf"), 0, "start"))
        self.assertIsNone(output.to_dict()["stream"]["frame_age_s"])


if __name__ == "__main__":
    unittest.main()
