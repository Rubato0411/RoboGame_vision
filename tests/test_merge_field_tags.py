from pathlib import Path
import json
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


class MergeFieldTagsTests(unittest.TestCase):
    def test_preserves_camera_and_merges_six_tags(self):
        with tempfile.TemporaryDirectory() as folder:
            coordinates = Path(folder) / "coordinate_front.json"
            coordinates.write_text(json.dumps({
                "robot_from_camera": {
                    "configured": True,
                    "translation_m": [1, 2, 3],
                    "rpy_deg": [4, 5, 6],
                },
                "field_from_tags": {},
            }), encoding="utf-8")
            result = subprocess.run([
                sys.executable, str(ROOT / "tools" / "merge_field_tags.py"),
                "--coordinates", str(coordinates),
                "--tags", str(ROOT / "configs" / "field_tags_measured.json"),
            ], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr + result.stdout)
            merged = json.loads(coordinates.read_text(encoding="utf-8"))
            self.assertEqual(
                merged["robot_from_camera"]["translation_m"], [1, 2, 3])
            self.assertEqual(set(merged["field_from_tags"]), set("123456"))
            self.assertTrue(coordinates.with_suffix(".json.bak").is_file())


if __name__ == "__main__":
    unittest.main()
