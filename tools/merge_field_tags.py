from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil


def load_object(path: Path) -> dict:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Merge measured field tags into an existing camera-coordinate file")
    parser.add_argument("--coordinates", required=True,
                        help="Existing coordinate_front.json containing robot_from_camera")
    parser.add_argument("--tags", default="configs/field_tags_measured.json")
    parser.add_argument("--output",
                        help="Output path; default overwrites --coordinates after making .bak")
    args = parser.parse_args()

    coordinates_path = Path(args.coordinates)
    tags_path = Path(args.tags)
    if not coordinates_path.is_file():
        print(f"ERROR: coordinate file does not exist: {coordinates_path}")
        return 1
    if not tags_path.is_file():
        print(f"ERROR: measured tag file does not exist: {tags_path}")
        return 1

    coordinates = load_object(coordinates_path)
    measured = load_object(tags_path)
    camera = coordinates.get("robot_from_camera")
    if not isinstance(camera, dict) or not camera.get("configured", False):
        print("ERROR: robot_from_camera must already be configured; refusing to overwrite")
        return 2
    tags = measured.get("field_from_tags")
    if not isinstance(tags, dict) or set(tags) != {str(value) for value in range(1, 7)}:
        print("ERROR: measured tag file must contain configured IDs 1 through 6")
        return 2
    for tag_id, item in tags.items():
        if not isinstance(item, dict) or not item.get("configured", False):
            print(f"ERROR: tag {tag_id} is not configured")
            return 2

    output_path = Path(args.output) if args.output else coordinates_path
    if output_path.resolve() == coordinates_path.resolve():
        backup = coordinates_path.with_suffix(coordinates_path.suffix + ".bak")
        shutil.copy2(coordinates_path, backup)
        print(f"Backup: {backup}")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    coordinates["field_from_tags"] = {
        tag_id: {
            "configured": True,
            "translation_m": item["translation_m"],
            "rpy_deg": item["rpy_deg"],
        }
        for tag_id, item in sorted(tags.items(), key=lambda pair: int(pair[0]))
    }
    output_path.write_text(
        json.dumps(coordinates, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(f"Saved: {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
