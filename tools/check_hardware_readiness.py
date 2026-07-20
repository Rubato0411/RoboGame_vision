from __future__ import annotations

import argparse
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.hardware_config import HardwareMeasurementConfig  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description="List unresolved robot measurements")
    parser.add_argument(
        "--config", default=str(ROOT / "configs" / "hardware_measurements.json"))
    args = parser.parse_args()
    result = HardwareMeasurementConfig.from_json(args.config).readiness()
    print(f"ready={result.ready}")
    for path in result.missing:
        print(f"MISSING {path}")
    for section in result.not_verified:
        print(f"UNVERIFIED {section}")
    return 0 if result.ready else 2


if __name__ == "__main__":
    raise SystemExit(main())

