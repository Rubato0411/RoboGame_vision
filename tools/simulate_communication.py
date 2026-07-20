from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.raspberry_pi_endpoint import RaspberryPiVisionEndpoint  # noqa: E402
from src.simulated_lower_controller import SimulatedLowerController  # noqa: E402
from src.vision_output import VisionMode  # noqa: E402


def chunks(data: bytes, size: int):
    for start in range(0, len(data), size):
        yield data[start:start+size]


def main() -> int:
    parser = argparse.ArgumentParser(description="Simulate Raspberry Pi <-> lower controller protocol")
    parser.add_argument("--jsonl", required=True, help="VisionPipeline JSONL output")
    parser.add_argument("--mode", choices=[item.value for item in VisionMode], default="FIND_BLOCKS")
    parser.add_argument("--chunk-size", type=int, default=7, help="Simulated fragmented serial reads")
    args = parser.parse_args()
    if args.chunk_size < 1:
        parser.error("chunk-size must be positive")

    lines = [line for line in Path(args.jsonl).read_text(encoding="utf-8").splitlines() if line.strip()]
    if not lines:
        raise ValueError("JSONL file is empty")
    vision = json.loads(lines[-1])
    pi = RaspberryPiVisionEndpoint()
    lower = SimulatedLowerController()

    command = lower.command_mode(args.mode)
    responses = []
    for piece in chunks(command, args.chunk_size):
        result = pi.feed_received(piece)
        responses.extend(result.responses)
        for event in result.events:
            print(f"PI EVENT: {event.kind} seq={event.sequence} payload={event.payload}")
    for response in responses:
        for event in lower.feed_from_pi(response, now=0.0):
            print(f"LOWER EVENT: {event.kind} seq={event.sequence} payload={event.payload}")

    encoded_vision = pi.encode_vision(vision)
    for piece in chunks(encoded_vision, args.chunk_size):
        for event in lower.feed_from_pi(piece, now=0.1):
            print(f"LOWER EVENT: {event.kind} seq={event.sequence} "
                  f"frame={event.payload.get('frame_id')}")
    print(f"pi_mode={pi.mode.value}")
    print(f"vision_usable_at_0.2s={lower.vision_usable(now=0.2)}")
    print(f"vision_usable_at_0.5s={lower.vision_usable(now=0.5)}")
    print(f"pi_decoder={pi.decoder.stats}")
    print(f"lower_decoder={lower.decoder.stats}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
