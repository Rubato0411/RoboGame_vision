from __future__ import annotations

from dataclasses import dataclass
from time import monotonic

from .communication_protocol import (MessageType, PacketStreamDecoder,
                                     encode_json_packet)
from .vision_output import VisionMode


@dataclass(frozen=True)
class LowerControllerEvent:
    kind: str
    sequence: int
    payload: dict


class SimulatedLowerController:
    """PC-side STM32 substitute for protocol and timeout testing."""

    def __init__(self, vision_timeout_s: float = 0.30, clock=monotonic) -> None:
        self.vision_timeout_s = vision_timeout_s
        self.clock = clock
        self.decoder = PacketStreamDecoder()
        self.latest_vision: dict | None = None
        self.latest_vision_received_at: float | None = None
        self.latest_sequence: int | None = None
        self._tx_sequence = 0

    def command_mode(self, mode: VisionMode | str, request_id: int | None = None) -> bytes:
        selected = VisionMode(mode)
        sequence = self._next_sequence()
        return encode_json_packet(MessageType.MODE_COMMAND, sequence, {
            "mode": selected.value,
            "request_id": sequence if request_id is None else request_id,
        })

    def heartbeat(self, now: float | None = None) -> bytes:
        return encode_json_packet(MessageType.HEARTBEAT, self._next_sequence(), {
            "role": "lower_controller", "monotonic_s": self.clock() if now is None else now,
        })

    def feed_from_pi(self, data: bytes, now: float | None = None) -> tuple[LowerControllerEvent, ...]:
        received_at = self.clock() if now is None else now
        events = []
        for packet in self.decoder.feed(data):
            try:
                payload = packet.json()
                if not isinstance(payload, dict):
                    raise ValueError("JSON payload must be an object")
                if packet.message_type == MessageType.VISION_OUTPUT:
                    self.latest_vision = payload
                    self.latest_vision_received_at = received_at
                    self.latest_sequence = packet.sequence
                    events.append(LowerControllerEvent("vision", packet.sequence, payload))
                elif packet.message_type == MessageType.ACK:
                    events.append(LowerControllerEvent("ack", packet.sequence, payload))
                elif packet.message_type == MessageType.HEARTBEAT:
                    events.append(LowerControllerEvent("heartbeat", packet.sequence, payload))
                else:
                    events.append(LowerControllerEvent("unexpected_message", packet.sequence, payload))
            except (ValueError, UnicodeDecodeError) as exc:
                events.append(LowerControllerEvent("invalid_packet", packet.sequence,
                                                   {"error": str(exc)}))
        return tuple(events)

    def vision_usable(self, now: float | None = None) -> bool:
        current = self.clock() if now is None else now
        if self.latest_vision is None or self.latest_vision_received_at is None:
            return False
        fresh = current-self.latest_vision_received_at <= self.vision_timeout_s
        return bool(fresh and self.latest_vision.get("valid", False) and
                    self.latest_vision.get("stream", {}).get("healthy", False))

    def _next_sequence(self) -> int:
        value = self._tx_sequence
        self._tx_sequence = (self._tx_sequence + 1) & 0xFFFFFFFF
        return value
