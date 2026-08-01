from __future__ import annotations

from dataclasses import dataclass
from time import monotonic
from typing import Mapping

from .communication_protocol import (MessageType, PacketStreamDecoder,
                                     ProtocolPacket, encode_json_packet)
from .vision_output import VisionMode, VisionOutput
from .competition_controller import RobotFeedback


@dataclass(frozen=True)
class EndpointEvent:
    kind: str
    sequence: int
    payload: dict


@dataclass(frozen=True)
class EndpointReceiveResult:
    events: tuple[EndpointEvent, ...]
    responses: tuple[bytes, ...]


class RaspberryPiVisionEndpoint:
    """Protocol state owned by the Raspberry Pi vision process."""

    def __init__(self, initial_mode: VisionMode = VisionMode.IDLE) -> None:
        self.mode = initial_mode
        self.decoder = PacketStreamDecoder()
        self._tx_sequence = 0
        self.last_command_sequence: int | None = None

    def _next_sequence(self) -> int:
        value = self._tx_sequence
        self._tx_sequence = (self._tx_sequence + 1) & 0xFFFFFFFF
        return value

    def encode_vision(self, output: VisionOutput | Mapping) -> bytes:
        payload = output.to_dict() if isinstance(output, VisionOutput) else dict(output)
        return encode_json_packet(MessageType.VISION_OUTPUT, self._next_sequence(), payload)

    def encode_heartbeat(self, now: float | None = None) -> bytes:
        payload = {"role": "raspberry_pi", "monotonic_s": monotonic() if now is None else now,
                   "mode": self.mode.value}
        return encode_json_packet(MessageType.HEARTBEAT, self._next_sequence(), payload)

    def encode_competition_command(self, command: Mapping) -> bytes:
        return encode_json_packet(
            MessageType.COMPETITION_COMMAND, self._next_sequence(), dict(command))

    def feed_received(self, data: bytes) -> EndpointReceiveResult:
        events, responses = [], []
        for packet in self.decoder.feed(data):
            try:
                payload = packet.json()
                if not isinstance(payload, dict):
                    raise ValueError("JSON payload must be an object")
                if packet.message_type == MessageType.MODE_COMMAND:
                    requested = VisionMode(payload["mode"])
                    self.mode = requested
                    self.last_command_sequence = packet.sequence
                    events.append(EndpointEvent("mode_changed", packet.sequence, payload))
                    responses.append(self._ack(packet, True, f"mode={requested.value}"))
                elif packet.message_type == MessageType.HEARTBEAT:
                    events.append(EndpointEvent("heartbeat", packet.sequence, payload))
                    responses.append(self._ack(packet, True, "heartbeat"))
                elif packet.message_type == MessageType.ROBOT_FEEDBACK:
                    RobotFeedback.from_mapping(payload)
                    events.append(EndpointEvent("robot_feedback", packet.sequence, payload))
                    responses.append(self._ack(packet, True, "robot_feedback"))
                elif packet.message_type == MessageType.START_SIGNAL:
                    events.append(EndpointEvent("start_signal", packet.sequence, payload))
                    responses.append(self._ack(packet, True, "start_signal"))
                else:
                    events.append(EndpointEvent("unexpected_message", packet.sequence, payload))
                    responses.append(self._ack(packet, False, "unsupported message for Pi endpoint"))
            except (KeyError, ValueError, UnicodeDecodeError) as exc:
                responses.append(self._ack(packet, False, str(exc)))
                events.append(EndpointEvent("invalid_command", packet.sequence,
                                            {"error": str(exc)}))
        return EndpointReceiveResult(tuple(events), tuple(responses))

    def _ack(self, packet: ProtocolPacket, accepted: bool, detail: str) -> bytes:
        return encode_json_packet(MessageType.ACK, self._next_sequence(), {
            "ack_sequence": packet.sequence, "accepted": accepted, "detail": detail,
        })
