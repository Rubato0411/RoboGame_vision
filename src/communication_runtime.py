from __future__ import annotations

from dataclasses import dataclass
from time import monotonic
from typing import Protocol

from .competition_controller import CompetitionDecision
from .raspberry_pi_endpoint import EndpointEvent, RaspberryPiVisionEndpoint
from .vision_output import VisionOutput


class DuplexByteStream(Protocol):
    def read_available(self, max_bytes: int = 65536) -> bytes: ...
    def write(self, data: bytes) -> int: ...
    def close(self) -> None: ...


class SerialByteStream:
    """Non-blocking pyserial adapter loaded only when a real port is requested."""

    def __init__(self, port: str, baudrate: int, write_timeout_s: float = 0.2) -> None:
        if not port:
            raise ValueError("serial port must not be empty")
        if baudrate <= 0:
            raise ValueError("serial baudrate must be positive")
        try:
            import serial
        except ImportError as exc:
            raise RuntimeError("pyserial is required for a real serial transport") from exc
        self._serial = serial.Serial(
            port=port, baudrate=baudrate, timeout=0, write_timeout=write_timeout_s)

    def read_available(self, max_bytes: int = 65536) -> bytes:
        waiting = min(int(self._serial.in_waiting), max_bytes)
        return self._serial.read(waiting) if waiting > 0 else b""

    def write(self, data: bytes) -> int:
        return int(self._serial.write(data))

    def close(self) -> None:
        self._serial.close()


@dataclass(frozen=True)
class CommunicationPoll:
    events: tuple[EndpointEvent, ...]
    bytes_received: int
    bytes_sent: int


class PiCommunicationRuntime:
    """Connect protocol framing to a live duplex stream without owning control policy."""

    def __init__(self, stream: DuplexByteStream,
                 endpoint: RaspberryPiVisionEndpoint | None = None,
                 heartbeat_interval_s: float = 1.0, clock=monotonic) -> None:
        if heartbeat_interval_s <= 0:
            raise ValueError("heartbeat_interval_s must be positive")
        self.stream = stream
        self.endpoint = endpoint or RaspberryPiVisionEndpoint()
        self.heartbeat_interval_s = float(heartbeat_interval_s)
        self.clock = clock
        self._last_heartbeat_s: float | None = None

    def poll(self, now_s: float | None = None) -> CommunicationPoll:
        now = self.clock() if now_s is None else float(now_s)
        incoming = self.stream.read_available()
        events = ()
        sent = 0
        if incoming:
            result = self.endpoint.feed_received(incoming)
            events = result.events
            for response in result.responses:
                sent += self._write_all(response)
        if (self._last_heartbeat_s is None or
                now - self._last_heartbeat_s >= self.heartbeat_interval_s):
            sent += self._write_all(self.endpoint.encode_heartbeat(now))
            self._last_heartbeat_s = now
        return CommunicationPoll(events, len(incoming), sent)

    def publish_vision(self, output: VisionOutput) -> int:
        return self._write_all(self.endpoint.encode_vision(output))

    def publish_decision(self, decision: CompetitionDecision) -> int:
        return self._write_all(self.endpoint.encode_competition_command(decision.to_dict()))

    def close(self) -> None:
        self.stream.close()

    def _write_all(self, payload: bytes) -> int:
        total = 0
        while total < len(payload):
            written = self.stream.write(payload[total:])
            if written <= 0:
                raise RuntimeError("communication stream accepted zero bytes")
            total += written
        return total

