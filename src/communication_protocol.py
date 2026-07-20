from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import json
import struct
from typing import Any, Iterable
import zlib


MAGIC = b"RG"
PROTOCOL_VERSION = 1
HEADER = struct.Struct("<2sBBIH")
CRC = struct.Struct("<I")
MAX_PAYLOAD_BYTES = 65535


class MessageType(IntEnum):
    VISION_OUTPUT = 0x01
    HEARTBEAT = 0x02
    MODE_COMMAND = 0x10
    ROBOT_FEEDBACK = 0x11
    START_SIGNAL = 0x12
    COMPETITION_COMMAND = 0x20
    ACK = 0x7E
    ERROR = 0x7F


@dataclass(frozen=True)
class ProtocolPacket:
    message_type: MessageType
    sequence: int
    payload: bytes

    def json(self) -> Any:
        return json.loads(self.payload.decode("utf-8"))


@dataclass(frozen=True)
class DecoderStats:
    decoded_packets: int
    crc_errors: int
    format_errors: int
    discarded_bytes: int


def json_payload(value: Any) -> bytes:
    return json.dumps(value, ensure_ascii=False, separators=(",", ":"),
                      allow_nan=False).encode("utf-8")


def encode_packet(message_type: MessageType | int, sequence: int,
                  payload: bytes | bytearray | memoryview) -> bytes:
    body = bytes(payload)
    if len(body) > MAX_PAYLOAD_BYTES:
        raise ValueError(f"payload exceeds {MAX_PAYLOAD_BYTES} bytes")
    if not 0 <= sequence <= 0xFFFFFFFF:
        raise ValueError("sequence must fit uint32")
    header = HEADER.pack(MAGIC, PROTOCOL_VERSION, int(message_type), sequence, len(body))
    checksum = zlib.crc32(header + body) & 0xFFFFFFFF
    return header + body + CRC.pack(checksum)


def encode_json_packet(message_type: MessageType | int, sequence: int, value: Any) -> bytes:
    return encode_packet(message_type, sequence, json_payload(value))


class PacketStreamDecoder:
    """Incremental serial/TCP byte decoder with resynchronization and CRC."""

    def __init__(self, max_payload_bytes: int = MAX_PAYLOAD_BYTES) -> None:
        self.max_payload_bytes = max_payload_bytes
        self._buffer = bytearray()
        self._decoded = self._crc_errors = self._format_errors = self._discarded = 0

    @property
    def stats(self) -> DecoderStats:
        return DecoderStats(self._decoded, self._crc_errors,
                            self._format_errors, self._discarded)

    def feed(self, data: bytes | bytearray | memoryview) -> list[ProtocolPacket]:
        self._buffer.extend(data)
        packets = []
        while True:
            marker = self._buffer.find(MAGIC)
            if marker < 0:
                keep = 1 if self._buffer.endswith(MAGIC[:1]) else 0
                discarded = len(self._buffer) - keep
                if discarded:
                    del self._buffer[:discarded]
                    self._discarded += discarded
                break
            if marker:
                del self._buffer[:marker]
                self._discarded += marker
            if len(self._buffer) < HEADER.size:
                break
            magic, version, raw_type, sequence, payload_size = HEADER.unpack_from(self._buffer)
            if magic != MAGIC or version != PROTOCOL_VERSION or payload_size > self.max_payload_bytes:
                del self._buffer[0]
                self._format_errors += 1
                continue
            total = HEADER.size + payload_size + CRC.size
            if len(self._buffer) < total:
                break
            frame = bytes(self._buffer[:total])
            expected_crc = CRC.unpack_from(frame, total-CRC.size)[0]
            actual_crc = zlib.crc32(frame[:total-CRC.size]) & 0xFFFFFFFF
            if expected_crc != actual_crc:
                del self._buffer[0]
                self._crc_errors += 1
                continue
            try:
                message_type = MessageType(raw_type)
            except ValueError:
                del self._buffer[:total]
                self._format_errors += 1
                continue
            payload = frame[HEADER.size:HEADER.size+payload_size]
            packets.append(ProtocolPacket(message_type, sequence, payload))
            del self._buffer[:total]
            self._decoded += 1
        return packets
