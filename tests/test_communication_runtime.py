from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.communication_protocol import (MessageType, PacketStreamDecoder,
                                        encode_json_packet)  # noqa: E402
from src.communication_runtime import PiCommunicationRuntime  # noqa: E402


class FakeStream:
    def __init__(self):
        self.incoming = bytearray()
        self.outgoing = bytearray()
        self.closed = False

    def read_available(self, max_bytes=65536):
        result = bytes(self.incoming[:max_bytes])
        del self.incoming[:max_bytes]
        return result

    def write(self, data):
        self.outgoing.extend(data)
        return len(data)

    def close(self):
        self.closed = True


class CommunicationRuntimeTests(unittest.TestCase):
    def test_feedback_is_decoded_and_acknowledged(self):
        stream = FakeStream()
        stream.incoming.extend(encode_json_packet(
            MessageType.ROBOT_FEEDBACK, 8, {"at_material_zone": True}))
        runtime = PiCommunicationRuntime(stream, heartbeat_interval_s=1.0)
        result = runtime.poll(now_s=2.0)
        self.assertEqual(result.events[0].kind, "robot_feedback")
        packets = PacketStreamDecoder().feed(stream.outgoing)
        self.assertEqual([item.message_type for item in packets],
                         [MessageType.ACK, MessageType.HEARTBEAT])

    def test_heartbeat_interval_is_enforced(self):
        stream = FakeStream()
        runtime = PiCommunicationRuntime(stream, heartbeat_interval_s=1.0)
        runtime.poll(now_s=0.0)
        first_size = len(stream.outgoing)
        runtime.poll(now_s=0.9)
        self.assertEqual(len(stream.outgoing), first_size)
        runtime.poll(now_s=1.0)
        self.assertGreater(len(stream.outgoing), first_size)

    def test_valid_gripper_pose_feedback_is_accepted(self):
        stream = FakeStream()
        stream.incoming.extend(encode_json_packet(
            MessageType.ROBOT_FEEDBACK, 9, {
                "gripper_pose": {
                    "valid": True,
                    "sample_sequence": 12,
                    "translation_m": [0.2, -0.1, 0.4],
                    "rpy_deg": [5.0, -10.0, 20.0],
                }
            }))
        result = PiCommunicationRuntime(stream).poll(now_s=0.0)
        self.assertEqual(result.events[0].kind, "robot_feedback")

    def test_invalid_gripper_pose_feedback_is_nacked(self):
        stream = FakeStream()
        stream.incoming.extend(encode_json_packet(
            MessageType.ROBOT_FEEDBACK, 10, {
                "gripper_pose": {
                    "valid": True,
                    "sample_sequence": 12,
                    "translation_m": [200, 100],
                    "rpy_deg": [0, 0, 0],
                }
            }))
        result = PiCommunicationRuntime(stream).poll(now_s=0.0)
        self.assertEqual(result.events[0].kind, "invalid_command")
        packets = PacketStreamDecoder().feed(stream.outgoing)
        ack = packets[0].json()
        self.assertFalse(ack["accepted"])

