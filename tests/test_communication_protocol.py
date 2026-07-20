from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.communication_protocol import (MessageType, PacketStreamDecoder,
                                        encode_json_packet)  # noqa: E402
from src.raspberry_pi_endpoint import RaspberryPiVisionEndpoint  # noqa: E402
from src.simulated_lower_controller import SimulatedLowerController  # noqa: E402
from src.vision_output import StreamOutput, VisionMode, VisionOutput  # noqa: E402


class CommunicationProtocolTests(unittest.TestCase):
    def test_fragmented_packet_round_trip(self):
        encoded = encode_json_packet(MessageType.HEARTBEAT, 9, {"ok": True})
        decoder, packets = PacketStreamDecoder(), []
        for byte in encoded:
            packets.extend(decoder.feed(bytes([byte])))
        self.assertEqual(len(packets), 1)
        self.assertEqual(packets[0].sequence, 9)
        self.assertEqual(packets[0].json(), {"ok": True})

    def test_crc_corruption_is_rejected_and_next_packet_recovers(self):
        bad = bytearray(encode_json_packet(MessageType.HEARTBEAT, 1, {"x": 1}))
        bad[-1] ^= 0x80
        good = encode_json_packet(MessageType.HEARTBEAT, 2, {"x": 2})
        decoder = PacketStreamDecoder()
        packets = decoder.feed(b"noise" + bad + good)
        self.assertEqual([item.sequence for item in packets], [2])
        self.assertEqual(decoder.stats.crc_errors, 1)

    def test_lower_controller_changes_pi_mode_and_receives_ack(self):
        pi, lower = RaspberryPiVisionEndpoint(), SimulatedLowerController()
        result = pi.feed_received(lower.command_mode(VisionMode.FOLLOW_LINE))
        self.assertEqual(pi.mode, VisionMode.FOLLOW_LINE)
        events = lower.feed_from_pi(result.responses[0], now=0)
        self.assertEqual(events[0].kind, "ack")
        self.assertTrue(events[0].payload["accepted"])

    def test_vision_timeout_invalidates_old_data(self):
        pi, lower = RaspberryPiVisionEndpoint(), SimulatedLowerController(vision_timeout_s=.3)
        output = VisionOutput(1, 0, "test", "IDLE",
                              StreamOutput("HEALTHY", True, 30, 0, 0, "ok"))
        lower.feed_from_pi(pi.encode_vision(output), now=1.0)
        self.assertTrue(lower.vision_usable(now=1.2))
        self.assertFalse(lower.vision_usable(now=1.31))

    def test_invalid_vision_is_never_usable(self):
        pi, lower = RaspberryPiVisionEndpoint(), SimulatedLowerController()
        output = VisionOutput(1, 0, "test", "IDLE",
                              StreamOutput("DISCONNECTED", False, 0, 2, 0, "lost"),
                              errors=("unsafe stream",))
        lower.feed_from_pi(pi.encode_vision(output), now=0)
        self.assertFalse(lower.vision_usable(now=.1))


if __name__ == "__main__":
    unittest.main()
