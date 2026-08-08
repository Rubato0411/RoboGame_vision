from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from time import monotonic


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.communication_runtime import PiCommunicationRuntime, SerialByteStream  # noqa: E402
from src.competition_controller import GripperPoseFeedback, RobotFeedback  # noqa: E402
from src.coordinate_transform import RigidTransform  # noqa: E402
from src.hand_eye_calibration import load_gripper_from_camera  # noqa: E402
from src.hardware_config import HardwareMeasurementConfig  # noqa: E402
from src.image_source import CameraConfig, ImageSource  # noqa: E402
from src.raspberry_pi_endpoint import RaspberryPiVisionEndpoint  # noqa: E402
from src.vision_output import VisionMode  # noqa: E402
from src.vision_pipeline import VisionPipeline, VisionPipelinePaths  # noqa: E402


def resolve_project_path(value: str | None) -> str | None:
    if not value:
        return None
    path = Path(value)
    return str(path if path.is_absolute() else ROOT / path)


def parse_source(value: str):
    return int(value) if value.isdigit() else value


def observation_payload(vision, state: "DynamicPoseState", transform, pose_age, pose_reason):
    data = vision.to_dict()
    return {
        "frame_id": data["frame_id"],
        "pose_valid": transform is not None,
        "pose_sequence": state.last_sequence,
        "pose_age_s": pose_age,
        "pose_reason": pose_reason,
        "stream": data["stream"],
        "alignment": data["gripper_alignment"],
        "blocks": data["blocks"],
        "errors": data["errors"],
    }


class DynamicPoseState:
    """Accept only fresh STM32 poses and compose T_robot_camera on demand."""

    def __init__(self, transform_gripper_camera: RigidTransform, timeout_s: float) -> None:
        if timeout_s <= 0:
            raise ValueError("pose timeout must be positive")
        self.transform_gripper_camera = transform_gripper_camera
        self.timeout_s = float(timeout_s)
        self.pose = GripperPoseFeedback()
        self.last_sequence: int | None = None
        self.last_update_s: float | None = None

    def update(self, pose: GripperPoseFeedback, now_s: float) -> bool:
        if not pose.valid:
            self.pose = GripperPoseFeedback()
            self.last_update_s = None
            return False
        if pose.sample_sequence == self.last_sequence:
            return False
        self.pose = pose
        self.last_sequence = pose.sample_sequence
        self.last_update_s = float(now_s)
        return True

    def current(self, now_s: float) -> tuple[RigidTransform | None, float | None, str]:
        if (not self.pose.valid or self.pose.translation_m is None or self.pose.rpy_deg is None or
                self.last_update_s is None):
            return None, None, "no valid gripper pose"
        age = max(0.0, float(now_s) - self.last_update_s)
        if age > self.timeout_s:
            return None, age, "gripper pose timed out"
        robot_from_gripper = RigidTransform.from_xyz_rpy(
            self.pose.translation_m, self.pose.rpy_deg)
        return robot_from_gripper.compose(self.transform_gripper_camera), age, "fresh pose"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Observe dynamic eye-in-hand 3D results without enabling robot motion")
    parser.add_argument("--hardware", default=str(ROOT / "configs" / "hardware_measurements.json"))
    parser.add_argument("--source", help="Gripper CSI index; defaults to hardware config")
    parser.add_argument("--calibration", help="Gripper intrinsic JSON")
    parser.add_argument("--hand-eye", required=True,
                        help="Measured eye-in-hand JSON containing T_gripper_camera")
    parser.add_argument("--serial-port", required=True,
                        help="STM32 serial device carrying RobotFeedback")
    parser.add_argument("--serial-baud", required=True, type=int)
    parser.add_argument("--pose-timeout", type=float, default=0.15)
    parser.add_argument("--target-color", choices=["orange", "purple"], default="orange")
    parser.add_argument("--width", type=int)
    parser.add_argument("--height", type=int)
    parser.add_argument("--fps", type=float)
    parser.add_argument("--backend", choices=["auto", "v4l2", "picamera2"])
    parser.add_argument("--max-frames", type=int, default=0, help="0 means run until Ctrl+C")
    parser.add_argument("--jsonl", help="Optional copy of compact observations")
    parser.add_argument("--no-publish-vision", action="store_true",
                        help="Do not return VisionOutput packets to STM32")
    parser.add_argument("--quiet", action="store_true")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.serial_baud <= 0 or args.pose_timeout <= 0 or args.max_frames < 0:
        raise SystemExit("serial baud/pose timeout must be positive and max-frames non-negative")
    hardware = HardwareMeasurementConfig.from_json(args.hardware)
    calibration = resolve_project_path(
        args.calibration or hardware.get("cameras.gripper.calibration_file"))
    hand_eye = resolve_project_path(args.hand_eye)
    if not calibration or not Path(calibration).is_file():
        raise SystemExit(f"gripper calibration file not found: {calibration}")
    if not hand_eye or not Path(hand_eye).is_file():
        raise SystemExit(f"hand-eye file not found: {hand_eye}")

    prefix = "cameras.gripper"
    source_value = args.source or str(hardware.get(f"{prefix}.device_path", 0))
    width = args.width or int(hardware.get(f"{prefix}.requested_width_px", 1280))
    height = args.height or int(hardware.get(f"{prefix}.requested_height_px", 720))
    fps = args.fps or float(hardware.get(f"{prefix}.requested_fps", 30.0))
    camera_config = CameraConfig(
        width=width, height=height, fps=fps,
        backend=args.backend or str(hardware.get(f"{prefix}.backend", "picamera2")),
        buffer_size=int(hardware.get(f"{prefix}.buffer_size", 1)),
        exposure_time_us=hardware.get(f"{prefix}.exposure_time_us"),
        analogue_gain=hardware.get(f"{prefix}.analogue_gain"),
        colour_gains=(tuple(hardware.get(f"{prefix}.colour_gains"))
                      if hardware.get(f"{prefix}.colour_gains") is not None else None),
        lens_position=hardware.get(f"{prefix}.lens_position"),
        csi_auto_exposure=bool(hardware.get(f"{prefix}.csi_auto_exposure", False)),
        csi_auto_white_balance=bool(hardware.get(f"{prefix}.csi_auto_white_balance", False)),
        csi_auto_focus=bool(hardware.get(f"{prefix}.csi_auto_focus", False)),
    )
    pipeline = VisionPipeline.from_paths(
        VisionPipelinePaths.project_defaults(ROOT), calibration, None,
        require_field_tags=False, allow_dynamic_camera_transform=True)
    state = DynamicPoseState(load_gripper_from_camera(hand_eye), args.pose_timeout)
    communication = PiCommunicationRuntime(
        SerialByteStream(args.serial_port, args.serial_baud), RaspberryPiVisionEndpoint())
    output_path = Path(args.jsonl) if args.jsonl else None
    if output_path is not None:
        output_path.parent.mkdir(parents=True, exist_ok=True)
    output_handle = output_path.open("w", encoding="utf-8") if output_path else None
    processed = 0
    print("COMMISSIONING ONLY: this tool observes coordinates and sends no arm motion commands.")
    try:
        with ImageSource(parse_source(source_value), camera_config=camera_config) as camera:
            while not args.max_frames or processed < args.max_frames:
                now = monotonic()
                poll = communication.poll(now)
                for event in poll.events:
                    if event.kind == "robot_feedback":
                        state.update(RobotFeedback.from_mapping(event.payload).gripper_pose, now)
                transform, pose_age, pose_reason = state.current(now)
                packet = camera.read()
                if packet is None:
                    raise RuntimeError("gripper camera stopped producing frames")
                vision = pipeline.process(
                    packet, VisionMode.GRAB_ASSIST, args.target_color,
                    transform_robot_camera=transform)
                if not args.no_publish_vision:
                    communication.publish_vision(vision)
                payload = observation_payload(
                    vision, state, transform, pose_age, pose_reason)
                line = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
                if not args.quiet:
                    print(line, flush=True)
                if output_handle is not None:
                    output_handle.write(line + "\n")
                    output_handle.flush()
                processed += 1
    except KeyboardInterrupt:
        return 0
    finally:
        if output_handle is not None:
            output_handle.close()
        communication.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
