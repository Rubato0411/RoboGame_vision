from __future__ import annotations

import argparse
from contextlib import ExitStack
import json
from pathlib import Path

from src.communication_runtime import PiCommunicationRuntime, SerialByteStream
from src.competition_controller import CompetitionController
from src.competition_runtime import CompetitionRuntime
from src.hardware_config import HardwareMeasurementConfig
from src.image_source import CameraConfig, ImageSource
from src.hand_eye_calibration import load_gripper_from_camera
from src.raspberry_pi_endpoint import RaspberryPiVisionEndpoint
from src.vision_pipeline import VisionPipeline, VisionPipelinePaths


ROOT = Path(__file__).resolve().parent


def parse_source(value: str):
    return int(value) if value.isdigit() else value


def resolve_project_path(value: str) -> str:
    path = Path(value)
    return str(path if path.is_absolute() else ROOT / path)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Hardware-gated RoboGame competition runtime")
    parser.add_argument("--front-source")
    parser.add_argument("--gripper-source")
    parser.add_argument("--front-calibration")
    parser.add_argument("--front-coordinates")
    parser.add_argument("--gripper-calibration")
    parser.add_argument("--gripper-hand-eye",
                        help="Eye-in-hand calibration JSON containing T_gripper_camera")
    parser.add_argument("--serial-port",
                        help="Wired lower-controller serial device")
    parser.add_argument("--serial-baud", type=int)
    parser.add_argument("--hardware", default=str(ROOT / "configs" / "hardware_measurements.json"))
    parser.add_argument("--rules", default=str(ROOT / "configs" / "competition_rules.json"))
    parser.add_argument("--strategy", default=str(ROOT / "configs" / "competition_strategy.json"))
    parser.add_argument("--width", type=int)
    parser.add_argument("--height", type=int)
    parser.add_argument("--fps", type=float)
    parser.add_argument("--fourcc")
    parser.add_argument("--backend", choices=["auto", "v4l2", "picamera2"])
    parser.add_argument("--feedback-timeout", type=float)
    parser.add_argument("--gripper-pose-timeout", type=float)
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--quiet", action="store_true")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    hardware = HardwareMeasurementConfig.from_json(args.hardware)
    try:
        hardware.require_ready()
    except RuntimeError as exc:
        print(f"HARDWARE CONFIG BLOCKED STARTUP: {exc}")
        return 2

    front_source = args.front_source or str(hardware.get("cameras.front.device_path"))
    gripper_source = args.gripper_source or str(hardware.get("cameras.gripper.device_path"))
    front_calibration = resolve_project_path(
        args.front_calibration or hardware.get("cameras.front.calibration_file"))
    front_coordinates = resolve_project_path(
        args.front_coordinates or hardware.get("cameras.front.coordinate_geometry_file"))
    gripper_calibration = resolve_project_path(
        args.gripper_calibration or hardware.get("cameras.gripper.calibration_file"))
    gripper_hand_eye = resolve_project_path(
        args.gripper_hand_eye or hardware.get("cameras.gripper.hand_eye_calibration_file"))
    serial_port = args.serial_port or str(hardware.get("lower_controller.device"))
    serial_baud = args.serial_baud or int(hardware.get("lower_controller.baudrate"))
    feedback_timeout = args.feedback_timeout or float(hardware.get("safety.heartbeat_timeout_s"))
    gripper_pose_timeout = args.gripper_pose_timeout or float(
        hardware.get("safety.gripper_pose_timeout_s"))
    if (args.max_frames < 0 or serial_baud <= 0 or feedback_timeout <= 0 or
            gripper_pose_timeout <= 0):
        raise SystemExit("serial baud and feedback timeout must be positive")

    def camera_config(role: str) -> CameraConfig:
        prefix = f"cameras.{role}"
        width = args.width or int(hardware.get(f"{prefix}.actual_width_px"))
        height = args.height or int(hardware.get(f"{prefix}.actual_height_px"))
        fps = args.fps or float(hardware.get(f"{prefix}.actual_fps"))
        if width <= 0 or height <= 0 or fps <= 0:
            raise ValueError(f"invalid measured camera mode for {role}")
        return CameraConfig(
            width=width, height=height, fps=fps,
            backend=args.backend or str(hardware.get(f"{prefix}.backend", "v4l2")),
            fourcc=args.fourcc or str(hardware.get(f"{prefix}.actual_fourcc", "MJPG")),
            buffer_size=int(hardware.get(f"{prefix}.buffer_size", 1)),
            exposure=hardware.get(f"{prefix}.exposure"),
            gain=hardware.get(f"{prefix}.gain"),
            white_balance=hardware.get(f"{prefix}.white_balance"),
            focus=hardware.get(f"{prefix}.focus"),
            exposure_time_us=hardware.get(f"{prefix}.exposure_time_us"),
            analogue_gain=hardware.get(f"{prefix}.analogue_gain"),
            colour_gains=(tuple(hardware.get(f"{prefix}.colour_gains"))
                          if hardware.get(f"{prefix}.colour_gains") is not None else None),
            lens_position=hardware.get(f"{prefix}.lens_position"),
            csi_auto_exposure=bool(hardware.get(f"{prefix}.csi_auto_exposure", True)),
            csi_auto_white_balance=bool(hardware.get(f"{prefix}.csi_auto_white_balance", True)),
            csi_auto_focus=bool(hardware.get(f"{prefix}.csi_auto_focus", True)),
        )

    front_pipeline = VisionPipeline.from_paths(
        VisionPipelinePaths.project_defaults(ROOT),
        front_calibration, front_coordinates, require_field_tags=True)
    gripper_pipeline = VisionPipeline.from_paths(
        VisionPipelinePaths.project_defaults(ROOT),
        gripper_calibration, None, require_field_tags=False,
        allow_dynamic_camera_transform=True)
    transform_gripper_camera = load_gripper_from_camera(gripper_hand_eye)
    controller = CompetitionController.from_json(args.rules, args.strategy)
    endpoint = RaspberryPiVisionEndpoint()
    communication = PiCommunicationRuntime(
        SerialByteStream(serial_port, serial_baud), endpoint)
    runtime = CompetitionRuntime(
        front_pipeline, controller, communication, hardware,
        require_hardware_ready=True, feedback_timeout_s=feedback_timeout,
        gripper_pipeline=gripper_pipeline,
        transform_gripper_camera=transform_gripper_camera,
        gripper_pose_timeout_s=gripper_pose_timeout)
    front_camera = camera_config("front")
    gripper_camera = camera_config("gripper")
    processed = 0
    try:
        with ExitStack() as stack:
            front = stack.enter_context(ImageSource(parse_source(front_source), front_camera))
            gripper = stack.enter_context(ImageSource(parse_source(gripper_source), gripper_camera))
            while True:
                front_packet = front.read()
                gripper_packet = gripper.read()
                if front_packet is None or gripper_packet is None:
                    raise RuntimeError("one or more cameras stopped producing frames")
                cycle = runtime.process_packets(front_packet, gripper_packet)
                if not args.quiet:
                    print(json.dumps({
                        "decision": cycle.decision.to_dict(),
                        "vision": cycle.output.to_dict(),
                    }, ensure_ascii=False, separators=(",", ":")))
                processed += 1
                if args.max_frames and processed >= args.max_frames:
                    break
    except (FileNotFoundError, RuntimeError, ValueError) as exc:
        print(f"COMPETITION RUNTIME ERROR: {exc}")
        return 1
    finally:
        runtime.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
