from __future__ import annotations

from dataclasses import dataclass, replace

from .communication_runtime import PiCommunicationRuntime
from .competition_controller import (CompetitionController, CompetitionDecision,
                                     RobotFeedback)
from .hardware_config import HardwareMeasurementConfig
from .image_source import FramePacket
from .coordinate_transform import RigidTransform
from .vision_output import VisionMode, VisionOutput
from .vision_pipeline import VisionPipeline


@dataclass(frozen=True)
class CompetitionCycle:
    output: VisionOutput
    decision: CompetitionDecision
    feedback: RobotFeedback


class CompetitionRuntime:
    """Join vision, high-level policy and wired communication for one robot.

    Hardware readiness is mandatory by default. Unit tests and offline
    simulation must explicitly opt out with require_hardware_ready=False.
    """

    def __init__(self, pipeline: VisionPipeline, controller: CompetitionController,
                 communication: PiCommunicationRuntime | None = None,
                 hardware: HardwareMeasurementConfig | None = None,
                 require_hardware_ready: bool = True,
                 feedback_timeout_s: float = 0.5,
                 gripper_pipeline: VisionPipeline | None = None,
                 transform_gripper_camera: RigidTransform | None = None,
                 gripper_pose_timeout_s: float = 0.15) -> None:
        if feedback_timeout_s <= 0:
            raise ValueError("feedback_timeout_s must be positive")
        if gripper_pose_timeout_s <= 0:
            raise ValueError("gripper_pose_timeout_s must be positive")
        if require_hardware_ready:
            if hardware is None:
                raise RuntimeError("hardware measurement configuration is required")
            hardware.require_ready()
        self.pipeline = pipeline
        self.gripper_pipeline = gripper_pipeline or pipeline
        self.controller = controller
        self.communication = communication
        self.feedback_timeout_s = float(feedback_timeout_s)
        self.gripper_pose_timeout_s = float(gripper_pose_timeout_s)
        self.transform_gripper_camera = transform_gripper_camera
        self.feedback = RobotFeedback()
        self.last_feedback_s: float | None = None
        self.last_gripper_pose_s: float | None = None
        self.last_gripper_pose_sequence: int | None = None
        self.last_output: VisionOutput | None = None
        self._start_latched = False

    def update_feedback(self, feedback: RobotFeedback, now_s: float) -> None:
        pose = feedback.gripper_pose
        if (pose.valid and pose.sample_sequence is not None and
                pose.sample_sequence != self.last_gripper_pose_sequence):
            self.last_gripper_pose_sequence = pose.sample_sequence
            self.last_gripper_pose_s = float(now_s)
        elif pose.valid and pose.sample_sequence == self.last_gripper_pose_sequence:
            # A retransmission may update other feedback flags, but a repeated
            # pose sequence must never replace or refresh the accepted sample.
            feedback = replace(feedback, gripper_pose=self.feedback.gripper_pose)
        self.feedback = feedback
        self.last_feedback_s = float(now_s)
        self._start_latched = self._start_latched or feedback.start_signal

    def current_transform_robot_camera(self, now_s: float) -> RigidTransform | None:
        pose = self.feedback.gripper_pose
        if (self.transform_gripper_camera is None or not pose.valid or
                pose.translation_m is None or pose.rpy_deg is None or
                self.last_gripper_pose_s is None or
                now_s - self.last_gripper_pose_s > self.gripper_pose_timeout_s):
            return None
        transform_robot_gripper = RigidTransform.from_xyz_rpy(
            pose.translation_m, pose.rpy_deg)
        return transform_robot_gripper.compose(self.transform_gripper_camera)

    def process(self, packet: FramePacket) -> CompetitionCycle:
        return self.process_packets(packet, packet)

    def process_packets(self, front_packet: FramePacket,
                        gripper_packet: FramePacket) -> CompetitionCycle:
        now = max(float(front_packet.timestamp), float(gripper_packet.timestamp))
        if self.communication is not None:
            poll = self.communication.poll(now)
            for event in poll.events:
                if event.kind == "robot_feedback":
                    self.update_feedback(RobotFeedback.from_mapping(event.payload), now)
                elif event.kind == "start_signal":
                    self._start_latched = self._start_latched or bool(
                        event.payload.get("active", True))

        feedback = replace(self.feedback, start_signal=self._start_latched)
        if self.communication is not None and (
                self.last_feedback_s is None or
                now - self.last_feedback_s > self.feedback_timeout_s):
            feedback = replace(feedback, lower_controller_healthy=False)

        requested = self.controller.step(self.last_output, feedback, now)
        use_gripper = requested.vision_mode in (VisionMode.FIND_BLOCKS, VisionMode.GRAB_ASSIST)
        selected_pipeline = self.gripper_pipeline if use_gripper else self.pipeline
        selected_packet = gripper_packet if use_gripper else front_packet
        dynamic_camera_transform = (
            self.current_transform_robot_camera(now) if use_gripper else None)
        output = selected_pipeline.process(
            selected_packet, requested.vision_mode,
            desired_block_color=requested.desired_block_color,
            occupied_slot_ids=tuple(sorted(self.controller.occupied_slot_ids)),
            transform_robot_camera=dynamic_camera_transform,
            requested_placement_slot_id=requested.placement_slot_id,
        )
        decision = self.controller.step(output, feedback, now)
        self.last_output = output
        if self.communication is not None:
            self.communication.endpoint.mode = decision.vision_mode
            self.communication.publish_vision(output)
            self.communication.publish_decision(decision)
        return CompetitionCycle(output, decision, feedback)

    def close(self) -> None:
        if self.communication is not None:
            self.communication.close()
