from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from time import perf_counter

import numpy as np

from .apriltag_detector import AprilTagDetector
from .black_line_detector import BlackLineDetector
from .block_detector_robust import BlockDetector
from .coordinate_transform import CoordinateTransformer
from .gripper_alignment import GripperAligner
from .image_source import FramePacket
from .stream_health import StreamHealth, StreamHealthMonitor, StreamStatus
from .temporal_tracker import (TemporalObjectTracker, observations_from_apriltags,
                               observations_from_blocks)
from .placement_tag_locator import PlacementTagLocator
from .target_selector import TargetSelector, TargetSelectionConfig
from .vision_output import (BlockOutput, GripperAlignmentOutput, LineOutput, PlacementOutput,
                            ProcessingOutput, RobotPoseOutput, SelectedTargetOutput,
                            StreamOutput, TagOutput, VisionMode, VisionOutput)


@dataclass(frozen=True)
class VisionPipelinePaths:
    apriltag_config: Path
    block_config: Path
    line_config: Path
    temporal_config: Path
    stream_health_config: Path
    gripper_alignment_config: Path
    placement_slots_config: Path

    @classmethod
    def project_defaults(cls, root: str | Path) -> "VisionPipelinePaths":
        configs = Path(root) / "configs"
        return cls(
            configs / "apriltag_detector.json",
            configs / "block_detector_robust.json",
            configs / "black_line_detector.json",
            configs / "temporal_tracker.json",
            configs / "stream_health.json",
            configs / "gripper_alignment.json",
            configs / "placement_slots.json",
        )


class VisionPipeline:
    """Hardware-independent orchestration: FramePacket -> VisionOutput."""

    UNSAFE_STREAM_STATES = {
        StreamStatus.FROZEN, StreamStatus.STALE_FRAME, StreamStatus.INVALID_FRAME,
        StreamStatus.TIMEOUT, StreamStatus.DISCONNECTED,
    }

    def __init__(self, apriltags: AprilTagDetector, blocks: BlockDetector,
                 line: BlackLineDetector, tag_tracker: TemporalObjectTracker,
                 block_tracker: TemporalObjectTracker, stream_monitor: StreamHealthMonitor,
                 coordinates: CoordinateTransformer | None = None,
                 target_selector: TargetSelector | None = None,
                 gripper_aligner: GripperAligner | None = None,
                 placement_locator: PlacementTagLocator | None = None) -> None:
        self.apriltags = apriltags
        self.blocks = blocks
        self.line = line
        self.tag_tracker = tag_tracker
        self.block_tracker = block_tracker
        self.stream_monitor = stream_monitor
        self.coordinates = coordinates
        self.target_selector = target_selector or TargetSelector(TargetSelectionConfig())
        self.gripper_aligner = gripper_aligner
        self.placement_locator = placement_locator

    @classmethod
    def from_paths(cls, paths: VisionPipelinePaths,
                   calibration_path: str | Path | None = None,
                   coordinate_geometry_path: str | Path | None = None) -> "VisionPipeline":
        apriltags = AprilTagDetector.from_json(paths.apriltag_config, calibration_path)
        coordinates = None
        if calibration_path and coordinate_geometry_path:
            coordinates = CoordinateTransformer.from_json(calibration_path, coordinate_geometry_path)
        return cls(
            apriltags, BlockDetector.from_json(paths.block_config),
            BlackLineDetector.from_json(paths.line_config),
            TemporalObjectTracker.from_json(paths.temporal_config),
            TemporalObjectTracker.from_json(paths.temporal_config),
            StreamHealthMonitor.from_json(paths.stream_health_config), coordinates,
            TargetSelector(TargetSelectionConfig()),
            GripperAligner.from_json(paths.gripper_alignment_config),
            PlacementTagLocator.from_json(paths.placement_slots_config),
        )

    def reset(self) -> None:
        self.tag_tracker.reset()
        self.block_tracker.reset()

    def process(self, packet: FramePacket, mode: VisionMode | str = VisionMode.DEBUG_ALL) -> VisionOutput:
        started = perf_counter()
        selected_mode = VisionMode(mode)
        errors: list[str] = []
        timings = {"apriltag_ms": 0.0, "blocks_ms": 0.0,
                   "line_ms": 0.0, "coordinate_ms": 0.0}
        health = self.stream_monitor.observe(packet, now=packet.timestamp)
        stream = self._stream_output(health)
        if health.status in self.UNSAFE_STREAM_STATES:
            self.reset()
            return VisionOutput(
                packet.frame_id, packet.timestamp, packet.source_name, selected_mode.value,
                stream, processing=ProcessingOutput((perf_counter()-started)*1000),
                errors=(f"unsafe stream: {health.status.value}",),
            )

        run_tags = selected_mode in (VisionMode.LOCALIZATION, VisionMode.PLACE_ASSIST,
                                     VisionMode.DEBUG_ALL)
        run_blocks = selected_mode in (VisionMode.FIND_BLOCKS, VisionMode.GRAB_ASSIST,
                                       VisionMode.DEBUG_ALL)
        run_line = selected_mode in (VisionMode.FOLLOW_LINE, VisionMode.DEBUG_ALL)
        tag_outputs: tuple[TagOutput, ...] = ()
        block_outputs: tuple[BlockOutput, ...] = ()
        line_output = LineOutput(False, None, None, None, 0.0, False)
        pose_output = RobotPoseOutput(False)
        selected_output = SelectedTargetOutput()
        alignment_output = GripperAlignmentOutput()
        placement_output = PlacementOutput()

        stable_tags = []
        if run_tags:
            step = perf_counter()
            try:
                raw = self.apriltags.process(packet.image)
                temporal = self.tag_tracker.update(observations_from_apriltags(raw.detections))
                stable_tags = list(temporal.tracks)
                tag_outputs = tuple(self._tag_output(track) for track in stable_tags)
            except (ValueError, RuntimeError) as exc:
                errors.append(f"apriltag: {exc}")
            timings["apriltag_ms"] = (perf_counter()-step)*1000

        if run_blocks:
            step = perf_counter()
            try:
                raw = self.blocks.process(packet.image)
                temporal = self.block_tracker.update(observations_from_blocks(raw.detections))
                block_outputs = tuple(self._block_output(track, errors) for track in temporal.tracks)
                if selected_mode in (VisionMode.GRAB_ASSIST, VisionMode.DEBUG_ALL):
                    selection = self.target_selector.select(block_outputs, packet.image.shape[1])
                    selected_output = SelectedTargetOutput(
                        selection.valid, selection.track_id, selection.score, selection.reason)
                    if selection.valid and self.gripper_aligner is not None:
                        selected_track = next(track for track in temporal.tracks
                                              if track.track_id == selection.track_id)
                        aligned = self.gripper_aligner.align(
                            selected_track.center_px, selected_track.detection.angle_deg,
                            selected_track.detection.confidence, selected_track.track_id,
                            selected_track.predicted)
                        alignment_output = GripperAlignmentOutput(
                            aligned.valid, aligned.target_track_id, aligned.dx_px, aligned.dy_px,
                            aligned.angle_error_deg, aligned.aligned, aligned.confidence,
                            aligned.predicted, aligned.reason)
            except (ValueError, RuntimeError) as exc:
                errors.append(f"blocks: {exc}")
            timings["blocks_ms"] = (perf_counter()-step)*1000

        if run_line:
            step = perf_counter()
            try:
                raw_line = self.line.process(packet.image)
                line_output = LineOutput(
                    raw_line.found, raw_line.lateral_offset_px,
                    raw_line.lateral_offset_normalized, raw_line.heading_error_deg,
                    raw_line.confidence, raw_line.intersection_detected,
                )
            except (ValueError, RuntimeError) as exc:
                errors.append(f"line: {exc}")
            timings["line_ms"] = (perf_counter()-step)*1000

        if run_tags and self.coordinates is not None and stable_tags:
            step = perf_counter()
            try:
                estimate = self.coordinates.estimate_robot_pose(
                    [track.detection for track in stable_tags if not track.predicted])
                confidence = min(1.0, len(estimate.used_tag_ids)/2.0)
                pose_output = RobotPoseOutput(
                    True, estimate.xyz_m, estimate.rpy_deg, confidence,
                    estimate.used_tag_ids, estimate.rejected_tag_ids,
                )
            except (ValueError, KeyError) as exc:
                errors.append(f"coordinates: {exc}")
            timings["coordinate_ms"] = (perf_counter()-step)*1000

        if (selected_mode in (VisionMode.PLACE_ASSIST, VisionMode.DEBUG_ALL) and
                self.coordinates is not None and self.placement_locator is not None):
            try:
                target = self.placement_locator.select_next(
                    [track.detection for track in stable_tags if not track.predicted],
                    self.coordinates.transform_robot_camera)
                placement_output = PlacementOutput(
                    target.valid, target.slot_id, target.reference_tag_id,
                    target.position_robot_m, target.rpy_robot_deg, target.layer, target.reason)
            except (ValueError, KeyError) as exc:
                errors.append(f"placement: {exc}")

        processing = ProcessingOutput((perf_counter()-started)*1000, **timings)
        return VisionOutput(
            packet.frame_id, packet.timestamp, packet.source_name, selected_mode.value,
            stream, pose_output, tag_outputs, block_outputs, line_output,
            processing, tuple(errors), selected_output, alignment_output, placement_output,
        )

    @staticmethod
    def _stream_output(health: StreamHealth) -> StreamOutput:
        return StreamOutput(health.status.value, health.healthy, health.fps,
                            health.frame_age_s, health.reconnect_count, health.reason)

    @staticmethod
    def _tag_output(track) -> TagOutput:
        item = track.detection
        position = None if item.tvec_m is None else tuple(float(v) for v in item.tvec_m)
        reprojection_confidence = (1.0 if item.reprojection_error_px is None else
                                   float(np.exp(-item.reprojection_error_px/2.0)))
        return TagOutput(
            item.tag_id, track.track_id, track.confirmed, track.predicted,
            tuple(float(v) for v in track.center_px),
            tuple(tuple(float(v) for v in point) for point in item.corners_px),
            position, item.distance_m, item.reprojection_error_px,
            reprojection_confidence,
        )

    def _block_output(self, track, errors: list[str]) -> BlockOutput:
        item = track.detection
        position = None
        if self.coordinates is not None and not track.predicted:
            try:
                point = self.coordinates.pixel_to_plane(
                    track.center_px, self.coordinates.transform_robot_camera)
                position = tuple(float(v) for v in point)
            except ValueError as exc:
                errors.append(f"block track {track.track_id} coordinates: {exc}")
        return BlockOutput(
            track.track_id, item.class_name, track.confirmed, track.predicted,
            tuple(float(v) for v in track.center_px), tuple(int(v) for v in item.bbox),
            position, float(item.confidence),
        )
