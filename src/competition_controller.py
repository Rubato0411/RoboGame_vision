from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import json
from pathlib import Path
import math

from .competition_rules import CompetitionRules
from .vision_output import VisionMode, VisionOutput


class CompetitionPhase(str, Enum):
    WAITING_START = "WAITING_START"
    NAVIGATE_TO_MATERIAL = "NAVIGATE_TO_MATERIAL"
    SEARCH_BLOCK = "SEARCH_BLOCK"
    ALIGN_GRAB = "ALIGN_GRAB"
    VERIFY_GRASP = "VERIFY_GRASP"
    STOW_CARGO = "STOW_CARGO"
    NAVIGATE_TO_BUILD = "NAVIGATE_TO_BUILD"
    RETRIEVE_CARGO = "RETRIEVE_CARGO"
    LOCALIZE_BUILD = "LOCALIZE_BUILD"
    ALIGN_PLACE = "ALIGN_PLACE"
    VERIFY_RELEASE = "VERIFY_RELEASE"
    VERIFY_STABILITY = "VERIFY_STABILITY"
    RECOVERY = "RECOVERY"
    FINISHED = "FINISHED"
    SAFE_STOP = "SAFE_STOP"


@dataclass(frozen=True)
class CompetitionStrategyConfig:
    """Conservative strategy that remains useful before mechanical details exist."""

    trip_capacity: int = 3
    purple_material_tag_id: int = 3
    orange_material_tag_id: int = 4
    purple_absence_confirm_s: float = 3.0
    roof_when_remaining_s: float = 90.0
    min_orange_layers_before_roof: int = 1
    max_orange_layers_before_forced_roof: int = 5
    max_buildings: int = 3
    cargo_slot_ids: tuple[str, ...] = ("cargo_right", "cargo_left", "cargo_center")
    target_loss_timeout_s: float = 2.0
    phase_timeout_s: float = 30.0

    @classmethod
    def from_json(cls, path: str | Path) -> "CompetitionStrategyConfig":
        raw = json.loads(Path(path).read_text(encoding="utf-8"))
        values = dict(raw.get("strategy", raw))
        if "cargo_slot_ids" in values:
            values["cargo_slot_ids"] = tuple(values["cargo_slot_ids"])
        return cls(**values)

    def validate(self, rules: CompetitionRules) -> None:
        if not 1 <= self.trip_capacity <= rules.max_carried_blocks:
            raise ValueError("trip_capacity exceeds the rule carrying limit")
        if len(self.cargo_slot_ids) < self.trip_capacity:
            raise ValueError("cargo_slot_ids must cover every carried block")
        if len(set(self.cargo_slot_ids)) != len(self.cargo_slot_ids):
            raise ValueError("cargo_slot_ids must be unique")
        if any(not slot_id for slot_id in self.cargo_slot_ids):
            raise ValueError("cargo slot IDs cannot be empty")
        if self.target_loss_timeout_s <= 0 or self.phase_timeout_s <= 0:
            raise ValueError("strategy timeouts must be positive")
        if self.purple_absence_confirm_s <= 0 or self.roof_when_remaining_s <= 0:
            raise ValueError("purple absence and roof timing must be positive")
        if self.purple_material_tag_id == self.orange_material_tag_id:
            raise ValueError("purple and orange material tags must differ")
        if not 1 <= self.max_buildings <= rules.max_scoring_buildings:
            raise ValueError("max_buildings must fit the scoring building limit")
        if not 1 <= self.min_orange_layers_before_roof <= self.max_orange_layers_before_forced_roof:
            raise ValueError("invalid orange layer limits")


@dataclass(frozen=True)
class GripperPoseFeedback:
    """Latest STM32 sample of T_base_gripper; lengths are metres, RPY degrees."""

    valid: bool = False
    sample_sequence: int | None = None
    translation_m: tuple[float, float, float] | None = None
    rpy_deg: tuple[float, float, float] | None = None

    @classmethod
    def from_mapping(cls, value) -> "GripperPoseFeedback":
        if value is None:
            return cls()
        if not isinstance(value, dict):
            raise ValueError("gripper_pose must be an object or null")
        valid = bool(value.get("valid", False))
        if not valid:
            return cls(False)
        sequence = value.get("sample_sequence")
        translation = value.get("translation_m")
        rpy = value.get("rpy_deg")
        if (not isinstance(sequence, int) or isinstance(sequence, bool) or
                not 0 <= sequence <= 0xFFFFFFFF):
            raise ValueError("valid gripper_pose requires uint32 sample_sequence")
        for name, vector in (("translation_m", translation), ("rpy_deg", rpy)):
            if not isinstance(vector, (list, tuple)) or len(vector) != 3:
                raise ValueError(f"valid gripper_pose requires three-value {name}")
            if not all(isinstance(item, (int, float)) and not isinstance(item, bool) and
                       math.isfinite(float(item)) for item in vector):
                raise ValueError(f"gripper_pose {name} must contain finite numbers")
        return cls(True, sequence, tuple(float(v) for v in translation),
                   tuple(float(v) for v in rpy))


@dataclass(frozen=True)
class RobotFeedback:
    """Hardware feedback consumed by the high-level controller.

    All values default to a safe, unconfirmed state so missing hardware data can
    never be interpreted as a successful robot action.
    """

    start_signal: bool = False
    e_stop_active: bool = False
    lower_controller_healthy: bool = True
    fault_detected: bool = False
    at_material_zone: bool = False
    at_material_tag_id: int | None = None
    at_build_zone: bool = False
    grasp_confirmed: bool = False
    cargo_stowed_confirmed: bool = False
    cargo_stowed_slot_id: str | None = None
    cargo_retrieved_confirmed: bool = False
    cargo_retrieved_slot_id: str | None = None
    place_pose_reached: bool = False
    release_confirmed: bool = False
    target_in_slot: bool = False
    structure_stable: bool = False
    robot_in_start_zone: bool = False
    recovery_acknowledged: bool = False
    gripper_pose: GripperPoseFeedback = GripperPoseFeedback()

    @classmethod
    def from_mapping(cls, value: dict) -> "RobotFeedback":
        if not isinstance(value, dict):
            raise ValueError("RobotFeedback payload must be an object")
        allowed = cls.__dataclass_fields__
        converted = {}
        for key, item in value.items():
            if key not in allowed:
                continue
            if key in {"cargo_stowed_slot_id", "cargo_retrieved_slot_id"}:
                converted[key] = str(item) if item not in (None, "") else None
            elif key == "at_material_tag_id":
                if item is not None and (not isinstance(item, int) or isinstance(item, bool)):
                    raise ValueError("at_material_tag_id must be an integer or null")
                converted[key] = item
            elif key == "gripper_pose":
                converted[key] = GripperPoseFeedback.from_mapping(item)
            else:
                if not isinstance(item, bool):
                    raise ValueError(f"RobotFeedback field {key} must be boolean")
                converted[key] = item
        return cls(**converted)


@dataclass(frozen=True)
class CompetitionDecision:
    phase: CompetitionPhase
    vision_mode: VisionMode
    motion_intent: str
    gripper_intent: str
    desired_block_color: str | None
    cargo_slot_id: str | None
    placement_slot_id: str | None
    safe_stop: bool
    match_elapsed_s: float
    reason: str
    material_tag_id: int | None = None

    def to_dict(self) -> dict:
        return {
            "phase": self.phase.value,
            "vision_mode": self.vision_mode.value,
            "motion_intent": self.motion_intent,
            "gripper_intent": self.gripper_intent,
            "desired_block_color": self.desired_block_color,
            "cargo_slot_id": self.cargo_slot_id,
            "placement_slot_id": self.placement_slot_id,
            "safe_stop": self.safe_stop,
            "match_elapsed_s": self.match_elapsed_s,
            "reason": self.reason,
            "material_tag_id": self.material_tag_id,
        }


class CompetitionController:
    """Deterministic high-level match controller with rule safety guards.

    It emits semantic intents rather than motor values. A lower controller must
    translate those intents into chassis and manipulator motion and must retain
    independent emergency-stop authority.
    """

    def __init__(self, rules: CompetitionRules,
                 strategy: CompetitionStrategyConfig | None = None) -> None:
        rules.validate()
        self.rules = rules
        self.strategy = strategy or CompetitionStrategyConfig()
        self.strategy.validate(rules)
        self.phase = CompetitionPhase.WAITING_START
        self.phase_entered_s = 0.0
        self.match_started_s: float | None = None
        self.carried_colors: list[str] = []
        self.cargo_slots: dict[str, str] = {}
        self.placed_colors: list[str] = []
        self.occupied_slot_ids: set[str] = set()
        self.desired_block_color: str | None = None
        self.active_slot_id: str | None = None
        self.active_cargo_slot_id: str | None = None
        self._stable_since_s: float | None = None
        self.purple_collected_count = 0
        self.purple_exhausted = False
        self.active_building_index = 0
        self.building_orange_layers = [0] * self.strategy.max_buildings
        self.building_roofed = [False] * self.strategy.max_buildings
        self.current_material_tag_id: int | None = None

    @classmethod
    def from_json(cls, rules_path: str | Path, strategy_path: str | Path | None = None):
        rules = CompetitionRules.from_json(rules_path)
        strategy = (CompetitionStrategyConfig.from_json(strategy_path)
                    if strategy_path else CompetitionStrategyConfig())
        return cls(rules, strategy)

    @property
    def match_running(self) -> bool:
        return self.match_started_s is not None and self.phase not in {
            CompetitionPhase.FINISHED, CompetitionPhase.SAFE_STOP,
        }

    def reset_for_new_match(self, now_s: float = 0.0) -> None:
        self.phase = CompetitionPhase.WAITING_START
        self.phase_entered_s = float(now_s)
        self.match_started_s = None
        self.carried_colors.clear()
        self.cargo_slots.clear()
        self.placed_colors.clear()
        self.occupied_slot_ids.clear()
        self.desired_block_color = None
        self.active_slot_id = None
        self.active_cargo_slot_id = None
        self._stable_since_s = None
        self.purple_collected_count = 0
        self.purple_exhausted = False
        self.active_building_index = 0
        self.building_orange_layers = [0] * self.strategy.max_buildings
        self.building_roofed = [False] * self.strategy.max_buildings
        self.current_material_tag_id = None

    def step(self, vision: VisionOutput | None, feedback: RobotFeedback,
             now_s: float) -> CompetitionDecision:
        now = float(now_s)
        elapsed = self._match_elapsed(now)

        if feedback.e_stop_active:
            self._enter(CompetitionPhase.SAFE_STOP, now)
            return self._decision(VisionMode.SAFE_STOP, "STOP", "HOLD", elapsed,
                                  "emergency stop is active")
        if not feedback.lower_controller_healthy:
            if self.phase == CompetitionPhase.WAITING_START and self.match_started_s is None:
                return self._decision(VisionMode.SAFE_STOP, "STOP", "HOLD", elapsed,
                                      "waiting for the first healthy lower-controller feedback")
            self._enter(CompetitionPhase.SAFE_STOP, now)
            return self._decision(VisionMode.SAFE_STOP, "STOP", "HOLD", elapsed,
                                  "lower controller heartbeat is unhealthy")
        if self.match_started_s is not None and elapsed >= self.rules.match_duration_s:
            self._enter(CompetitionPhase.FINISHED, now)
        elif feedback.fault_detected and self.phase not in {
                CompetitionPhase.RECOVERY, CompetitionPhase.SAFE_STOP,
                CompetitionPhase.FINISHED}:
            self._enter(CompetitionPhase.RECOVERY, now)

        if self.phase == CompetitionPhase.WAITING_START:
            if feedback.start_signal:
                self.match_started_s = now
                self.desired_block_color = self._choose_next_color()
                self._enter(CompetitionPhase.NAVIGATE_TO_MATERIAL, now)
                elapsed = 0.0
            else:
                return self._decision(VisionMode.IDLE, "HOLD", "HOLD", elapsed,
                                      "waiting for an on-robot start signal")

        if self.phase == CompetitionPhase.NAVIGATE_TO_MATERIAL:
            if "purple" in self.carried_colors and self._should_place_roof(elapsed):
                self.desired_block_color = None
                self._enter(CompetitionPhase.NAVIGATE_TO_BUILD, now)
                return self._decision(VisionMode.FOLLOW_LINE, "GO_TO_BUILD", "HOLD", elapsed,
                                      "roof deadline reached; returning to build immediately")
            requested_tag = self._material_tag_id()
            at_requested_zone = (feedback.at_material_zone and
                                 feedback.at_material_tag_id in (None, requested_tag))
            if at_requested_zone:
                self.current_material_tag_id = requested_tag
                if self.desired_block_color is None:
                    self.desired_block_color = self._choose_next_color()
                self._enter(CompetitionPhase.SEARCH_BLOCK, now)
            else:
                return self._decision(VisionMode.FOLLOW_LINE, "GO_TO_MATERIAL", "HOLD", elapsed,
                                      "following the marked route to a material zone")

        if self.phase == CompetitionPhase.SEARCH_BLOCK:
            if (self.desired_block_color == "orange" and
                    "purple" in self.carried_colors and self._should_place_roof(elapsed)):
                self.desired_block_color = None
                self._enter(CompetitionPhase.NAVIGATE_TO_BUILD, now)
                return self._decision(VisionMode.FOLLOW_LINE, "GO_TO_BUILD", "HOLD", elapsed,
                                      "roof deadline reached; orange search cancelled")
            if self._selected_target_visible(vision):
                self._enter(CompetitionPhase.ALIGN_GRAB, now)
            elif (self.desired_block_color == "purple" and
                  self._phase_timed_out(now, self.strategy.purple_absence_confirm_s)):
                self.purple_exhausted = True
                self.desired_block_color = "orange"
                self._enter(CompetitionPhase.NAVIGATE_TO_MATERIAL, now)
                return self._decision(VisionMode.FOLLOW_LINE, "GO_TO_MATERIAL", "HOLD", elapsed,
                                      "purple supply confirmed absent; switching permanently to orange")
            elif self._phase_timed_out(now, self.strategy.phase_timeout_s):
                self._enter(CompetitionPhase.RECOVERY, now)
            else:
                return self._decision(VisionMode.GRAB_ASSIST, "SEARCH_BLOCK", "OPEN", elapsed,
                                      "searching for a stable target of the requested color")

        if self.phase == CompetitionPhase.ALIGN_GRAB:
            if not self._selected_target_visible(vision):
                if self._phase_timed_out(now, self.strategy.target_loss_timeout_s):
                    self._enter(CompetitionPhase.SEARCH_BLOCK, now)
                return self._decision(VisionMode.GRAB_ASSIST, "HOLD", "OPEN", elapsed,
                                      "selected block is temporarily unavailable")
            if vision is not None and vision.gripper_alignment.aligned:
                self._enter(CompetitionPhase.VERIFY_GRASP, now)
            else:
                return self._decision(VisionMode.GRAB_ASSIST, "ALIGN_TO_BLOCK", "OPEN", elapsed,
                                      "aligning gripper with selected block")

        if self.phase == CompetitionPhase.VERIFY_GRASP:
            if feedback.grasp_confirmed:
                color = self.desired_block_color or "orange"
                if not self.rules.carrying_allowed(self.carried_colors, color):
                    self._enter(CompetitionPhase.SAFE_STOP, now)
                    return self._decision(VisionMode.SAFE_STOP, "STOP", "HOLD", elapsed,
                                          "reported grasp would violate carrying limits")
                self.carried_colors.append(color)
                if color == "purple":
                    self.purple_collected_count += 1
                self.active_cargo_slot_id = self._first_free_cargo_slot()
                if self.active_cargo_slot_id is None:
                    self._enter(CompetitionPhase.SAFE_STOP, now)
                    return self._decision(VisionMode.SAFE_STOP, "STOP", "HOLD", elapsed,
                                          "no free onboard cargo position is available")
                self._enter(CompetitionPhase.STOW_CARGO, now)
            elif self._phase_timed_out(now, self.strategy.phase_timeout_s):
                self._enter(CompetitionPhase.RECOVERY, now)
            else:
                return self._decision(VisionMode.GRAB_ASSIST, "HOLD", "CLOSE", elapsed,
                                      "waiting for independent grasp confirmation")

        if self.phase == CompetitionPhase.STOW_CARGO:
            if (feedback.cargo_stowed_confirmed and
                    feedback.cargo_stowed_slot_id == self.active_cargo_slot_id):
                assert self.active_cargo_slot_id is not None
                self.cargo_slots[self.active_cargo_slot_id] = self.carried_colors[-1]
                self.active_cargo_slot_id = None
                next_color = self._choose_next_color()
                if (len(self.carried_colors) < self.strategy.trip_capacity and
                        self.rules.carrying_allowed(self.carried_colors, next_color)):
                    self.desired_block_color = next_color
                    next_phase = (CompetitionPhase.SEARCH_BLOCK
                                  if self.current_material_tag_id == self._material_tag_for_color(next_color)
                                  else CompetitionPhase.NAVIGATE_TO_MATERIAL)
                    self._enter(next_phase, now)
                else:
                    self.desired_block_color = None
                    self._enter(CompetitionPhase.NAVIGATE_TO_BUILD, now)
            elif self._phase_timed_out(now, self.strategy.phase_timeout_s):
                self._enter(CompetitionPhase.RECOVERY, now)
            else:
                return self._decision(
                    VisionMode.IDLE, "HOLD", "STOW_TO_CARGO", elapsed,
                    "waiting for confirmation that the grasped block is secured onboard")

        if self.phase == CompetitionPhase.NAVIGATE_TO_BUILD:
            if feedback.at_build_zone:
                self.active_cargo_slot_id = self._next_cargo_for_placement(elapsed)
                if self.active_cargo_slot_id is None:
                    if self.carried_colors:
                        self.desired_block_color = self._choose_next_color()
                        self._enter(CompetitionPhase.NAVIGATE_TO_MATERIAL, now)
                        return self._decision(VisionMode.FOLLOW_LINE, "GO_TO_MATERIAL", "HOLD", elapsed,
                                              "roof retained onboard while more orange layers are collected")
                    self._enter(CompetitionPhase.SAFE_STOP, now)
                    return self._decision(VisionMode.SAFE_STOP, "STOP", "HOLD", elapsed,
                                          "cargo manifest is empty at the building zone")
                self.desired_block_color = self.cargo_slots[self.active_cargo_slot_id]
                self._enter(CompetitionPhase.RETRIEVE_CARGO, now)
            else:
                return self._decision(VisionMode.FOLLOW_LINE, "GO_TO_BUILD", "HOLD", elapsed,
                                      "transporting three secured blocks to the building zone")

        if self.phase == CompetitionPhase.RETRIEVE_CARGO:
            if (feedback.cargo_retrieved_confirmed and
                    feedback.cargo_retrieved_slot_id == self.active_cargo_slot_id):
                color = self.cargo_slots.get(self.active_cargo_slot_id)
                self.active_slot_id = self._next_placement_slot_id(color)
                if self.active_slot_id is None:
                    self._enter(CompetitionPhase.SAFE_STOP, now)
                    return self._decision(VisionMode.SAFE_STOP, "STOP", "HOLD", elapsed,
                                          "no reachable virtual platform level remains")
                self._enter(CompetitionPhase.LOCALIZE_BUILD, now)
            elif self._phase_timed_out(now, self.strategy.phase_timeout_s):
                self._enter(CompetitionPhase.RECOVERY, now)
            else:
                return self._decision(
                    VisionMode.IDLE, "HOLD", "RETRIEVE_FROM_CARGO", elapsed,
                    "waiting for the selected onboard block to be presented for placement")

        if self.phase == CompetitionPhase.LOCALIZE_BUILD:
            if (vision is not None and vision.placement.valid and
                    vision.placement.slot_id == self.active_slot_id):
                self._enter(CompetitionPhase.ALIGN_PLACE, now)
            elif self._phase_timed_out(now, self.strategy.phase_timeout_s):
                self._enter(CompetitionPhase.RECOVERY, now)
            else:
                return self._decision(VisionMode.PLACE_ASSIST, "SEARCH_PLACE", "HOLD", elapsed,
                                      "waiting for a configured visible placement target")

        if self.phase == CompetitionPhase.ALIGN_PLACE:
            if feedback.place_pose_reached:
                self._enter(CompetitionPhase.VERIFY_RELEASE, now)
            else:
                return self._decision(VisionMode.PLACE_ASSIST, "ALIGN_TO_PLACE", "HOLD", elapsed,
                                      "moving to the selected placement pose")

        if self.phase == CompetitionPhase.VERIFY_RELEASE:
            if feedback.release_confirmed and feedback.target_in_slot:
                self._stable_since_s = now if feedback.structure_stable else None
                self._enter(CompetitionPhase.VERIFY_STABILITY, now)
            elif self._phase_timed_out(now, self.strategy.phase_timeout_s):
                self._enter(CompetitionPhase.RECOVERY, now)
            else:
                return self._decision(VisionMode.PLACE_ASSIST, "HOLD", "OPEN", elapsed,
                                      "waiting for release and in-slot confirmation")

        if self.phase == CompetitionPhase.VERIFY_STABILITY:
            if feedback.structure_stable:
                if self._stable_since_s is None:
                    self._stable_since_s = now
                if now - self._stable_since_s >= self.rules.build_stability_s:
                    if self._commit_placement():
                        if self.active_building_index >= self.strategy.max_buildings:
                            self._enter(CompetitionPhase.FINISHED, now)
                        elif self.carried_colors:
                            self.active_cargo_slot_id = self._next_cargo_for_placement(elapsed)
                            if self.active_cargo_slot_id is None:
                                self.desired_block_color = self._choose_next_color()
                                self._enter(CompetitionPhase.NAVIGATE_TO_MATERIAL, now)
                                return self._decision(
                                    VisionMode.FOLLOW_LINE, "GO_TO_MATERIAL", "HOLD", elapsed,
                                    "roof retained until time or height threshold")
                            self.desired_block_color = (
                                self.cargo_slots[self.active_cargo_slot_id]
                                if self.active_cargo_slot_id else None)
                            next_phase = CompetitionPhase.RETRIEVE_CARGO
                        else:
                            next_phase = CompetitionPhase.NAVIGATE_TO_MATERIAL
                        self._enter(next_phase, now)
            else:
                self._stable_since_s = None
            if self.phase == CompetitionPhase.VERIFY_STABILITY:
                return self._decision(VisionMode.PLACE_ASSIST, "HOLD", "OPEN", elapsed,
                                      "requiring continuous structure stability for the rule duration")

        if self.phase == CompetitionPhase.RECOVERY:
            waited = now - self.phase_entered_s
            if (feedback.robot_in_start_zone and feedback.recovery_acknowledged and
                    waited >= self.rules.abnormal_restart_wait_s):
                has_pending_stow = (
                    self.active_cargo_slot_id is not None and
                    self.active_cargo_slot_id not in self.cargo_slots and
                    len(self.carried_colors) > len(self.cargo_slots))
                if has_pending_stow:
                    next_phase = CompetitionPhase.STOW_CARGO
                elif self.cargo_slots:
                    next_phase = CompetitionPhase.NAVIGATE_TO_BUILD
                else:
                    next_phase = CompetitionPhase.NAVIGATE_TO_MATERIAL
                self._enter(next_phase, now)
            else:
                return self._decision(VisionMode.SAFE_STOP, "STOP", "HOLD", elapsed,
                                      "awaiting manual abnormal handling and six-second restart delay")

        if self.phase in (CompetitionPhase.FINISHED, CompetitionPhase.SAFE_STOP):
            reason = "match time elapsed" if self.phase == CompetitionPhase.FINISHED else "safe stop latched"
            return self._decision(VisionMode.SAFE_STOP, "STOP", "HOLD", elapsed, reason)

        return self.step(vision, feedback, now)

    def _enter(self, phase: CompetitionPhase, now_s: float) -> None:
        if phase != self.phase:
            self.phase = phase
            self.phase_entered_s = float(now_s)

    def _phase_timed_out(self, now_s: float, timeout_s: float) -> bool:
        return now_s - self.phase_entered_s >= timeout_s

    def _match_elapsed(self, now_s: float) -> float:
        return 0.0 if self.match_started_s is None else max(0.0, now_s - self.match_started_s)

    @staticmethod
    def _selected_target_visible(vision: VisionOutput | None) -> bool:
        return bool(vision is not None and vision.valid and vision.selected_target.valid)

    def _choose_next_color(self) -> str:
        can_take_purple = (
            not self.purple_exhausted and
            self.purple_collected_count < self.rules.max_available_purple_blocks and
            "purple" not in self.carried_colors and
            self.active_building_index < self.strategy.max_buildings
        )
        if can_take_purple:
            return "purple"
        return "orange"

    def _first_free_cargo_slot(self) -> str | None:
        return next((slot_id for slot_id in self.strategy.cargo_slot_ids
                     if slot_id not in self.cargo_slots), None)

    def _next_loaded_cargo_slot(self) -> str | None:
        return next((slot_id for slot_id in self.strategy.cargo_slot_ids
                     if slot_id in self.cargo_slots), None)

    def _next_cargo_for_placement(self, elapsed_s: float) -> str | None:
        purple = next((slot for slot in self.strategy.cargo_slot_ids
                       if self.cargo_slots.get(slot) == "purple"), None)
        orange = next((slot for slot in self.strategy.cargo_slot_ids
                       if self.cargo_slots.get(slot) == "orange"), None)
        if purple is not None and self._should_place_roof(elapsed_s):
            return purple
        return orange

    def _should_place_roof(self, elapsed_s: float) -> bool:
        if self.active_building_index >= self.strategy.max_buildings:
            return False
        layers = self.building_orange_layers[self.active_building_index]
        if layers < self.strategy.min_orange_layers_before_roof:
            return False
        remaining = max(0.0, self.rules.match_duration_s - elapsed_s)
        return (layers >= self.strategy.max_orange_layers_before_forced_roof or
                remaining <= self.strategy.roof_when_remaining_s)

    def _next_placement_slot_id(self, color: str | None) -> str | None:
        if color not in ("orange", "purple") or self.active_building_index >= self.strategy.max_buildings:
            return None
        level = self.building_orange_layers[self.active_building_index] + 1
        if level > self.strategy.max_orange_layers_before_forced_roof + 1:
            return None
        return f"building_{self.active_building_index + 1}_level_{level}"

    def _commit_placement(self) -> bool:
        if not self.carried_colors or self.active_cargo_slot_id not in self.cargo_slots:
            self._enter(CompetitionPhase.SAFE_STOP, self.phase_entered_s)
            return False
        color = self.cargo_slots.pop(self.active_cargo_slot_id)
        self.carried_colors.remove(color)
        self.placed_colors.append(color)
        if color == "orange":
            self.building_orange_layers[self.active_building_index] += 1
            if (self.building_orange_layers[self.active_building_index] >=
                    self.strategy.max_orange_layers_before_forced_roof and
                    (self.purple_exhausted or
                     self.purple_collected_count >= self.rules.max_available_purple_blocks) and
                    "purple" not in self.carried_colors):
                self.active_building_index += 1
        else:
            self.building_roofed[self.active_building_index] = True
            self.active_building_index += 1
        if self.active_slot_id:
            self.occupied_slot_ids.add(self.active_slot_id)
        self.active_slot_id = None
        self.active_cargo_slot_id = None
        self.desired_block_color = None
        self._stable_since_s = None
        return True

    def _decision(self, vision_mode: VisionMode, motion: str, gripper: str,
                  elapsed: float, reason: str) -> CompetitionDecision:
        return CompetitionDecision(
            self.phase, vision_mode, motion, gripper, self.desired_block_color,
            self.active_cargo_slot_id, self.active_slot_id, self.phase in {
                CompetitionPhase.RECOVERY, CompetitionPhase.FINISHED,
                CompetitionPhase.SAFE_STOP,
            }, float(elapsed), reason,
            self._material_tag_id(),
        )

    def _material_tag_id(self) -> int | None:
        if self.phase not in {CompetitionPhase.NAVIGATE_TO_MATERIAL,
                              CompetitionPhase.SEARCH_BLOCK,
                              CompetitionPhase.ALIGN_GRAB}:
            return None
        return self._material_tag_for_color(self.desired_block_color)

    def _material_tag_for_color(self, color: str | None) -> int | None:
        if color == "purple":
            return self.strategy.purple_material_tag_id
        if color == "orange":
            return self.strategy.orange_material_tag_id
        return None
