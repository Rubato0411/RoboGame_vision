from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import json
from pathlib import Path

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

    orange_blocks_before_roof: int = 2
    trip_capacity: int = 3
    cargo_slot_ids: tuple[str, ...] = ("cargo_left", "cargo_center", "cargo_right")
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
        if self.orange_blocks_before_roof < 1:
            raise ValueError("orange_blocks_before_roof must be at least one")
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

    @classmethod
    def from_mapping(cls, value: dict) -> "RobotFeedback":
        allowed = cls.__dataclass_fields__
        converted = {}
        for key, item in value.items():
            if key not in allowed:
                continue
            if key in {"cargo_stowed_slot_id", "cargo_retrieved_slot_id"}:
                converted[key] = str(item) if item not in (None, "") else None
            else:
                converted[key] = bool(item)
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
                self._enter(CompetitionPhase.NAVIGATE_TO_MATERIAL, now)
                elapsed = 0.0
            else:
                return self._decision(VisionMode.IDLE, "HOLD", "HOLD", elapsed,
                                      "waiting for an on-robot start signal")

        if self.phase == CompetitionPhase.NAVIGATE_TO_MATERIAL:
            if feedback.at_material_zone:
                self.desired_block_color = self._choose_next_color()
                self._enter(CompetitionPhase.SEARCH_BLOCK, now)
            else:
                return self._decision(VisionMode.FOLLOW_LINE, "GO_TO_MATERIAL", "HOLD", elapsed,
                                      "following the marked route to a material zone")

        if self.phase == CompetitionPhase.SEARCH_BLOCK:
            if self._selected_target_visible(vision):
                self._enter(CompetitionPhase.ALIGN_GRAB, now)
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
                    self._enter(CompetitionPhase.SEARCH_BLOCK, now)
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
                self.active_cargo_slot_id = self._next_loaded_cargo_slot()
                if self.active_cargo_slot_id is None:
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
                self._enter(CompetitionPhase.LOCALIZE_BUILD, now)
            elif self._phase_timed_out(now, self.strategy.phase_timeout_s):
                self._enter(CompetitionPhase.RECOVERY, now)
            else:
                return self._decision(
                    VisionMode.IDLE, "HOLD", "RETRIEVE_FROM_CARGO", elapsed,
                    "waiting for the selected onboard block to be presented for placement")

        if self.phase == CompetitionPhase.LOCALIZE_BUILD:
            if vision is not None and vision.placement.valid:
                self.active_slot_id = vision.placement.slot_id
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
                        if self.carried_colors:
                            self.active_cargo_slot_id = self._next_loaded_cargo_slot()
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
        orange_since_roof = 0
        for color in reversed(self.placed_colors + self.carried_colors):
            if color == "purple":
                break
            orange_since_roof += color == "orange"
        if orange_since_roof >= self.strategy.orange_blocks_before_roof:
            return "purple"
        return "orange"

    def _first_free_cargo_slot(self) -> str | None:
        return next((slot_id for slot_id in self.strategy.cargo_slot_ids
                     if slot_id not in self.cargo_slots), None)

    def _next_loaded_cargo_slot(self) -> str | None:
        return next((slot_id for slot_id in self.strategy.cargo_slot_ids
                     if slot_id in self.cargo_slots), None)

    def _commit_placement(self) -> bool:
        if not self.carried_colors or self.active_cargo_slot_id not in self.cargo_slots:
            self._enter(CompetitionPhase.SAFE_STOP, self.phase_entered_s)
            return False
        color = self.cargo_slots.pop(self.active_cargo_slot_id)
        self.carried_colors.remove(color)
        self.placed_colors.append(color)
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
        )
