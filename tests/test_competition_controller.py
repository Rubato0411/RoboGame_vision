from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.competition_controller import (CompetitionController, CompetitionPhase,
                                        CompetitionStrategyConfig,
                                        RobotFeedback)  # noqa: E402
from src.competition_rules import CompetitionRules  # noqa: E402
from src.vision_output import (GripperAlignmentOutput, PlacementOutput,
                               SelectedTargetOutput, StreamOutput,
                               VisionOutput)  # noqa: E402


def vision(*, target=False, aligned=False, placement=None):
    return VisionOutput(
        1, 1.0, "test", "DEBUG_ALL",
        StreamOutput("HEALTHY", True, 30.0, 0.0, 0, "ok"),
        selected_target=SelectedTargetOutput(target, 7 if target else None, 0.9, "test"),
        gripper_alignment=GripperAlignmentOutput(
            target, 7 if target else None, 0.0, 0.0, 0.0,
            aligned, 0.9, False, "test"),
        placement=placement or PlacementOutput(),
    )


class CompetitionRulesTests(unittest.TestCase):
    def test_rulebook_building_examples(self):
        rules = CompetitionRules()
        self.assertEqual(rules.building_score(["orange"]), 0.0)
        self.assertEqual(rules.building_score(["orange", "orange"]), 1.0)
        self.assertEqual(rules.building_score(["orange", "purple"]), 1.5)
        self.assertEqual(rules.building_score(["orange", "orange", "purple"]), 3.0)
        self.assertEqual(rules.building_score(["orange", "purple", "orange"]), 1.5)

    def test_only_highest_three_buildings_score(self):
        rules = CompetitionRules()
        result = rules.total_building_score([
            ["orange"], ["orange", "orange"],
            ["orange", "orange", "orange"],
            ["orange", "orange", "purple"],
        ])
        self.assertEqual(result, 6.0)

    def test_carrying_limits(self):
        rules = CompetitionRules()
        self.assertFalse(rules.carrying_allowed(["purple"], "purple"))
        self.assertFalse(rules.carrying_allowed(["orange"] * 3, "orange"))
        self.assertTrue(rules.carrying_allowed(["orange", "purple"], "orange"))


class CompetitionControllerTests(unittest.TestCase):
    def test_feedback_preserves_cargo_slot_ids(self):
        feedback = RobotFeedback.from_mapping({
            "cargo_stowed_confirmed": True,
            "cargo_stowed_slot_id": "cargo_center",
            "cargo_retrieved_slot_id": None,
        })
        self.assertTrue(feedback.cargo_stowed_confirmed)
        self.assertEqual(feedback.cargo_stowed_slot_id, "cargo_center")
        self.assertIsNone(feedback.cargo_retrieved_slot_id)

    def test_full_single_block_cycle_requires_three_stable_seconds(self):
        controller = CompetitionController(
            CompetitionRules(), CompetitionStrategyConfig(trip_capacity=1))
        controller.purple_exhausted = True
        decision = controller.step(None, RobotFeedback(start_signal=True), 0.0)
        self.assertEqual(decision.phase, CompetitionPhase.NAVIGATE_TO_MATERIAL)

        decision = controller.step(None, RobotFeedback(at_material_zone=True), 0.1)
        self.assertEqual(decision.phase, CompetitionPhase.SEARCH_BLOCK)
        self.assertEqual(decision.desired_block_color, "orange")

        decision = controller.step(vision(target=True, aligned=True), RobotFeedback(), 0.2)
        self.assertEqual(decision.phase, CompetitionPhase.VERIFY_GRASP)
        self.assertEqual(decision.gripper_intent, "CLOSE")

        decision = controller.step(None, RobotFeedback(grasp_confirmed=True), 0.3)
        self.assertEqual(decision.phase, CompetitionPhase.STOW_CARGO)
        self.assertEqual(decision.cargo_slot_id, "cargo_right")

        decision = controller.step(
            None, RobotFeedback(cargo_stowed_confirmed=True,
                                cargo_stowed_slot_id="cargo_right"), 0.35)
        self.assertEqual(decision.phase, CompetitionPhase.NAVIGATE_TO_BUILD)
        self.assertEqual(controller.carried_colors, ["orange"])
        self.assertEqual(controller.cargo_slots, {"cargo_right": "orange"})

        decision = controller.step(None, RobotFeedback(at_build_zone=True), 0.4)
        self.assertEqual(decision.phase, CompetitionPhase.RETRIEVE_CARGO)
        self.assertEqual(decision.cargo_slot_id, "cargo_right")

        decision = controller.step(
            None, RobotFeedback(cargo_retrieved_confirmed=True,
                                cargo_retrieved_slot_id="cargo_right"), 0.45)
        self.assertEqual(decision.phase, CompetitionPhase.LOCALIZE_BUILD)

        place = PlacementOutput(True, "building_1_level_1", 1,
                                (0.1, 0.2, 0.0), (0.0, 0.0, 0.0), 1, "test")
        decision = controller.step(vision(placement=place), RobotFeedback(), 0.5)
        self.assertEqual(decision.phase, CompetitionPhase.ALIGN_PLACE)

        decision = controller.step(None, RobotFeedback(place_pose_reached=True), 0.6)
        self.assertEqual(decision.phase, CompetitionPhase.VERIFY_RELEASE)

        released = RobotFeedback(
            release_confirmed=True, target_in_slot=True, structure_stable=True)
        decision = controller.step(None, released, 0.7)
        self.assertEqual(decision.phase, CompetitionPhase.VERIFY_STABILITY)
        decision = controller.step(None, RobotFeedback(structure_stable=True), 3.69)
        self.assertEqual(decision.phase, CompetitionPhase.VERIFY_STABILITY)
        decision = controller.step(None, RobotFeedback(structure_stable=True), 3.70)
        self.assertEqual(decision.phase, CompetitionPhase.NAVIGATE_TO_MATERIAL)
        self.assertEqual(controller.carried_colors, [])
        self.assertEqual(controller.placed_colors, ["orange"])
        self.assertIn("building_1_level_1", controller.occupied_slot_ids)

    def test_stability_timer_resets_when_structure_moves(self):
        controller = CompetitionController(CompetitionRules())
        controller.phase = CompetitionPhase.VERIFY_STABILITY
        controller.match_started_s = 0.0
        controller.carried_colors = ["orange"]
        controller.cargo_slots = {"cargo_left": "orange"}
        controller.active_cargo_slot_id = "cargo_left"
        controller.active_slot_id = "slot"
        controller.step(None, RobotFeedback(structure_stable=True), 1.0)
        controller.step(None, RobotFeedback(structure_stable=False), 3.0)
        decision = controller.step(None, RobotFeedback(structure_stable=True), 4.0)
        self.assertEqual(decision.phase, CompetitionPhase.VERIFY_STABILITY)
        decision = controller.step(None, RobotFeedback(structure_stable=True), 7.0)
        self.assertEqual(decision.phase, CompetitionPhase.NAVIGATE_TO_MATERIAL)

    def test_early_roof_is_retained_while_more_orange_is_collected(self):
        controller = CompetitionController(CompetitionRules())
        controller.phase = CompetitionPhase.VERIFY_STABILITY
        controller.match_started_s = 0.0
        controller.carried_colors = ["orange", "purple"]
        controller.cargo_slots = {
            "cargo_left": "orange",
            "cargo_center": "purple",
        }
        controller.active_cargo_slot_id = "cargo_left"
        controller.active_slot_id = "layer_1"
        controller._stable_since_s = 1.0
        decision = controller.step(None, RobotFeedback(structure_stable=True), 4.0)
        self.assertEqual(decision.phase, CompetitionPhase.NAVIGATE_TO_MATERIAL)
        self.assertEqual(controller.carried_colors, ["purple"])
        self.assertEqual(decision.desired_block_color, "orange")
        self.assertEqual(decision.material_tag_id, 4)

    def test_three_blocks_are_stowed_then_retrieved_by_vehicle_position(self):
        controller = CompetitionController(CompetitionRules())
        controller.step(None, RobotFeedback(start_signal=True), 0.0)
        controller.step(None, RobotFeedback(at_material_zone=True), 0.1)

        expected = [
            ("purple", "cargo_right"),
            ("orange", "cargo_left"),
            ("orange", "cargo_center"),
        ]
        now = 0.2
        for index, (color, cargo_slot) in enumerate(expected):
            decision = controller.step(
                vision(target=True, aligned=True), RobotFeedback(), now)
            self.assertEqual(decision.phase, CompetitionPhase.VERIFY_GRASP)
            self.assertEqual(decision.desired_block_color, color)
            now += 0.1
            decision = controller.step(
                None, RobotFeedback(grasp_confirmed=True), now)
            self.assertEqual(decision.phase, CompetitionPhase.STOW_CARGO)
            self.assertEqual(decision.cargo_slot_id, cargo_slot)
            now += 0.1
            decision = controller.step(
                None, RobotFeedback(cargo_stowed_confirmed=True,
                                    cargo_stowed_slot_id=cargo_slot), now)
            if index == 0:
                self.assertEqual(decision.phase, CompetitionPhase.NAVIGATE_TO_MATERIAL)
                self.assertEqual(decision.material_tag_id, 4)
                now += 0.05
                decision = controller.step(
                    None, RobotFeedback(at_material_zone=True, at_material_tag_id=4), now)
                self.assertEqual(decision.phase, CompetitionPhase.SEARCH_BLOCK)
            elif index == 1:
                self.assertEqual(decision.phase, CompetitionPhase.SEARCH_BLOCK)
            now += 0.1

        self.assertEqual(decision.phase, CompetitionPhase.NAVIGATE_TO_BUILD)
        self.assertEqual(
            controller.cargo_slots,
            {"cargo_right": "purple", "cargo_left": "orange",
             "cargo_center": "orange"})

        decision = controller.step(None, RobotFeedback(at_build_zone=True), now)
        self.assertEqual(decision.phase, CompetitionPhase.RETRIEVE_CARGO)
        self.assertEqual(decision.cargo_slot_id, "cargo_left")

    def test_cargo_confirmation_must_match_commanded_slot(self):
        controller = CompetitionController(CompetitionRules())
        controller.phase = CompetitionPhase.STOW_CARGO
        controller.match_started_s = 0.0
        controller.carried_colors = ["orange"]
        controller.active_cargo_slot_id = "cargo_center"
        decision = controller.step(
            None, RobotFeedback(cargo_stowed_confirmed=True,
                                cargo_stowed_slot_id="cargo_left"), 1.0)
        self.assertEqual(decision.phase, CompetitionPhase.STOW_CARGO)
        self.assertEqual(controller.cargo_slots, {})

    def test_purple_is_prioritized_until_one_is_onboard(self):
        controller = CompetitionController(CompetitionRules())
        self.assertEqual(controller._choose_next_color(), "purple")
        controller.carried_colors.append("purple")
        self.assertEqual(controller._choose_next_color(), "orange")

    def test_purple_absence_switches_permanently_to_tag4_orange(self):
        controller = CompetitionController(CompetitionRules())
        controller.step(None, RobotFeedback(start_signal=True), 0.0)
        decision = controller.step(
            None, RobotFeedback(at_material_zone=True, at_material_tag_id=3), 0.1)
        self.assertEqual(decision.desired_block_color, "purple")
        decision = controller.step(None, RobotFeedback(), 3.1)
        self.assertTrue(controller.purple_exhausted)
        self.assertEqual(decision.desired_block_color, "orange")
        self.assertEqual(decision.material_tag_id, 4)
        self.assertEqual(controller._choose_next_color(), "orange")

    def test_roof_is_selected_by_remaining_time_or_height(self):
        controller = CompetitionController(CompetitionRules())
        controller.match_started_s = 0.0
        controller.building_orange_layers[0] = 2
        controller.carried_colors = ["purple", "orange"]
        controller.cargo_slots = {"cargo_right": "purple", "cargo_left": "orange"}
        self.assertEqual(controller._next_cargo_for_placement(100.0), "cargo_left")
        self.assertEqual(controller._next_cargo_for_placement(300.0), "cargo_right")

    def test_roof_deadline_preempts_orange_search(self):
        controller = CompetitionController(CompetitionRules())
        controller.phase = CompetitionPhase.SEARCH_BLOCK
        controller.match_started_s = 0.0
        controller.desired_block_color = "orange"
        controller.carried_colors = ["purple"]
        controller.cargo_slots = {"cargo_right": "purple"}
        controller.building_orange_layers[0] = 2
        decision = controller.step(None, RobotFeedback(), 300.0)
        self.assertEqual(decision.phase, CompetitionPhase.NAVIGATE_TO_BUILD)
        self.assertEqual(decision.motion_intent, "GO_TO_BUILD")

    def test_estop_latches_safe_stop(self):
        controller = CompetitionController(CompetitionRules())
        decision = controller.step(None, RobotFeedback(e_stop_active=True), 1.0)
        self.assertEqual(decision.phase, CompetitionPhase.SAFE_STOP)
        self.assertTrue(decision.safe_stop)

    def test_match_stops_at_six_minutes(self):
        controller = CompetitionController(CompetitionRules())
        controller.step(None, RobotFeedback(start_signal=True), 10.0)
        decision = controller.step(None, RobotFeedback(), 370.0)
        self.assertEqual(decision.phase, CompetitionPhase.FINISHED)
        self.assertTrue(decision.safe_stop)
