"""Check rejection paths before the adapter is reached."""

import unittest
from unittest.mock import Mock, patch

from unitree_gpt6.g1d_r1d_navigation.common.config import load_config, validate_config
from unitree_gpt6.g1d_r1d_navigation.common.contracts import SkillInput
from unitree_gpt6.g1d_r1d_navigation.src.agent import NavigationAgent


class ConfigurationTests(unittest.TestCase):
    def test_invalid_common_configuration(self):
        changes = [
            ("robot_model", "G1-D"),
            ("mode", "hardware"),
            ("model_provider", "unimplemented-provider"),
            ("limits", None),
            ("limits", {"max_actions": True, "task_timeout_s": 10}),
            ("limits", {"max_actions": 0, "task_timeout_s": 10}),
            ("limits", {"max_actions": 101, "task_timeout_s": 10}),
            ("limits", {"max_actions": 10, "task_timeout_s": float("nan")}),
            ("limits", {"max_actions": 10, "task_timeout_s": float("inf")}),
            ("limits", {"max_actions": 10, "task_timeout_s": -1}),
        ]
        for key, value in changes:
            with self.subTest(key=key, value=value):
                config = load_config("g1d_r1d_navigation", "R1-D")
                config[key] = value
                with self.assertRaises(ValueError):
                    validate_config(config, "R1-D")

    def test_non_mapping_config_is_rejected(self):
        for config in (None, [], "preview"):
            with self.subTest(config=config), self.assertRaises(ValueError):
                validate_config(config, "R1-D")


class PlanningBoundaryTests(unittest.TestCase):
    def setUp(self):
        self.config = load_config("g1d_r1d_navigation", "R1-D")
        self.agent = NavigationAgent()
        self.agent.adapter = Mock()

    def assert_rejected_before_execution(self, instruction="home"):
        result = self.agent.run(SkillInput(instruction, skill_config=self.config))
        self.assertEqual(result.task_status, "failed")
        self.agent.adapter.execute.assert_not_called()

    def test_hardware_is_rejected_before_adapter(self):
        self.config["mode"] = "hardware"
        self.assert_rejected_before_execution()

    def test_empty_instruction_is_rejected(self):
        self.assert_rejected_before_execution(" ")

    def test_missing_skill_section_is_rejected(self):
        del self.config["navigation"]
        self.assert_rejected_before_execution()

    def test_excessive_action_count_is_rejected(self):
        self.config["limits"]["max_actions"] = 1
        actions = self.agent.plan("home", self.config) * 2
        with patch.object(self.agent, "plan", return_value=actions):
            self.assert_rejected_before_execution()

    def test_planning_deadline_is_checked_before_adapter(self):
        with patch("unitree_gpt6.g1d_r1d_navigation.common.runtime.monotonic", side_effect=[0, 11]):
            self.assert_rejected_before_execution()


class NavigationBoundaryTests(unittest.TestCase):
    def test_non_finite_waypoint_is_rejected(self):
        config = load_config("g1d_r1d_navigation", "R1-D")
        config["waypoints"]["home"]["x_m"] = float("nan")
        result = NavigationAgent().run(SkillInput("home", skill_config=config))
        self.assertEqual(result.task_status, "failed")

    def test_new_named_waypoint_can_be_previewed(self):
        config = load_config("g1d_r1d_navigation", "R1-D")
        config["waypoints"]["desk"] = {"x_m": 1, "y_m": 2, "yaw_rad": 0}
        result = NavigationAgent().run(SkillInput("desk", skill_config=config))
        self.assertEqual(result.task_status, "previewed")
        self.assertEqual(result.action_plan[0].parameters["target"], "desk")
