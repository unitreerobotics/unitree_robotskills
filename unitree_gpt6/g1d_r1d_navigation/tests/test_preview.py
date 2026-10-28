"""Exercise the public preview flow without robot or model access."""

import unittest

from unitree_gpt6.g1d_r1d_navigation.common.config import load_config
from unitree_gpt6.g1d_r1d_navigation.common.contracts import SkillInput
from unitree_gpt6.g1d_r1d_navigation.src.agent import NavigationAgent


class PreviewTests(unittest.TestCase):
    def setUp(self):
        self.config = load_config("g1d_r1d_navigation", "R1-D")
        self.agent = NavigationAgent()

    def test_demo_produces_preview_without_claiming_task_completion(self):
        result = self.agent.run(SkillInput("home", skill_config=self.config))
        self.assertEqual(result.task_status, "previewed")
        self.assertTrue(result.action_plan)
        self.assertEqual(result.execution_feedback["hardware_commands_sent"], 0)
        self.assertFalse(result.execution_feedback["task_verified"])
        self.assertIsNone(result.error_info)

    def test_unsupported_instruction_is_rejected(self):
        result = self.agent.run(SkillInput("unsupported-demo", skill_config=self.config))
        self.assertEqual(result.task_status, "failed")
        self.assertEqual(result.action_plan, [])
        self.assertIsNotNone(result.error_info)

    def test_hardware_mode_is_rejected(self):
        self.config["mode"] = "hardware"
        result = self.agent.run(SkillInput("home", skill_config=self.config))
        self.assertEqual(result.task_status, "failed")
        self.assertEqual(result.action_plan, [])
