"""Ownership checks before transferring the real R1-D Runtime session."""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from r1d_chess.robot import handover


class HandoverTests(unittest.TestCase):
    def _run(self, active, command='/opt/site/r1d_hold_initial.py'):
        def response(args, **_kwargs):
            if args[2] == 'is-active':
                return SimpleNamespace(stdout='active\n' if args[3] in active else 'inactive\n')
            return SimpleNamespace(stdout=command)

        with patch.object(handover.subprocess, 'run', side_effect=response), \
                patch.object(handover, 'HOLD_PROGRAM', '/opt/site/r1d_hold_initial.py'):
            return handover.active_hold_service()

    def test_current_head_reference_is_identified(self):
        self.assertEqual(self._run({'r1d-head-cal-reference.service'}),
                         'r1d-head-cal-reference.service')

    def test_unknown_owner_is_not_claimed(self):
        self.assertIsNone(self._run(set()))

    def test_unexpected_program_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError, 'not the expected'):
            self._run({'r1d-head-cal-reference.service'}, '/tmp/other.py')

    def test_multiple_hold_services_are_rejected(self):
        with self.assertRaisesRegex(RuntimeError, 'multiple'):
            self._run({'r1d-head-cal-reference.service', 'r1d-hold-initial.service'})


if __name__ == '__main__':
    unittest.main()
