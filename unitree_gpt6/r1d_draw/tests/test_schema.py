"""Shared validators reject the inputs that silently pass ``float()``."""

import math
import unittest

import numpy as np

from unitree_gpt6.r1d_draw.src import schema


class NumberTests(unittest.TestCase):
    def test_accepts_int_and_float(self):
        self.assertEqual(schema.number(2, "n"), 2.0)
        self.assertIsInstance(schema.number(2, "n"), float)
        self.assertEqual(schema.number(-0.5, "n"), -0.5)

    def test_rejects_non_numbers(self):
        for bad in (True, "1.0", None, [1.0], math.nan, math.inf):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                schema.number(bad, "n")

    def test_bounds(self):
        with self.assertRaises(ValueError):
            schema.number(0, "n", positive=True)
        with self.assertRaises(ValueError):
            schema.number(1.9, "n", minimum=2)
        with self.assertRaises(ValueError):
            schema.number(2.1, "n", maximum=2)
        self.assertEqual(schema.number(2, "n", minimum=2, maximum=2), 2.0)


class ArrayTests(unittest.TestCase):
    def test_vector_returns_independent_copy(self):
        source = np.array([1.0, 2.0, 3.0])
        result = schema.vector(source, 3, "v")
        result[0] = 9.0
        self.assertEqual(source[0], 1.0)

    def test_vector_rejects_wrong_length_and_content(self):
        for bad in (
            [1, 2],
            [1, 2, 3, 4],
            [1, 2, None],
            [1, 2, "3"],
            [1, 2, True],
            [1, 2, math.nan],
            [[1, 2, 3]],
            5,
            None,
        ):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                schema.vector(bad, 3, "v")

    def test_array_shape(self):
        self.assertEqual(
            schema.array([[1, 2], [3, 4]], (2, 2), "a").tolist(), [[1.0, 2.0], [3.0, 4.0]]
        )
        for bad in ([[1, 2], [3]], [1, 2, 3, 4], [[1, 2], [3, 4], [5, 6]]):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                schema.array(bad, (2, 2), "a")


class PoseAndTextTests(unittest.TestCase):
    def test_pose_order_follows_pose_keys(self):
        value = {"yaw": 6, "pitch": 5, "roll": 4, "z": 3, "y": 2, "x": 1}
        self.assertEqual(schema.pose(value).tolist(), [1.0, 2.0, 3.0, 4.0, 5.0, 6.0])

    def test_pose_rejects_missing_key_and_non_object(self):
        with self.assertRaises(ValueError):
            schema.pose({"x": 0, "y": 0, "z": 0, "roll": 0, "pitch": 0})
        with self.assertRaises(ValueError):
            schema.pose([0, 0, 0, 0, 0, 0])

    def test_text(self):
        self.assertEqual(schema.text(" a ", "t"), " a ")
        for bad in ("", "   ", None, 3):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                schema.text(bad, "t")


class LayoutTests(unittest.TestCase):
    def test_arm_slots(self):
        self.assertEqual(schema.arm_slots("left"), tuple(range(15, 22)))
        self.assertEqual(schema.arm_slots("right"), tuple(range(22, 29)))
        self.assertEqual(len(schema.ARM_JOINTS), 7)
        with self.assertRaises(ValueError):
            schema.arm_slots("both")

    def test_slots_do_not_overlap_and_fit_the_state(self):
        used = [
            schema.LIFT_SLOT,
            schema.WAIST_YAW_SLOT,
            *schema.LEFT_ARM_SLOTS,
            *schema.RIGHT_ARM_SLOTS,
            *schema.HEAD_SLOTS,
            *schema.GRIP_SLOTS,
        ]
        self.assertEqual(len(used), len(set(used)))
        self.assertTrue(all(0 <= slot < schema.MOTOR_COUNT for slot in used))


class DigestTests(unittest.TestCase):
    def test_key_order_does_not_change_the_digest(self):
        self.assertEqual(
            schema.digest({"a": 1, "b": [1.5, "x"]}), schema.digest({"b": [1.5, "x"], "a": 1})
        )
        self.assertNotEqual(schema.digest({"a": 1}), schema.digest({"a": 2}))

    def test_rejects_nonfinite_and_non_json(self):
        for bad in ({"a": math.nan}, {"a": math.inf}, {"a": {1, 2}}, {"a": np.zeros(2)}):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                schema.digest(bad)


if __name__ == "__main__":
    unittest.main()
