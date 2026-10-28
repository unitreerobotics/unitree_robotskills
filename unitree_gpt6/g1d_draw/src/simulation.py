"""Deterministic kinematic transport for offline demonstrations, never DDS."""

import numpy as np

from .config import waist_vector


class SimulatedKinematics:
    """Synthetic test model only; not G1 geometry or a physical painting preview."""

    is_simulation = True

    lower = np.full(14, -3.0)
    upper = np.full(14, 3.0)

    @staticmethod
    def _arm_slice(hand):
        if hand not in {"left", "right"}:
            raise ValueError("hand must be left or right")
        return slice(0, 7) if hand == "left" else slice(7, 14)

    def _checked_q14(self, q14):
        from .config import vector

        result = vector(q14, 14, "synthetic q14").copy()
        if np.any(result < self.lower) or np.any(result > self.upper):
            raise ValueError("synthetic joint limits exceeded")
        return result

    def solve(self, pose, seed14, hand):
        result = self._checked_q14(seed14)
        active = self._arm_slice(hand)
        result[active.start : active.start + 3] = [pose[k] for k in ("x", "y", "z")]
        result = self._checked_q14(result)
        self.check_pose(result, pose, hand)
        return result

    def fk(self, q14, hand):
        result = np.eye(4)
        start = self._arm_slice(hand).start
        result[:3, 3] = np.asarray(q14)[start : start + 3]
        return result

    def gravity(self, q14):
        return np.zeros(14)

    def check_pose(self, q14, pose, hand):
        actual = self.fk(q14, hand)[:3, 3]
        if not np.allclose(actual, [pose[k] for k in ("x", "y", "z")], atol=1e-9) or any(
            abs(pose[k]) > 1e-9 for k in ("roll", "pitch", "yaw")
        ):
            raise ValueError("synthetic model target mismatch")


class SimulatedBackend:
    is_simulation = True

    def __init__(self, job):
        self.q = np.array(job["entry_q14"][0], dtype=float)
        self.grips = np.array(job["grip_q2"], dtype=float)
        self.waist = waist_vector(job["waist_q3"]).copy()
        self.acquired = self.stopped = self.connected = False
        self.commands = []

    def connect(self):
        self.connected = True

    def read(self):
        return {
            "q14": self.q.copy(),
            "dq14": np.zeros(14),
            "grip_q2": self.grips.copy(),
            "waist_q3": self.waist.copy(),
            "waist_dq3": np.zeros(3),
        }

    def acquire(self, grip_q2, waist_q3, tau14):
        if self.stopped:
            raise InterruptedError("simulator stopped")
        self.acquired = True

    def command(self, q14, tau14):
        if not self.acquired or self.stopped:
            raise InterruptedError("simulator not acquired or stopped")
        self.q = np.array(q14, dtype=float)
        self.commands.append(self.q.copy())

    def hold(self):
        self.stopped = True
        return self.health()

    def health(self):
        return {
            "publishing": self.acquired,
            "telemetry_fresh": self.connected,
            "fault": None,
            "stopped": self.stopped,
            "published_target": self.acquired,
            "holding": self.acquired,
            "last_sent_q14": self.q.tolist(),
        }

    def close(self):
        self.acquired = self.connected = False


def demo_job():
    """Executable offline fixture; generated values must never reach hardware."""
    from .job import FORMAT

    def batch(identifier, phase, xyz, contact):
        path, action = [], []
        for x, y, z in xyz:
            q = np.zeros(14)
            q[7:10] = [x, y, z]
            path.append(q.tolist())
            action.append(dict(x=x, y=y, z=z, roll=0.0, pitch=0.0, yaw=0.0, gripper=0.0))
        result = dict(
            id=identifier,
            phase=phase,
            color="water" if phase == "WASH" else "blue",
            action=action,
            joint_path_q14=path,
            end_clear=True,
            strokes=[],
        )
        if phase == "PAINT":
            result["strokes"] = [dict(id="demo-stroke", start=1, end=2)]
        else:
            result["contact_index"] = contact
        return result

    load = batch("load", "LOAD", [(0.0, 0.0, 0.1), (0.0, 0.0, 0.02), (0.0, 0.0, 0.1)], 1)
    paint = batch(
        "paint",
        "PAINT",
        [(0.0, 0.0, 0.1), (0.0, 0.0, 0.0), (0.05, 0.0, 0.0), (0.05, 0.0, 0.1)],
        None,
    )
    wash = batch("wash", "WASH", [(0.0, 0.1, 0.1), (0.0, 0.1, 0.02), (0.0, 0.1, 0.1)], 1)
    return dict(
        format=FORMAT,
        job_id="offline-demo-1",
        purpose="simulation",
        hand="right",
        grip_q2=[0.0, 0.0],
        waist_q3=[0.0, 0.0, 0.0],
        entry_q14=[load["joint_path_q14"][0]],
        park_q14=[wash["joint_path_q14"][-1], load["joint_path_q14"][0]],
        clearance_review="Synthetic offline fixture only; no physical clearance claim.",
        batches=[load, paint, wash],
    )
