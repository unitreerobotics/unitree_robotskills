"""URDF FK, continuous damped IK, and bounded multi-start precision IK."""

from __future__ import annotations

import math
import tarfile
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from ik.frames import add_world_frame


URDF_MEMBERS = {
    "right": "R1_Robotic_Arm_R_V1.2-1023/R1_Robotic_Arm_V1.2/R1_Robotic_Arm/urdf/R1_Robotic_Arm.urdf",
    "left": "R1_Robotic_Arm_L_V1.0-1027/R1_Robotic_Arm_L_V1.0-1027/urdf/R1_Robotic_Arm_L_V1.0-1027.urdf",
}


def _numbers(text: str | None, count: int, default: tuple[float, ...]) -> np.ndarray:
    if not text:
        return np.asarray(default, dtype=float)
    values = np.fromstring(text, sep=" ")
    if values.size != count:
        raise ValueError(f"expected {count} URDF numbers, got {text!r}")
    return values


def _rpy_matrix(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _axis_rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    x, y, z = axis / np.linalg.norm(axis)
    c, s, v = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    return np.array([
        [x * x * v + c, x * y * v - z * s, x * z * v + y * s],
        [y * x * v + z * s, y * y * v + c, y * z * v - x * s],
        [z * x * v - y * s, z * y * v + x * s, z * z * v + c],
    ])


def rotation_vector(rotation: np.ndarray) -> np.ndarray:
    """Stable SO(3) logarithm, including rotations close to pi."""
    trace = float(np.trace(rotation))
    qw = math.sqrt(max(0.0, 1.0 + trace)) * 0.5
    skew = np.array([
        rotation[2, 1] - rotation[1, 2],
        rotation[0, 2] - rotation[2, 0],
        rotation[1, 0] - rotation[0, 1],
    ])
    if qw > 1e-7:
        qv = skew / (4.0 * qw)
    else:
        values, vectors = np.linalg.eigh((rotation + np.eye(3)) * 0.5)
        qv = vectors[:, int(np.argmax(values))]
        qv *= 1.0 if np.dot(qv, skew) >= 0 else -1.0
    q = np.concatenate(([qw], qv))
    if q[0] < 0:
        q = -q
    norm = float(np.linalg.norm(q[1:]))
    if norm < 1e-10:
        return 2.0 * q[1:]
    angle = 2.0 * math.atan2(norm, max(0.0, float(q[0])))
    return q[1:] * (angle / norm)


def matrix_to_rpy(rotation: np.ndarray) -> list[float]:
    pitch = math.asin(float(np.clip(-rotation[2, 0], -1.0, 1.0)))
    if abs(math.cos(pitch)) > 1e-7:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = 0.0
        yaw = math.atan2(-rotation[0, 1], rotation[1, 1])
    return [roll, pitch, yaw]


@dataclass(frozen=True)
class Joint:
    name: str
    kind: str
    xyz: np.ndarray
    rotation: np.ndarray
    axis: np.ndarray
    lower: float
    upper: float
    velocity: float


class RobotModel:
    # End-effector load carried by Link7 (gripper, camera, mount): point mass in the
    # end-link frame. Zero keeps the bare-arm URDF gravity model.
    payload_mass = 0.0
    payload_com = (0.0, 0.0, 0.0)

    def __init__(self, archive: Path, hand: str = "right", end_link: str = "Link7"):
        if hand not in URDF_MEMBERS:
            raise ValueError("hand must be 'right' or 'left'")
        self.hand = hand
        with tarfile.open(archive, "r:gz") as bundle:
            member = bundle.extractfile(URDF_MEMBERS[hand])
            if member is None:
                raise ValueError(f"URDF not found for {hand} arm")
            root = ET.fromstring(member.read())
        add_world_frame(root)

        by_child = {}
        child_links = set()
        for element in root.findall("joint"):
            parent = element.find("parent")
            child = element.find("child")
            if parent is None or child is None:
                continue
            by_child[child.attrib["link"]] = element
            child_links.add(child.attrib["link"])
        roots = [element.attrib["name"] for element in root.findall("link")
                 if element.attrib["name"] not in child_links]
        if len(roots) != 1 or end_link not in by_child:
            raise ValueError("URDF must have one root and the requested end link")
        self.root_link = roots[0]
        self.end_link = end_link

        elements = []
        link = end_link
        while link != self.root_link:
            element = by_child.get(link)
            if element is None:
                raise ValueError(f"URDF chain from {end_link} does not reach root")
            elements.append(element)
            link = element.find("parent").attrib["link"]
        elements.reverse()

        self.chain: list[Joint] = []
        self._link_mass: dict[str, float] = {}
        self._link_com: dict[str, np.ndarray] = {}
        self._child_links: list[str] = []
        for element in elements:
            kind = element.attrib["type"]
            origin = element.find("origin")
            xyz = _numbers(origin.attrib.get("xyz") if origin is not None else None, 3, (0, 0, 0))
            rpy = _numbers(origin.attrib.get("rpy") if origin is not None else None, 3, (0, 0, 0))
            axis_tag = element.find("axis")
            axis = _numbers(axis_tag.attrib.get("xyz") if axis_tag is not None else None, 3, (1, 0, 0))
            limit = element.find("limit")
            lower = float(limit.attrib.get("lower", -math.pi)) if limit is not None else -math.pi
            upper = float(limit.attrib.get("upper", math.pi)) if limit is not None else math.pi
            velocity = float(limit.attrib.get("velocity", 2.0)) if limit is not None else 2.0
            self.chain.append(Joint(
                element.attrib["name"], kind, xyz, _rpy_matrix(rpy), axis,
                lower, upper, velocity,
            ))
            child_name = element.find("child").attrib["link"]
            self._child_links.append(child_name)
            link_element = root.find(f"link[@name='{child_name}']")
            if link_element is not None:
                inertial = link_element.find("inertial")
                if inertial is not None:
                    self._link_mass[child_name] = float(inertial.find("mass").attrib["value"])
                    com_origin = inertial.find("origin")
                    self._link_com[child_name] = _numbers(
                        com_origin.attrib.get("xyz") if com_origin is not None else None,
                        3, (0, 0, 0),
                    )
        self.movable = [joint for joint in self.chain if joint.kind in ("revolute", "continuous")]
        if len(self.movable) != 7:
            raise ValueError(f"expected 7 movable joints, found {len(self.movable)}")
        self.names = [joint.name for joint in self.movable]
        self.lower = np.array([joint.lower for joint in self.movable])
        self.upper = np.array([joint.upper for joint in self.movable])
        self.velocity = np.array([joint.velocity for joint in self.movable])

    def forward(self, q: np.ndarray | list[float]) -> tuple[np.ndarray, np.ndarray, list[np.ndarray]]:
        q = np.asarray(q, dtype=float)
        if q.shape != (7,) or not np.all(np.isfinite(q)):
            raise ValueError("q must contain seven finite joint angles")
        transform = np.eye(4)
        points = [transform[:3, 3].copy()]
        index = 0
        for joint in self.chain:
            origin = np.eye(4)
            origin[:3, :3] = joint.rotation
            origin[:3, 3] = joint.xyz
            transform = transform @ origin
            if joint.kind in ("revolute", "continuous"):
                rotation = np.eye(4)
                rotation[:3, :3] = _axis_rotation(joint.axis, float(q[index]))
                transform = transform @ rotation
                index += 1
            points.append(transform[:3, 3].copy())
        return transform[:3, 3].copy(), transform[:3, :3].copy(), points

    def _potential_energy(self, q: np.ndarray) -> float:
        """Potential energy of the URDF links in the world +Z gravity field."""
        transform = np.eye(4)
        index = 0
        energy = 0.0
        for joint, child in zip(self.chain, self._child_links):
            origin = np.eye(4)
            origin[:3, :3] = joint.rotation
            origin[:3, 3] = joint.xyz
            transform = transform @ origin
            if joint.kind in ("revolute", "continuous"):
                rotation = np.eye(4)
                rotation[:3, :3] = _axis_rotation(joint.axis, float(q[index]))
                transform = transform @ rotation
                index += 1
            if child in self._link_mass:
                com = np.r_[self._link_com[child], 1.0]
                energy += self._link_mass[child] * 9.81 * float((transform @ com)[2])
        # The chain ends at the end link, so transform is now world <- Link7.
        energy += self.payload_mass * 9.81 * float((transform @ np.r_[self.payload_com, 1.0])[2])
        return energy

    def gravity_torque(self, q: np.ndarray | list[float], epsilon: float = 1e-5) -> np.ndarray:
        """Generalized torque needed to hold the URDF links against gravity."""
        q = np.asarray(q, dtype=float)
        if q.shape != (7,) or not np.all(np.isfinite(q)):
            raise ValueError("q must contain seven finite joint angles")
        if not math.isfinite(epsilon) or epsilon <= 0:
            raise ValueError("gravity difference step must be positive and finite")
        torque = np.zeros(7)
        for index in range(7):
            plus, minus = q.copy(), q.copy()
            plus[index] += epsilon
            minus[index] -= epsilon
            torque[index] = (self._potential_energy(plus) - self._potential_energy(minus)) / (2.0 * epsilon)
        return torque

    def jacobian(self, q: np.ndarray) -> np.ndarray:
        transform = np.eye(4)
        origins, axes = [], []
        index = 0
        for joint in self.chain:
            origin = np.eye(4)
            origin[:3, :3] = joint.rotation
            origin[:3, 3] = joint.xyz
            transform = transform @ origin
            if joint.kind in ("revolute", "continuous"):
                origins.append(transform[:3, 3].copy())
                axes.append(transform[:3, :3] @ (joint.axis / np.linalg.norm(joint.axis)))
                rotation = np.eye(4)
                rotation[:3, :3] = _axis_rotation(joint.axis, float(q[index]))
                transform = transform @ rotation
                index += 1
        position = transform[:3, 3]
        jacobian = np.zeros((6, 7))
        for i, (origin, axis) in enumerate(zip(origins, axes)):
            jacobian[:3, i] = np.cross(axis, position - origin)
            jacobian[3:, i] = axis
        return jacobian

    def solve(self, position: list[float] | np.ndarray,
              rotation: np.ndarray | None,
              seed: list[float] | np.ndarray, *, realtime: bool = False) -> dict:
        target = np.asarray(position, dtype=float)
        seed = np.asarray(seed, dtype=float)
        if target.shape != (3,) or seed.shape != (7,):
            raise ValueError("IK needs XYZ and seven seed joint angles")
        if not np.all(np.isfinite(target)) or not np.all(np.isfinite(seed)):
            raise ValueError("IK inputs must be finite")
        if rotation is not None:
            rotation = np.asarray(rotation, dtype=float)
            if rotation.shape != (3, 3) or not np.all(np.isfinite(rotation)):
                raise ValueError("target rotation must be a finite 3x3 matrix")
            if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6) or np.linalg.det(rotation) < 0.999:
                raise ValueError("invalid target rotation")
        low, high = self.lower, self.upper
        seed = np.clip(seed, low + 1e-9, high - 1e-9)
        span = high - low
        if realtime:
            q = seed.copy()
            mid = (low + high) * 0.5
            realtime_deadline = time.monotonic() + 0.045
            for _ in range(90):
                p, current_r, _ = self.forward(q)
                ep = target - p
                er = rotation_vector(rotation @ current_r.T) if rotation is not None else np.zeros(3)
                if np.linalg.norm(ep) < 1e-6 and (rotation is None or np.linalg.norm(er) < 1e-5):
                    break
                if time.monotonic() > realtime_deadline:
                    break
                jacobian = self.jacobian(q)
                task_jacobian = jacobian[:3] if rotation is None else np.vstack((jacobian[:3], 0.18 * jacobian[3:]))
                error = ep if rotation is None else np.r_[ep, 0.18 * er]
                singular = np.linalg.svd(task_jacobian, compute_uv=False)
                damping = 0.0005 + max(0.0, 0.025 - float(singular[-1])) * 0.35
                system = task_jacobian @ task_jacobian.T + damping**2 * np.eye(task_jacobian.shape[0])
                inverse = task_jacobian.T @ np.linalg.solve(system, np.eye(system.shape[0]))
                delta = inverse @ error
                center_gradient = (mid - q) / np.maximum(span * span, 1e-8)
                centering = 0.06 * min(1.0, float(np.linalg.norm(error)) / 0.001)
                delta += (np.eye(7) - inverse @ task_jacobian) @ (centering * center_gradient)
                norm = float(np.linalg.norm(delta))
                if norm > 0.22:
                    delta *= 0.22 / norm
                old_score = float(error @ error)
                accepted = False
                for scale in (1.0, 0.5, 0.25, 0.125, 0.0625, 0.03125):
                    candidate = np.clip(q + scale * delta, low, high)
                    candidate_p, candidate_r, _ = self.forward(candidate)
                    candidate_error = target - candidate_p
                    candidate_rotation = rotation_vector(rotation @ candidate_r.T) if rotation is not None else np.zeros(3)
                    weighted = candidate_error if rotation is None else np.r_[candidate_error, 0.18 * candidate_rotation]
                    if float(weighted @ weighted) < old_score - 1e-18:
                        q = candidate
                        accepted = True
                        break
                if not accepted:
                    break
            p, current_r, _ = self.forward(q)
            position_error = float(np.linalg.norm(target - p))
            orientation_error = float(np.linalg.norm(rotation_vector(rotation @ current_r.T))) if rotation is not None else 0.0
            success = position_error <= 0.0001 and (rotation is None or orientation_error <= 0.001)
            return {
                "success": bool(success), "q": q.tolist(),
                "position_error_m": position_error, "orientation_error_rad": orientation_error,
                "seed_distance_rad": float(np.linalg.norm(q - seed)),
                "min_limit_margin_rad": float(np.min(np.minimum(q - self.lower, self.upper - q))),
                "message": "converged" if success else "target unreachable from the current configuration",
            }
        best = None
        deadline = time.monotonic() + 0.8

        class BudgetExpired(Exception):
            pass

        def residual(q):
            nonlocal best
            if time.monotonic() > deadline and best is not None:
                raise BudgetExpired()
            p, r, _ = self.forward(q)
            ep = target - p
            er = rotation_vector(rotation @ r.T) if rotation is not None else np.zeros(3)
            value = np.r_[ep, 0.18 * er] if rotation is not None else ep
            score = float(value @ value)
            if best is None or score < best[0]:
                best = (score, q.copy(), float(np.linalg.norm(ep)), float(np.linalg.norm(er)))
            return value

        def derivative(q):
            j = self.jacobian(q)
            if rotation is None:
                return -j[:3]
            _, r, _ = self.forward(q)
            e = rotation_vector(rotation @ r.T)
            theta = float(np.linalg.norm(e))
            x, y, z = e
            skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
            coefficient = (1 / 12 + theta * theta / 720) if theta < 1e-4 else (1 - theta / (2 * math.tan(theta / 2))) / theta**2
            right_inverse = np.eye(3) + 0.5 * skew + coefficient * (skew @ skew)
            return np.vstack((-j[:3], -0.18 * right_inverse @ j[3:]))

        starts = [seed]
        starts.append((low + high) / 2)
        # Deterministic redundant configurations for recovery from local minima.
        rng = np.random.default_rng(71)
        starts.extend(rng.uniform(low + 1e-6, high - 1e-6, size=(20, 7)))
        for start in starts:
            try:
                result = least_squares(
                    residual, start, jac=derivative, bounds=(low, high),
                    ftol=1e-11, xtol=1e-11, gtol=1e-11,
                    max_nfev=160,
                )
                residual(result.x)
            except BudgetExpired:
                break
            if best[2] < 1e-5 and (rotation is None or best[3] < 1e-4):
                break
        assert best is not None
        success = best[2] <= 0.0001 and (rotation is None or best[3] <= 0.001)
        q = best[1]
        return {
            "success": bool(success), "q": q.tolist(),
            "position_error_m": best[2], "orientation_error_rad": best[3],
            "seed_distance_rad": float(np.linalg.norm(q - seed)),
            "min_limit_margin_rad": float(np.min(np.minimum(q - self.lower, self.upper - q))),
            "message": "converged" if success else "target is unreachable or no solution within the solve budget",
        }


def load_model(sdk_root: Path, hand: str) -> RobotModel:
    return RobotModel(sdk_root / "urdf" / "URDF_R1-7A.tar.gz", hand)
