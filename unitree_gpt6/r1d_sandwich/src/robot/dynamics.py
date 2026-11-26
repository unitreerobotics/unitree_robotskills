# Copied from r1_d sdk/dynamics.py (copied 2026-11-26); only imports and paths changed. Keep in sync by copying; do not restyle.
"""Arm feedforward torques from the URDF: gravity from mass/COM (level base, grippers
at zero travel) and joint friction from <dynamics damping friction load_friction>."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from .model import ARM_NAMES, ARM_TAU_LIMIT

GRAVITY = np.array([0.0, 0.0, -9.81])
SIDES = ("left", "right")


def rotation_rpy(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def rotation_axis(axis, angle):
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    k = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + s * k + (1.0 - c) * (k @ k)


def vector(text):
    return np.array([float(v) for v in text.split()])


class ArmGravity:
    """Holding torque of the seven arm joints; linear in per-body mass and mass*COM.

    Each chain body lumps the links rigidly attached below it (gripper base,
    fingers at zero travel, camera). Gravity is fixed in waist_yaw_link, which
    stays upright because the lift is vertical and the waist only yaws.

    friction[side] is (Coulomb Nm, viscous Nm s/rad) per joint, zero when the
    URDF has no <dynamics> element. load_friction[side] is the Coulomb friction
    added per Nm of gravity torque on the joint (gear losses grow with the load);
    it is this project's own attribute of <dynamics>, other URDF readers ignore it.
    """

    def __init__(self, urdf):
        self.path = Path(urdf)
        root = ET.parse(self.path).getroot()
        self.inertials = {}
        for link in root.findall("link"):
            inertial = link.find("inertial")
            if inertial is not None:
                self.inertials[link.attrib["name"]] = (
                    float(inertial.find("mass").attrib["value"]),
                    vector(inertial.find("origin").attrib.get("xyz", "0 0 0")),
                )
        self.joints = {j.attrib["name"]: j for j in root.findall("joint")}
        self.children = {}
        for joint in self.joints.values():
            self.children.setdefault(joint.find("parent").attrib["link"], []).append(joint)
        self.chains, self.friction, self.load_friction = {}, {}, {}
        for side in SIDES:
            origins, rotations, axes, links = [], [], [], []
            coulomb, viscous, load = np.zeros(7), np.zeros(7), np.zeros(7)
            for index, name in enumerate(ARM_NAMES):
                joint = self.joints[f"{side}_{name}_joint"]
                dynamics = joint.find("dynamics")
                if dynamics is not None:
                    coulomb[index] = float(dynamics.attrib.get("friction", 0.0))
                    viscous[index] = float(dynamics.attrib.get("damping", 0.0))
                    load[index] = float(dynamics.attrib.get("load_friction", 0.0))
                origin = joint.find("origin").attrib
                origins.append(vector(origin.get("xyz", "0 0 0")))
                rotations.append(rotation_rpy(vector(origin.get("rpy", "0 0 0"))))
                axes.append(vector(joint.find("axis").attrib["xyz"]))
                links.append(joint.find("child").attrib["link"])
            self.chains[side] = {
                "origins": origins,
                "rotations": rotations,
                "axes": axes,
                "links": links,
                "attached": [self.attached(link, set(links)) for link in links],
            }
            self.friction[side] = (coulomb, viscous)
            self.load_friction[side] = load
        self.parameters = {side: self.lumped(side) for side in SIDES}

    def attached(self, link, chain_links):
        """Links rigidly carried by a chain link: [(name, offset, rotation)]."""
        result = [(link, np.zeros(3), np.eye(3))]
        for joint in self.children.get(link, []):
            child = joint.find("child").attrib["link"]
            if child in chain_links:
                continue
            origin = joint.find("origin").attrib
            offset = vector(origin.get("xyz", "0 0 0"))
            rotation = rotation_rpy(vector(origin.get("rpy", "0 0 0")))
            for name, p, r in self.attached(child, chain_links):
                result.append((name, offset + rotation @ p, rotation @ r))
        return result

    def lumped(self, side):
        """7 x 4 array of [mass, mass*COM xyz] in each chain link frame."""
        rows = []
        for attached in self.chains[side]["attached"]:
            mass, moment = 0.0, np.zeros(3)
            for name, offset, rotation in attached:
                if name in self.inertials:
                    m, com = self.inertials[name]
                    mass += m
                    moment += m * (offset + rotation @ com)
            rows.append([mass, *moment])
        return np.array(rows)

    def frames(self, side, q):
        chain = self.chains[side]
        position, rotation = np.zeros(3), np.eye(3)
        positions, rotations, axes = [], [], []
        for origin, fixed, axis, angle in zip(
            chain["origins"], chain["rotations"], chain["axes"], q
        ):
            position = position + rotation @ origin
            rotation = rotation @ fixed
            axes.append(rotation @ axis)
            rotation = rotation @ rotation_axis(axis, angle)
            positions.append(position)
            rotations.append(rotation)
        return positions, rotations, axes

    def regressor(self, side, q):
        """7 x 28 matrix Y with holding torque = Y @ parameters.ravel()."""
        positions, rotations, axes = self.frames(side, q)
        y = np.zeros((7, 28))
        for j in range(7):
            # tau_j = -axis_j . sum_k (m_k (p_k - p_j) + R_k h_k) x g
            lever = np.cross(GRAVITY, axes[j])
            for k in range(j, 7):
                y[j, 4 * k] = -(positions[k] - positions[j]) @ lever
                y[j, 4 * k + 1 : 4 * k + 4] = -lever @ rotations[k]
        return y

    def torques(self, side, q, parameters=None):
        parameters = self.parameters[side] if parameters is None else parameters
        return self.regressor(side, q) @ np.asarray(parameters).ravel()

    def feedforward(self, targets, direction=None, velocity=None, friction_share=1.0):
        """Torques for {'left_arm': q7, 'right_arm': q7}, bounded by ARM_TAU_LIMIT.

        Gravity only by default. direction (-1..1 per joint, the way the joint
        moves; zero at rest) adds that share of the Coulomb friction (constant
        plus load_friction times the gravity torque), velocity (rad/s) the
        viscous friction.
        """
        result = {}
        for side in SIDES:
            name = side + "_arm"
            tau = self.torques(side, targets[name])
            coulomb, viscous = self.friction[side]
            if direction is not None:
                dry = coulomb + self.load_friction[side] * np.abs(tau)
                tau = tau + friction_share * dry * np.asarray(direction[name])
            if velocity is not None:
                tau = tau + viscous * np.asarray(velocity[name])
            result[name] = [float(t) for t in np.clip(tau, -np.array(ARM_TAU_LIMIT), ARM_TAU_LIMIT)]
        return result
