"""Point-cloud self-collision check for both arms, from the URDF collision meshes.

Everything is expressed in waist_yaw_link. The static cloud holds the torso,
head, lift column and chassis for one lift height, waist angle and head pose.
"""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import cKDTree

from r1_d.sdk.dynamics import ArmGravity, SIDES, rotation_axis, rotation_rpy, vector

FINGER_TRAVEL = 0.041
# chain index -> group
GROUP_OF = {0: "shoulder", 1: "shoulder", 2: "upper", 3: "elbow", 4: "fore", 5: "hand", 6: "hand"}


def surface_points(path, spacing, rng):
    """Points no further apart than about `spacing` on a binary STL surface."""
    data = Path(path).read_bytes()
    count = int(np.frombuffer(data, "<u4", 1, 80)[0])
    if len(data) != 84 + 50 * count:
        raise ValueError(f"{path} is not a binary STL")
    dtype = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    tri = np.frombuffer(data, dtype, count, 84)["v"].astype(float)
    a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
    area = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
    n = np.ceil(area / (0.25 * spacing**2)).astype(int)
    index = np.repeat(np.arange(count), n)
    u, v = rng.random(len(index)), rng.random(len(index))
    flip = u + v > 1.0
    u[flip], v[flip] = 1.0 - u[flip], 1.0 - v[flip]
    points = a[index] + u[:, None] * (b - a)[index] + v[:, None] * (c - a)[index]
    points = np.vstack([points, tri.reshape(-1, 3)])
    keys = np.unique(np.round(points / spacing).astype(np.int64), axis=0, return_index=True)[1]
    return points[keys]


class Collision:
    def __init__(self, urdf, height, waist=0.0, head=(0.0, 0.0), spacing=0.012):
        self.model = ArmGravity(urdf)
        self.spacing = spacing
        rng = np.random.default_rng(0)
        root = ET.parse(urdf).getroot()
        meshes = Path(urdf).parent / "meshes"
        cloud = {}
        for link in root.findall("link"):
            mesh = link.find("collision").find("geometry").find("mesh").attrib["filename"]
            cloud[link.attrib["name"]] = surface_points(meshes / Path(mesh).name, spacing, rng)
        joints = {j.attrib["name"]: j for j in root.findall("joint")}

        def origin(name):
            o = joints[name].find("origin").attrib
            return vector(o.get("xyz", "0 0 0")), rotation_rpy(vector(o.get("rpy", "0 0 0")))

        # Arm clouds per side and group, in chain link frames.
        self.arm = {}
        for side in SIDES:
            groups = {}
            for k, attached in enumerate(self.model.chains[side]["attached"]):
                for name, offset, rotation in attached:
                    points = cloud[name] @ rotation.T + offset
                    parts = [points]
                    if "finger" in name:
                        joint = next(
                            j for j in joints.values() if j.find("child").attrib["link"] == name
                        )
                        axis = vector(joint.find("axis").attrib["xyz"])
                        parts.append(points + rotation @ axis * FINGER_TRAVEL)
                    groups.setdefault(GROUP_OF[k], []).append((k, np.vstack(parts)))
            self.arm[side] = groups

        # Static cloud in waist_yaw_link.
        static = {"torso": [cloud["waist_yaw_link"]]}
        p, r = origin("head_pitch_joint")
        r = r @ rotation_axis((0, 1, 0), head[0])
        static["head"] = [cloud["head_pitch_Link"] @ r.T + p]
        p2, r2 = origin("head_yaw_joint")
        p, r = p + r @ p2, r @ r2 @ rotation_axis((0, 0, 1), head[1])
        static["head"].append(cloud["head_yaw_Link"] @ r.T + p)
        # Below the waist joint, in pelvis (= lift_stage2) coordinates.
        lower = [cloud["pelvis_link"], cloud["lift_stage2_link"]]
        s2 = origin("lift_stage2_joint")[0]
        s1 = origin("lift_stage1_joint")[0]
        mount = origin("lift_mount_joint")[0]
        # The split of the height between the two stages is not reported: cover the extremes.
        for q2 in {height / 2.0, max(0.0, height - 0.425), min(height, 0.425)}:
            lower.append(cloud["lift_stage1_link"] - s2 - np.array([0.0, 0.0, q2]))
        mount_origin = -s2 - s1 - np.array([0.0, 0.0, height])
        lower.append(cloud["lift_mount_link"] + mount_origin)
        base_origin = mount_origin - mount
        lower.append(cloud["base_link"] + base_origin)
        for name in ("Left_Wheel", "Right_Wheel"):
            lower.append(cloud[name + "_link"] + base_origin + origin(name + "_joint")[0])
        static["lower"] = [np.vstack(lower) @ rotation_axis((0, 0, 1), -waist).T]
        self.static = {k: cKDTree(np.vstack(v)) for k, v in static.items()}

    def points(self, side, q):
        positions, rotations, _ = self.model.frames(side, q)
        return {
            group: np.vstack([pts @ rotations[k].T + positions[k] for k, pts in parts])
            for group, parts in self.arm[side].items()
        }

    def clearance(self, q_left, q_right):
        """Smallest distances (m) per category for one configuration."""
        clouds = {"left": self.points("left", q_left), "right": self.points("right", q_right)}
        result = {}
        for side in SIDES:
            c = clouds[side]
            distal = np.vstack([c["elbow"], c["fore"], c["hand"]])
            result[side + "_distal_static"] = min(
                tree.query(distal)[0].min() for tree in self.static.values()
            )
            result[side + "_upper_static"] = min(
                self.static[k].query(c["upper"])[0].min() for k in ("torso", "lower")
            )
            own = cKDTree(np.vstack([c["shoulder"], c["upper"]]))
            result[side + "_hand_self"] = own.query(np.vstack([c["hand"], c["fore"]]))[0].min()
        left = cKDTree(np.vstack(list(clouds["left"].values())))
        result["arm_arm"] = left.query(np.vstack(list(clouds["right"].values())))[0].min()
        return result

    def violation(self, q_left, q_right, margins):
        """Name of the first category below its margin, or None."""
        for name, distance in self.clearance(q_left, q_right).items():
            key = name.split("_", 1)[1] if name != "arm_arm" else name
            if distance < margins[key]:
                return name
        return None

    def path_violation(self, start, end, margins, step=0.06):
        """Check the straight joint-space path; start/end are (q_left, q_right)."""
        a = np.concatenate(start)
        b = np.concatenate(end)
        n = max(1, int(math.ceil(np.abs(b - a).max() / step)))
        for i in range(n, 0, -1):  # the far end is the most likely to fail
            q = a + (b - a) * i / n
            bad = self.violation(q[:7], q[7:], margins)
            if bad:
                return bad
        return None
