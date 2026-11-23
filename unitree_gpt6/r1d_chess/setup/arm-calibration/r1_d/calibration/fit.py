"""Fit arm mass/COM and joint friction to exploration logs and write the URDF.

Model for a joint that is moving slowly (inertia torque is negligible):

    tau_est = g + (c + mu * |g|) * sign(dq) + b * dq,   g = Y(q) @ pi

Y is the gravity regressor of the URDF chain, pi the lumped [mass, mass*COM]
of the seven arm bodies, c Coulomb friction, mu the friction added per Nm of
gravity load on the joint (gear losses), b viscous friction. Rows are
weighted by 1/kp, so the fit minimises the position error that a wrong
feedforward torque would cause. Parameters that the data cannot see stay at
the official URDF values (ridge prior).
"""

import argparse
import json
from pathlib import Path
import re
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from explore import CMD, DQ, FF, LOST, MOVING, Q, T, TAU
from r1_d.sdk.dynamics import ArmGravity, GRAVITY, SIDES
from r1_d.sdk.model import ARM_KP, ARM_NAMES, urdf

# Row weights 1/KP (the source gains): a torque error counts more on the weaker joints.
KP = np.array((50.0, 50.0, 40.0, 40.0, 30.0, 30.0, 30.0))
GAIN = np.array(ARM_KP)  # degrees are reported at the gains the controller uses
V_MIN = 0.03  # rad/s: slower joints have no defined friction direction
SIDE_SLICE = {"left": slice(0, 7), "right": slice(7, 14)}
SIDE_LOST = {"left": 0x7F | 1 << 14, "right": 0x7F << 7 | 1 << 15}


def batch_rotation(axis, angle):
    x, y, z = axis
    k = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + np.sin(angle)[:, None, None] * k + (1.0 - np.cos(angle))[:, None, None] * (k @ k)


def batch_regressor(model, side, q):
    """N x 7 x 28 gravity regressors for N joint vectors."""
    chain = model.chains[side]
    n = len(q)
    position, rotation = np.zeros((n, 3)), np.tile(np.eye(3), (n, 1, 1))
    positions, rotations, axes = [], [], []
    for origin, fixed, axis, angle in zip(chain["origins"], chain["rotations"], chain["axes"], q.T):
        position = position + rotation @ origin
        rotation = rotation @ fixed
        axes.append(rotation @ axis)
        rotation = rotation @ batch_rotation(axis, angle)
        positions.append(position)
        rotations.append(rotation)
    y = np.zeros((n, 7, 28))
    for j in range(7):
        lever = np.cross(GRAVITY, axes[j])
        for k in range(j, 7):
            y[:, j, 4 * k] = -np.einsum("ni,ni->n", positions[k] - positions[j], lever)
            y[:, j, 4 * k + 1 : 4 * k + 4] = -np.einsum("ni,nij->nj", lever, rotations[k])
    return y


def load(directories, stride=1):
    """All recorded rows of the given run directories, and the per-waypoint statics."""
    rows, statics = [], []
    for directory in directories:
        directory = Path(directory)
        chunks = sorted(directory.glob("chunk_*.npz"))
        if chunks:
            rows.append(np.vstack([np.load(c)["data"] for c in chunks]))
        path = directory / "waypoints.jsonl"
        if path.exists():
            statics += [json.loads(line) for line in path.read_text().splitlines()]
    data = np.vstack(rows)
    return data, statics


def moving_rows(data, side, stride=4):
    """Per joint: rows where it moves steadily in the commanded direction."""
    s = SIDE_SLICE[side]
    t = data[:, T]
    dt = np.gradient(t)
    command_rate = np.gradient(data[:, CMD:FF][:, s], axis=0) / np.maximum(dt, 1e-3)[:, None]
    dq = data[:, DQ:TAU][:, s]
    ok = (data[:, MOVING] > 0) & ((data[:, LOST].astype(np.int64) & SIDE_LOST[side]) == 0)
    ok &= (dt > 0) & (dt < 0.1)
    masks = (
        ok[:, None]
        & (np.abs(command_rate) > V_MIN)
        & (np.abs(dq) > 0.5 * V_MIN)
        & (np.sign(dq) == np.sign(command_rate))
    )
    keep = np.zeros(len(data), bool)
    keep[::stride] = True
    return masks & keep[:, None]


class Fit:
    """Normal equations of one side.

    Columns: 28 gravity, 7 Coulomb, 7 viscous, 7 offset, 7 load friction. The load
    friction column needs the gravity torque itself: `gravity` is the estimate of
    the lumped parameters it is computed with (default: the model's own).
    """

    P = 56

    def __init__(self, model, side, gravity=None):
        self.model, self.side = model, side
        self.gravity = np.asarray(model.parameters[side] if gravity is None else gravity).ravel()
        self.ata, self.atb = np.zeros((self.P, self.P)), np.zeros(self.P)
        self.count = np.zeros(7, int)

    def design(self, data, masks):
        """Rows (weighted by 1/kp), targets and joint index of the selected samples."""
        s = SIDE_SLICE[self.side]
        used = masks.any(1)
        q, dq, tau = (data[used][:, a:b][:, s] for a, b in ((Q, DQ), (DQ, TAU), (TAU, CMD)))
        y = batch_regressor(self.model, self.side, q)
        load = np.abs(y @ self.gravity)
        blocks, targets, joints = [], [], []
        for j in range(7):
            m = masks[used][:, j]
            a = np.zeros((m.sum(), self.P))
            a[:, :28] = y[m, j]
            a[:, 28 + j] = np.sign(dq[m, j])
            a[:, 35 + j] = dq[m, j]
            a[:, 42 + j] = 1.0
            a[:, 49 + j] = np.sign(dq[m, j]) * load[m, j]
            blocks.append(a / KP[j])
            targets.append(tau[m, j] / KP[j])
            joints.append(np.full(m.sum(), j))
        return np.vstack(blocks), np.concatenate(targets), np.concatenate(joints)

    def add(self, data, masks, chunk=20000):
        for start in range(0, len(data), chunk):
            a, b, joints = self.design(data[start : start + chunk], masks[start : start + chunk])
            self.ata += a.T @ a
            self.atb += a.T @ b
            self.count += np.bincount(joints, minlength=7)

    def solve(
        self, sigma_rad, mass_fraction, com_metres, friction_sigma=3.0, offsets=False, load_sigma=0.0
    ):
        """Ridge solution toward the model's own parameters; returns a parameter dict."""
        prior = np.zeros(self.P)
        prior[:28] = self.model.parameters[self.side].ravel()
        sigma = np.empty(self.P)
        for k in range(7):
            mass = prior[4 * k]
            sigma[4 * k] = mass_fraction * mass
            sigma[4 * k + 1 : 4 * k + 4] = com_metres * mass
        sigma[28:42] = friction_sigma
        sigma[42:49] = 1.0 if offsets else 1e-9
        sigma[49:] = load_sigma or 1e-9
        # Solve for the change in units of its prior standard deviation.
        scale = np.diag(sigma)
        lhs = scale @ self.ata @ scale / sigma_rad**2 + np.eye(self.P)
        rhs = scale @ (self.atb - self.ata @ prior) / sigma_rad**2
        solution = prior + sigma * np.linalg.solve(lhs, rhs)
        return {
            "pi": solution[:28].reshape(7, 4),
            "coulomb": solution[28:35],
            "viscous": solution[35:42],
            "offset": solution[42:49],
            "load": solution[49:],
        }


def predict(model, side, parameters, q, dq=None):
    gravity = np.einsum("njp,p->nj", batch_regressor(model, side, q), parameters["pi"].ravel())
    tau = gravity + parameters["offset"]
    if dq is not None:
        dry = parameters["coulomb"] + parameters.get("load", 0.0) * np.abs(gravity)
        tau = tau + dry * np.sign(dq) + parameters["viscous"] * dq
    return tau


def moving_residuals(model, side, parameters, data, masks):
    """Per joint: RMS and 95th percentile of the torque residual, in Nm and in degrees at kp."""
    s = SIDE_SLICE[side]
    used = masks.any(1)
    d = data[used]
    residual = d[:, TAU:CMD][:, s] - predict(
        model, side, parameters, d[:, Q:DQ][:, s], d[:, DQ:TAU][:, s]
    )
    result = {}
    for j, name in enumerate(ARM_NAMES):
        r = residual[masks[used][:, j], j]
        if len(r) == 0:
            continue
        result[name] = {
            "samples": int(len(r)),
            "rms_nm": float(np.sqrt((r**2).mean())),
            "p95_nm": float(np.percentile(np.abs(r), 95)),
            "rms_deg": float(np.degrees(np.sqrt((r**2).mean()) / GAIN[j])),
            "p95_deg": float(np.degrees(np.percentile(np.abs(r), 95) / GAIN[j])),
        }
    return result


def static_residuals(model, side, parameters, statics):
    """Holding torque minus modelled gravity at the waypoints (friction holds the rest)."""
    s = SIDE_SLICE[side]
    rows = [w for w in statics if not w.get("lost", 0) & SIDE_LOST[side]]
    q = np.array([w["q"][s] for w in rows])
    tau = np.array([w["tau_est"][s] for w in rows])
    residual = tau - predict(model, side, parameters, q)
    return {
        name: {
            "samples": len(rows),
            "rms_nm": float(np.sqrt((residual[:, j] ** 2).mean())),
            "p95_nm": float(np.percentile(np.abs(residual[:, j]), 95)),
            "max_nm": float(np.abs(residual[:, j]).max()),
            "rms_deg": float(np.degrees(np.sqrt((residual[:, j] ** 2).mean()) / GAIN[j])),
        }
        for j, name in enumerate(ARM_NAMES)
    }


def tracking(statics):
    """Measured minus commanded angle at the waypoints, degrees, per side and joint."""
    result = {}
    for side in SIDES:
        s = SIDE_SLICE[side]
        rows = [w for w in statics if not w.get("lost", 0) & SIDE_LOST[side]]
        error = np.degrees(np.array([np.subtract(w["q"][s], w["commanded"][s]) for w in rows]))
        result[side] = {
            name: {
                "samples": len(rows),
                "mean": float(error[:, j].mean()),
                "rms": float(np.sqrt((error[:, j] ** 2).mean())),
                "p95": float(np.percentile(np.abs(error[:, j]), 95)),
                "max": float(np.abs(error[:, j]).max()),
            }
            for j, name in enumerate(ARM_NAMES)
        }
    return result


def link_inertials(model, side, fitted):
    """Turn lumped body parameters back into per-link mass/COM.

    A chain link with rigidly attached children (the gripper) keeps its own
    values; the change is assigned to the first attached child (gripper base).
    """
    result = {}
    for k, attached in enumerate(model.chains[side]["attached"]):
        mass, moment = fitted[k, 0], fitted[k, 1:].copy()
        target = attached[1] if len(attached) > 1 else attached[0]
        for name, offset, rotation in attached:
            if name != target[0] and name in model.inertials:
                m, com = model.inertials[name]
                mass -= m
                moment -= m * (offset + rotation @ com)
        name, offset, rotation = target
        if mass <= 0.02:
            raise RuntimeError(f"Fitted mass of {name} is not physical: {mass:.3f} kg")
        result[name] = (mass, rotation.T @ (moment / mass - offset))
    return result


def write_urdf(source, destination, inertials, source_model, friction=None):
    """Copy the source URDF with new arm inertials and joint <dynamics>.

    friction maps joint name to (coulomb Nm, viscous Nm s/rad, load friction Nm/Nm).
    """
    # newline="" keeps the source line endings (the official file uses CRLF).
    with open(source, newline="") as file:
        text = file.read()
    eol = "\r\n" if "\r\n" in text else "\n"
    for name, (mass, com) in inertials.items():
        pattern = re.compile(
            r'(<link name="' + re.escape(name) + r'">\s*<inertial>)(.*?)(</inertial>)', re.S
        )
        match = pattern.search(text)
        if match is None:
            raise RuntimeError(f"No inertial block for {name}")
        block = match.group(2)
        scale = mass / source_model.inertials[name][0]
        block = re.sub(
            r'(<origin xyz=")[^"]*(")',
            lambda m: f"{m.group(1)}{com[0]:.6f} {com[1]:.6f} {com[2]:.6f}{m.group(2)}",
            block,
            count=1,
        )
        block = re.sub(
            r'(<mass value=")[^"]*(")', lambda m: f"{m.group(1)}{mass:.6f}{m.group(2)}", block
        )
        # Keep the inertia shape; scale it with the mass.
        block = re.sub(
            r'\b(i[xyz]{2})="([^"]*)"',
            lambda m: f'{m.group(1)}="{float(m.group(2)) * scale:.10f}"',
            block,
        )
        text = text[: match.start(2)] + block + text[match.end(2) :]
    for name, (coulomb, viscous, load) in (friction or {}).items():
        pattern = re.compile(
            r'(<joint name="' + re.escape(name) + r'" type="revolute">.*?)(\s*<dynamics[^>]*/>)?'
            r"(\s*</joint>)",
            re.S,
        )
        match = pattern.search(text)
        if match is None:
            raise RuntimeError(f"No revolute joint {name}")
        line = (
            f'{eol}    <dynamics damping="{viscous:.4f}" friction="{coulomb:.4f}" '
            f'load_friction="{load:.4f}"/>'
        )
        text = text[: match.end(1)] + line + match.group(3) + text[match.end() :]
    with open(destination, "w", newline="") as file:
        file.write(text)


def rest_shares(model, side, parameters, statics):
    """Friction left when a joint has stopped, as a share of its Coulomb friction.

    At a waypoint the holding torque minus gravity is the friction that keeps the joint
    where it stopped; its part along the arrival direction, averaged over the waypoints
    the joint moved to, is what the feedforward must supply to stop without a bias.
    """
    s = SIDE_SLICE[side]
    quiet = dict(parameters, coulomb=np.zeros(7), viscous=np.zeros(7), load=np.zeros(7))
    pairs = [
        (a, b)
        for a, b in zip(statics, statics[1:])
        if b["index"] == a["index"] + 1 and not b.get("lost", 0) & SIDE_LOST[side]
    ]
    result = np.ones(7)
    if not pairs:
        return result
    q = np.array([b["q"][s] for _, b in pairs])
    step = np.array([np.subtract(b["commanded"][s], a["commanded"][s]) for a, b in pairs])
    gravity = predict(model, side, quiet, q)
    held = np.array([b["tau_est"][s] for _, b in pairs]) - gravity
    dry = np.maximum(parameters["coulomb"], 0.0) + np.maximum(parameters["load"], 0.0) * np.abs(gravity)
    for j in range(7):
        moved = np.abs(step[:, j]) > 0.02
        if moved.sum() >= 30 and dry[moved, j].mean() > 0.02:
            rest = (np.sign(step[moved, j]) * held[moved, j]).mean()
            result[j] = np.clip(rest / dry[moved, j].mean(), 0.2, 1.2)
    return result


def fit(args):
    source = ArmGravity(args.source_urdf)
    data, statics = load(args.data)
    record = {
        "source_urdf": str(args.source_urdf),
        "data": [str(p) for p in args.data],
        "rows": int(len(data)),
        "sigma_rad": args.sigma_rad,
        "prior_mass_fraction": args.mass_fraction,
        "prior_com_metres": args.com_metres,
        "rest_share": args.rest_share,
        "load_sigma": args.load_sigma,
        "sides": {},
    }
    inertials, friction = {}, {}
    for side in SIDES:
        masks = moving_rows(data, side, args.stride)
        parameters = None
        # The load friction columns use the gravity torque: repeat with the fitted gravity.
        for _ in range(3 if args.load_sigma else 1):
            problem = Fit(source, side, None if parameters is None else parameters["pi"])
            problem.add(data, masks)
            parameters = problem.solve(
                args.sigma_rad, args.mass_fraction, args.com_metres, load_sigma=args.load_sigma
            )
        if args.rest_share == ["auto"]:
            shares = rest_shares(source, side, parameters, statics)
        elif args.rest_share:
            start = SIDES.index(side) * 7
            shares = np.array([float(v) for v in args.rest_share[start : start + 7]])
        else:
            shares = np.ones(7)
        official = {
            "pi": source.parameters[side],
            "coulomb": np.zeros(7),
            "viscous": np.zeros(7),
            "offset": np.zeros(7),
            "load": np.zeros(7),
        }
        friction_only = dict(
            official,
            coulomb=parameters["coulomb"],
            viscous=parameters["viscous"],
            load=parameters["load"],
        )
        links = link_inertials(source, side, parameters["pi"])
        inertials.update(links)
        for j, name in enumerate(ARM_NAMES):
            # Negative fitted friction is noise: store zero. --rest-share scales the stored
            # friction to what is left when the joint has come to rest; that was needed while
            # the session kept the friction feedforward after the stop (v3..v5). The session
            # now fades it out, so the URDF holds the full moving friction (no --rest-share).
            share = shares[j]
            friction[f"{side}_{name}_joint"] = (
                share * max(0.0, parameters["coulomb"][j]),
                max(0.0, parameters["viscous"][j]),
                share * max(0.0, parameters["load"][j]),
            )
        record["sides"][side] = {
            "samples_per_joint": problem.count.tolist(),
            "rest_share": shares.tolist(),
            "coulomb_nm": parameters["coulomb"].tolist(),
            "load_friction": parameters["load"].tolist(),
            "viscous_nm_s_per_rad": parameters["viscous"].tolist(),
            "pi": parameters["pi"].tolist(),
            "moving_residual_official_gravity": moving_residuals(
                source, side, friction_only, data, masks
            ),
            "moving_residual_fitted": moving_residuals(source, side, parameters, data, masks),
            "static_residual_official_gravity": static_residuals(source, side, official, statics),
            "static_residual_fitted": static_residuals(source, side, parameters, statics),
            "links": {
                name: {
                    "mass_kg": [source.inertials[name][0], float(mass)],
                    "com_m": [source.inertials[name][1].tolist(), com.tolist()],
                    "com_shift_mm": float(np.linalg.norm(com - source.inertials[name][1]) * 1000),
                }
                for name, (mass, com) in links.items()
            },
        }
    if args.output:
        write_urdf(args.source_urdf, args.output, inertials, source, friction)
        record["output"] = str(args.output)
    if args.record:
        args.record.write_text(json.dumps(record, indent=1) + "\n")
    report(record)
    return record


def report(record):
    for side in SIDES:
        s = record["sides"][side]
        print(side, "samples per joint", s["samples_per_joint"])
        print("   coulomb Nm ", np.round(s["coulomb_nm"], 3).tolist())
        print("   load Nm/Nm ", np.round(s.get("load_friction", []), 3).tolist())
        print("   rest share ", np.round(s.get("rest_share", []), 3).tolist())
        print("   viscous    ", np.round(s["viscous_nm_s_per_rad"], 3).tolist())
        for key in (
            "moving_residual_official_gravity",
            "moving_residual_fitted",
            "static_residual_official_gravity",
            "static_residual_fitted",
        ):
            print(
                f"   {key:34s} rms Nm",
                [round(v["rms_nm"], 3) for v in s[key].values()],
                "rms deg",
                [round(v["rms_deg"], 2) for v in s[key].values()],
            )
        for name, link in s["links"].items():
            print(
                f"   {name}: mass {link['mass_kg'][0]:.3f} -> {link['mass_kg'][1]:.3f} kg, "
                f"COM shift {link['com_shift_mm']:.1f} mm"
            )


def evaluate(args):
    """Residuals of an existing URDF on recorded data, and the tracking error of that run."""
    model = ArmGravity(args.urdf)
    data, statics = load(args.data)
    result = {"urdf": str(args.urdf), "data": [str(p) for p in args.data], "sides": {}}
    for side in SIDES:
        masks = moving_rows(data, side, args.stride)
        problem = Fit(model, side)
        problem.add(data, masks)
        # Friction only: gravity parameters are pinned to the URDF.
        friction = problem.solve(args.sigma_rad, 1e-9, 1e-9, load_sigma=args.load_sigma)
        quiet = dict(friction, coulomb=np.zeros(7), viscous=np.zeros(7), load=np.zeros(7))
        result["sides"][side] = {
            "coulomb_nm": friction["coulomb"].tolist(),
            "load_friction": friction["load"].tolist(),
            "moving_residual": moving_residuals(model, side, friction, data, masks),
            "static_residual": static_residuals(model, side, quiet, statics),
        }
        print(side, "coulomb", np.round(friction["coulomb"], 3).tolist())
        print("   load", np.round(friction["load"], 3).tolist())
        for key in ("moving_residual", "static_residual"):
            print(
                f"   {key:16s} rms Nm",
                [round(v["rms_nm"], 3) for v in result["sides"][side][key].values()],
                "rms deg",
                [round(v["rms_deg"], 2) for v in result["sides"][side][key].values()],
            )
    result["tracking_deg"] = tracking(statics)
    for side in SIDES:
        t = result["tracking_deg"][side]
        print(side, "tracking at waypoints (deg): rms", [round(v["rms"], 2) for v in t.values()])
        print("   p95", [round(v["p95"], 2) for v in t.values()], "max", [round(v["max"], 2) for v in t.values()])
    if args.record:
        args.record.write_text(json.dumps(result, indent=1) + "\n")
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    sub = p.add_subparsers(dest="command", required=True)
    s = sub.add_parser("fit")
    s.add_argument("data", type=Path, nargs="+")
    s.add_argument("--source-urdf", type=Path, default=urdf)
    s.add_argument("--output", type=Path)
    s.add_argument("--record", type=Path)
    s.add_argument("--sigma-rad", type=float, default=0.004)
    s.add_argument("--mass-fraction", type=float, default=0.5)
    s.add_argument("--com-metres", type=float, default=0.03)
    s.add_argument("--stride", type=int, default=4)
    s.add_argument("--load-sigma", type=float, default=0.3, help="0: no load friction")
    s.add_argument("--rest-share", nargs="+", help="auto, or 14 values: left 7, right 7 joints")
    s = sub.add_parser("evaluate")
    s.add_argument("urdf", type=Path)
    s.add_argument("data", type=Path, nargs="+")
    s.add_argument("--record", type=Path)
    s.add_argument("--sigma-rad", type=float, default=0.004)
    s.add_argument("--stride", type=int, default=4)
    s.add_argument("--load-sigma", type=float, default=0.3, help="0: no load friction")
    args = p.parse_args()
    {"fit": fit, "evaluate": evaluate}[args.command](args)


if __name__ == "__main__":
    main()
