"""Moving-sample torque residual of URDFs (their own friction, unscaled) on runs.
python heldout.py URDF... -- RUN[:lo:hi]..."""
import sys

import numpy as np

import fit as F
from explore import CMD, DQ, INDEX, Q, TAU
from r1_d.sdk.dynamics import ArmGravity, SIDES

np.set_printoptions(precision=3, suppress=True, linewidth=220)
args = sys.argv[1:]
split = args.index("--")
parts = []
for spec in args[split + 1 :]:
    name, *limits = spec.split(":")
    data = F.load([name])[0]
    if limits:
        data = data[(data[:, INDEX] >= int(limits[0])) & (data[:, INDEX] < int(limits[1]))]
    parts.append(data)
data = np.vstack(parts)
print("rows", len(data))
for name in args[:split]:
    model = ArmGravity(name)
    scores = []
    for side in SIDES:
        s = F.SIDE_SLICE[side]
        masks = F.moving_rows(data, side, 4)
        used = masks.any(1)
        d = data[used]
        p = {"pi": model.parameters[side], "coulomb": model.friction[side][0], "viscous": model.friction[side][1],
             "offset": np.zeros(7), "load": model.load_friction[side]}
        r = d[:, TAU:CMD][:, s] - F.predict(model, side, p, d[:, Q:DQ][:, s], d[:, DQ:TAU][:, s])
        rms = np.array([np.sqrt((r[masks[used][:, j], j] ** 2).mean()) for j in range(7)])
        scores.append(1000 * np.sqrt(((rms / F.KP) ** 2).mean()))
        print(f"  {side:5s} rms Nm {rms}  weighted mrad {scores[-1]:.3f}")
    print(name.split("/")[-1], "both", round(float(np.sqrt(np.mean(np.square(scores)))), 3))
