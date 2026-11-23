---
name: r17a-marker-sort
description: Put markers scattered on a desk back into their numbered slots in a marker box (a 4×6 box in this example), on a real R1-7a arm with a Dex1-1 gripper and a wrist RGB camera. Locate the box, find the pens, grasp, read the colour number, insert, verify, and re-insert pens left standing tilted. Tools are in src/.
metadata:
  version: "1.0.0"
  language: en
---

# R1-7a marker sorting (real robot)

English | [中文](SKILL.zh.md)

All numbers here were measured on the real robot; they are starting points, not laws. Confirm them again with different pens, a different box or a different desk. Preparation (joint control interface, IK, camera calibration) and the tools are described in the [README](README.md).

Two supplementary documents, to read when the situation applies:

- The desk is white: pen bodies are the same colour as the desk, so finding pens, locating the box and the hang-straight check work differently. Read the [white desk notes](docs/white_desk.md) before starting.
- Changing detection, path, grasp or lowering parameters, investigating a failure, or wanting to know why something is done the way it is: read the [control and perception notes](docs/perception_and_control.md).

## Before you start

- Start only when three things are ready (README, "Before you start"): the user's own joint control wired into `state`, `move`, `gripper` in `src/robot/rt.py` and `frame` in `src/robot/grab.py`; IK from `src/ik/kinematics.py` with the R1-7a URDF; the camera calibrated with `src/calibration/`, so that `src/calib.json` exists.
- The desk may be dark or white; pens may lie any way, and step 3 says how to grasp each.
- The user's own control program enables and holds the arm; do not start or restart it. Read `state()`: `armed=true`, `fault=null`, gripper calibrated, camera 1280×720.
- **Never call a stop or disconnect that releases torque, and do not suggest the user casually restart the control program.** All of these release torque and the arm drops. On an anomaly, keep holding the current command and report.
- Run motion scripts in the foreground, one step at a time. Killing a script does not stop a trajectory already sent, and it can leave the arm mid-way with a pen in the gripper. After an interruption, read the gripper state before deciding the next step.
- Path checks must block with `assert`, not print a result and go anyway. After one failed assertion the next step starts from a half-way pose; find out where the hand is first.
- In a chain of steps, any failure must stop the whole chain; read the intermediate check results before continuing.
- When stuck, say why at once and give the user choices; do not switch on your own to a method the user has ruled out. Take the user's reading of a symptom seriously.

## This box and these pens (measured)

- Pens are 153–155 mm long. One end has a **large square cap** (with a dark ring about 30 mm from the end face); the other end is the fine tip. The big cap goes up in the box, so **always grasp the big-cap end**. Both caps carry the colour number; tell the big-cap end only by the ring.
- Slot pitch is 17.5–17.8 mm. Cap tops sit 150–151 mm above the desk with the pen standing on the box floor.
- About 65–70 mm below the cap tops there is a **grid of square holes**. The fine tip must enter a hole (tolerance about ±3 mm), and the square body must turn square to the hole before the pen drops to the floor. The height at which a pen sticks tells you why:

| Cap above its neighbours | State | Action |
| --- | --- | --- |
| about 85–90 mm | Tip resting on the grid, not in a hole | `reseat` |
| about 55–59 mm | Tip in the hole, body not square | `reseat` (grip at the current height) |
| 0 | Seated | Done |

- The colour number can only be read from the wrist image after pickup (big cap facing the camera). Take the box layout from the neighbouring numbers you see on the spot; do not apply a fixed table.

## Procedure (look at the image between steps)

Run every tool from `src/` as `python3 -m <dir>.<tool>`; each step is `python3 -m sorting.sort RUN <step>`, where `RUN` is the directory for this run.

1. **Locate the box. Every run, and again whenever the box was touched.** Take 2–3 look-down poses (`perception.view_plan`; reuse the previous run's poses if the box has roughly not moved). `perception.box` fits the slot lattice from the coloured cap tops across views, seeded with the previous result; if the box turned more than ±45° or moved more than 4 cm, give it a new seed first. **You must open the overlay `*_box.jpg` and check that every circle sits on a cap**, and check cap height 150±2 mm and pitch about 17.8 mm. The reprojection residual proves nothing: a fit off by half a slot or a full slot still shows 1.6 px. Confirm the empty slots on the same image. The views must be close to straight down with none of the 24 slots hidden by a finger. Check the overlays of all three views; one alone can hide the skewed one. The fit fixes cap height at 150.5 mm and uses a 17.85 mm prior on the pitch: with both free, oblique views pull the pitch to 17.4 mm, the far rows end up 5–8 mm off, and a pen gets put down on a neighbour's cap. Rearranging the scene or bumping the box moves it by a few millimetres up to 1 cm; if the user says it was touched, redo it.
2. **`look`.** High overview; find the pens on the desk and their big-cap ends (by the ring, not by thickness). For a pen hidden by a finger, look from another J1 angle (`look 0.45`); a partly hidden pen measures short, but its big-cap end position is still usable. If the detected count differs from what you count by eye, fix detection before moving (a pen lying on a paper scrap can be missed). Crossing or touching pens are split into separate axes and flagged `CROSSES`; if a pen runs behind a finger, change the angle (`look 0.3`) so every pen is fully in view. **With many pens in a pile**, zoom in and mark the pens not found automatically (dark caps, crossed pens) with `addpen`, taking the big-cap end by the ring.
3. **`pick K`: grasp the big-cap end.** Grasp straight down, 1–3 cm from the big-cap end face, **camera side towards the big cap**. Lift in small steps, turn to the carry pose (gripper horizontal, pen hanging, big cap facing the camera), read the number. Key points:
   - **Grip firmly.** The gripper is position-controlled; grip torque is roughly 26 Nm × (opening blocked by the pen − close command). A command of 0.13 gives only 0.7 Nm and the pen swings in the fingers after lift-off; use **0.08** (1.5–1.9 Nm). If a pen is crooked, suspect a weak grip first, not a collision.
   - **Descend to the computed height, stop when the fingers touch the desk, and close.** The computed fingertip height varies by up to 8 mm with arm pose: too high grasps above the pen (empty grasp), while forcing it down to "the computed 4 mm" jams the fingers into the desk. Correct towards the target height and stop as soon as the tracking error starts to grow. Closing while touching the desk is fine.
   - Keep the whole grasp plan at least 0.10 rad from every joint limit. Use `sorting.pick_direct RUN K [--wide]`; it picks the grasp from the table below. `--dry` only plans. When it reports no solution, check the stages offline (IK / clearance / path / descent / lift / to carry) to see where it dies.

   Choose the grasp by how the pen lies:

| How the pen lies | Grasp |
| --- | --- |
| Sideways or diagonal | Standard grasp: straight down, gripper across the pen body |
| Pointing at or away from the robot, or far from the base | Rotate the gripper ±10–30° about the vertical and move the grasp point to 30–45 mm from the big-cap end; grasp in place (the fingers straighten the pen). With no other pens nearby, up to ±60° (`--wide`) |
| Right next to the box | After lifting, move sideways away from the box before turning, or the pen tail sweeps the box during the turn |
| Crossing, touching or stacked with other pens | See step 4 |
| None of the above has a solution (too far, at a joint limit) | Restage; see step 5 |

| Case | Close command | Opening feedback | Torque |
| --- | --- | --- | --- |
| Holding a cap (grasp) | 0.08 | 0.144–0.150 | −1.5 to −1.9 |
| Holding a cap (light grip for `reseat`) | 0.13 | 0.155–0.159 | −0.65 to −0.77 |
| Holding the pen body (restage) | 0.13 | 0.175 | −1.2 |
| Empty grasp | 0.08 / 0.13 | 0.089 / 0.137 | −0.26 / −0.20 |

   The "is it holding" threshold follows the close command. Changing the command without changing the test reports a held pen as dropped.

4. **Stacked pens: no need to work out which is on top.** Grasp either, as long as the grasp point is at least 4 cm from the crossing and the open fingers do not land on the other pen (`pick` and `restage` check this and refuse). Take the one whose big-cap end is far from the crossing first. Taking the lower pen lets the upper one slide onto the desk. **`look` again after every pen you remove**, and re-mark: removing one pen often drags its neighbours, sometimes 40 cm away and out of reach. Take the pen you can identify, with a clear area around its big cap, first; identify the rest once the desk is simpler. A pen whose ring lies exactly under the crossing is reported with an unknown big end; take others first.
5. **When neither the direct nor the rotated grasp has a solution, `restage K`.** Grasp the pen body mid-way, lay it flat at (0.06, −0.20) along x with the big cap on the −x side, then `look` and `pick`. If another pen already lies at the staging spot, deal with that one first. In a mid-body grasp the pen sticks out 7.5 cm on each side; the restage path check includes pen-to-box clearance.
6. **If the pen swings crooked in the fingers, put it down and start again.** In the carry pose `pick` checks that the pen hangs straight; if not, `putdown X Y DX DY` lays it in a free spot for a new grasp. Never put a crooked pen into the box. The main cause is a weak grip (step 3); the tail hitting something is second.
7. **`place I J`.** If the pen sits differently in the fingers than nominal (cap low in the wrist image), set `GRASP_D` to the actual position before lowering. Search many IK solutions for the lowering configuration (the first few low-twist ones often hit the J6 limit half-way). Align the gripper heading with the lattice, servo the tip onto the slot while it is still above the caps, lower until the grid pushes the pen up by more than 5 mm or the fingers approach the caps, release, retreat.
8. **`verify`.** A look-down image confirms the cap is flush with its neighbours and in the right slot. Of straight pens put in the right way up, about a third go through the grid hole to the floor directly, a third rest on the grid and drop after release, and a third stand tilted after release (needing `reseat`). Tilted is normal, not an accident. Budget time for one extra recovery per pen.
9. **If it stands tilted, `sorting.reseat2 RUN I J`.** Triangulate this pen's cap from two views (if the two rays miss each other by more than 4 mm it is the wrong blob; do not use it) → gripper pitched down 25°, heading aligned with the lattice, **grip the cap firmly** with the pen 15 mm inside the fingers → straighten the pen above the slot centre with the tip just above the grid (grip point = neighbour cap top + 79 mm) → **loosen to opening 0.21** to form a cage (about 2 mm clearance per side) → jiggle in ±2/4/6 mm circles with ±8° yaw; the pen drops under its own weight. The first ray comes from the verify image, and the second view is chosen automatically, close and steep, with the cap not hidden by a finger. Pens put in the right way up almost always go in at the first attempt. For dark caps (purple, for example), pick the point by hand in both images (`sorting.reseat2 … px U V` plus `sorting.reseat_marked`). Judge whether it dropped from the look-down image. Then `verify` again and re-locate the box.

   **If the same pen has been recovered twice with the same symptom, stop and think of a different cause** (is it the right slot? is the gripper on the big-cap end?) instead of trying yet another variant.

## Reporting

Each run's images, fit results and log are in the `RUN` directory. Report the target count, the number confirmed placed on the look-down image, each pen's slot and evidence image, failures and recoveries, and time taken. Anything not confirmed on the look-down image does not count as success. If the scope was reduced, say so and do not change the denominator.
