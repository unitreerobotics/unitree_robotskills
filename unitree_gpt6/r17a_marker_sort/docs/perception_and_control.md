# Control and perception notes

English | [中文](perception_and_control.zh.md)

- The controller is PD only: steady-state joint error is 0.01–0.025 rad, 2–8 mm at the end effector. Use **measured joint angles** for all geometry. For accuracy use an outer loop (`rt.servo`, `task.servo_point`), which converges to about 1.5 mm.
- Measured joint angles near zero can slightly exceed the URDF lower limits (J2, J4 by about −0.03 rad). Path checks allow 0.06 rad of margin on measured angles; targets stay strictly inside the limits.
- The arm still swings back after a motion ends. Wait for several consecutive unchanged samples before taking a photo.
- Use multi-seed IK (`rt.ik_all`) and choose by small elbow twist. For gripping a standing pen, pitching down 25° reaches much further than horizontal or vertical.
- To leave the 25° pitched grip pose, lift straight up first; an oblique retreat often has no solution.
- Path checks while holding a pen must include the pen itself (against the desk, the box and the arm links).
- Judge contact by "height pushed up", not by a change in tracking error (false alarms).
- Any last step whose success depends on contact (fingertip height when grasping, tip position when lowering) is closed on the measured pose, never on the command.
- Straight-down reach is roughly x∈[−0.2, 0.2], y∈[−0.35, −0.15] m (world frame; in front of the arm is −y). At y=−0.40 only the centre is barely reachable.
- With the gripper open, the two fingers hide the lower left and right of the image; the TCP projects near pixel (632, 412). Keep pens and caps out of those areas when searching, and change the view if they are hidden.
- When choosing the lowering configuration, keep the whole path at least 0.05 rad from joint limits. A configuration that hugs a limit jams half-way once the steady-state correction is added.
- The gripper must be fully open before `look`. Half-open fingers hide the pens; zero pens are detected and the pen list is overwritten. An empty grasp does not move the pen, but `look` again anyway.
- Fingertip target for a pen lying flat is 4 mm above the desk (at 2–3 mm it digs into the desk in places).
- Under this camera the box's translucent plastic and the white pen bodies look pale blue (hue about 100–106, saturation 85–120) and get taken for coloured caps. The lattice fit requires higher saturation in that hue band.
- Three guards in the lattice fit: a rigid square grid (with free pitch it stretches to 15×21.6 mm to fit the data), a single hue inside each window (otherwise a point between two caps is also "zero residual"), and a coarse rotation-plus-translation search around the seed.
- Find pens lying flat with a robust axis (principal direction plus median); a paper scrap under the pen or arrows printed on the body must not matter. The ring is a dark line across the whole body; printing is not.
- For colour-based detection, sample the target first and then set thresholds, and regress on saved images (keep a set of good and bad images from your own scene as an offline self-check and run it after any detection change). A beige desk and yellow caps are separated by saturation. With same-coloured caps in the box, find "the one standing proud" as the **largest compact blob**, or it merges into another blob.
- Test a newly written image check on good and bad images before wiring it into the procedure; a check that gives the same answer for both is no check. "Pen hangs straight" holds only in the carry pose.

## Why it is done this way

- **Absolute localization is only about 4 mm (1 cm worst case).** That is enough for grasping a pen lying flat, but the slot tolerance is about ±3 mm, so insertion cannot rely on it; it relies on the caging described below.
- **Do not force it in.** Holding the cap, the tip is 14 cm away and the error is ≥5 mm, more than the hole tolerance. Pushing on against the grid only jams it, so do not search for the hole with a firm-grip "lift, shift, push".
- **A loose grip plus gravity self-centres**, but only if the pen was first straightened above the slot centre. Grip firmly and straighten first, then loosen into a cage and jiggle; if you release first and jiggle afterwards, the pen has already tilted away.
- **Whether a pen can be grasped directly depends on its heading and distance.** Pens lying sideways or diagonally, 15–30 cm from the base, can be grasped directly; pens pointing at or away from the robot, or far away, need the rotated grasp; restage when neither has a solution.
