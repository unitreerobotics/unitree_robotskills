# Marking the 90 Intersections of a New Board

English | [中文](README.zh.md)

This directory provides only general documentation, blank formats and offline scripts; it does not ship real board photos, historical pixels, arm XY, pose anchors or success samples. A new model first reads the [marking task instructions](prompts/mark_board.md), then builds the result from the actual images of this session.

## Contents and dependencies

- `scripts/mark_board.py`: new-image binding, global/split fitting, holdout validation, annotated image/CSV/JSON output, and intersection queries.
- `prompts/mark_board.md`: complete operating requirements and deliverables checklist for the model taking over.
- `requirements.txt`: Python dependencies for standalone use; inside the chess package, use the Python environment that already has the dependencies installed.
- `runs/<this marking name>/`: created when used; holds this session's original image, control points, fit results and review records; no data is pre-populated.

The script does not capture images, connect to the robot or generate motions; images are obtained by the current workflow. It requires Python 3.10+, NumPy and OpenCV. For standalone use, choose any Python with the dependencies installed; there is no need to copy the robot SDK.

## Steps

Run the following commands from the `g1d_chess` root. Replace the placeholder paths with this session's files; `--max-error-px` must be set to a positive number based on the actual resolution, point-reading error and purpose. No threshold carried over from an old board is provided.

```bash
python scripts/mark_board.py init \
  --image /path/to/current_left_high.png \
  --output runs/board_position/new_board/controls.json
```

Initialization records the original image's absolute path, SHA256 and dimensions; `points` is empty and `orientation_confirmed=false`. View the original image and confirm that, from the robot's viewpoint, a–i run left to right and 0–9 run near to far; in a standard head image the near end is at the bottom and the far end at the top. After confirming, change that field to true, and add records based on the board grid lines actually visible:

```json
{
  "square": "a0",
  "pixel_xy": [null, null],
  "usage": "fit",
  "evidence": "visible_board_line_intersection",
  "note": "Describe the grid-line basis visible in this frame; null must be replaced with original-image pixels"
}
```

This is only a blank format and does not mean a0 is necessarily visible. `usage` is `fit` or `holdout`; holdout points must not take part in fitting. Write each intersection only once; occluded points are not written into the observation list and are inferred by the model. Each fitting region needs at least 6 fit points and 2 holdout points, distributed across both sides of the region, the near and far boundaries and the interior; when more real intersections can be read, increase the coverage.

First try the `global` whole-board homography model; if a systematic bias remains on the two sides of the river even after re-reading the control points, use `split`, with the near region 0–4 and the far region 5–9 each meeting the control-point requirements. The split model only describes image geometry; it does not represent a measured river width or a real 3D transform.

```bash
python scripts/mark_board.py build \
  --controls runs/board_position/new_board/controls.json \
  --mapping global --max-error-px <pixel_error_threshold_for_this_run> \
  --output-dir runs/board_position/new_board/result

python scripts/mark_board.py query d7 \
  --reference runs/board_position/new_board/result/board_points_pixels.json
```

The output directory must be a new directory to avoid overwriting old results. On failure, read the specific reason; when the error exceeds the threshold, diagnostics and the annotated image are still output, with return code 2 and status `needs_reannotation`. Passing the numerical checks only means `pending_visual_review`; you must still view the annotated image and check the lines region by region, and must not automatically claim the calibration is complete. After correcting the marks, rerun with a new output directory; do not hide errors by arbitrarily relaxing the threshold.

## Output and review

- `board_points_annotated.png`: keeps the original image resolution. Green marks fit observation points, blue marks holdout observation points, and yellow marks inferred points with no direct observation; dots are fitted positions and crosses are manually read points. Colors do not indicate piece occupancy.
- `board_points_pixels.json`: the 90 intersections, image hash, original image dimensions, model matrix, fit/holdout residuals and the pending-visual-review status. All `original_image_pixel_xy` values are board-surface coordinates in the original image.
- `board_points.csv`: square name, square coordinates, original-image pixels and evidence type.
- `controls.json`: a copy of the control points used in this fit, recording traceable input.
- `visual_review.md`: written separately by the model or a person; records the original image hash, orientation, corner/river/interior checks, grid lines near source/target, occlusion inferences and remaining uncertainties, and gives a pass or redo conclusion. The script does not replace visual acceptance.

After the board, camera, head/waist pose or cropping changes, mark the new image again. Even if the dimensions are the same, old pixels must not be reused directly. Board-surface intersections, piece top surfaces, wrist-image pixels and arm end-effector XY are different coordinates; this tool does not output a robot base transform, physical dimensions or motion compensation. Unknown physical dimensions stay null; no historical square spacing is assumed by default.

## Feeding back into a round

During planning, check the source/target square names against this session's annotation; source follows the piece's actual position and target follows the target board-surface intersection, each estimated independently with the executing hand, the pose and the current vision. Do not convert pixel differences directly into base millimetres, or apply the source correction to the target. Head-image marking does not provide wrist-image registration.

Prepare the material before execute; during execution, still follow the checkpoint procedure of the main [round_plan](../prompts/round_plan.md), and do not rebuild the whole board while holding a piece. The board position directory does not carry any old motion pre-checks, correction distances or attempt-count policies.
