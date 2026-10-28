# Configuration Templates and Field Reference

English | [中文](README.zh.md)

This directory does not store the packaging machine's user name, absolute paths, network interface, device IPs, version snapshots, measured poses or historical error groups. For dependency installation see the chess group in the repository-root pyproject.toml and the [environment setup guide](../docs/environment_setup.md); for a first deployment, then follow [new G1D first-game setup](../docs/first_game_setup.md) to collect on-site information and fill in the fields you need.

Each JSON's `_field_help` gives descriptions and examples; **examples only demonstrate the format**. `192.0.2.10` is a documentation-only address; `/opt/…` is a fictitious installation path. Do not copy these values as the actual configuration. Unknown device parameters are `null`; do not substitute 0. Confirmed policies in the operating procedure, such as attempt counts, the 5mm lowering rule and "no distance limit", are still retained.

`gripper_open_sign=1` is the default direction shared by the left and right built-in Dex1 grippers: a positive angle opens, 0 closes, and it carries over unchanged when migrating to an ordinary environment. Both environment JSON files keep this default; they are records only. The launcher and the service independently default to +1, and support environment-variable and CLI overrides. This field does not need to be treated as a missing measurement at a new site.

| File | What it stores | How the current code uses it |
| --- | --- | --- |
| `environment.example.json` | Environment field descriptions and a blank template | Not read automatically; explicitly set environment variables/CLI according to what is filled in |
| `package_environment.json` | Deployment-environment record entry in the same format | Not read automatically; actual information may be filled in on site; it is not a dynamic state snapshot |
| `python_packages.json` | Target Python and dependency versions to be filled in | Does not install automatically or pin versions; use check_environment to verify the actual interpreter |
| `chess_camera_policy.json` | Logical key of the planning head image, the wrist-stream mapping pending confirmation, and its basis | Read by the camera_policy module with per-process caching; refuses to produce a camera selection while the left/right mapping is null |
| `manipulation_profile.json` | General procedure, policies, and pose/tool parameters to be filled in | Descriptive file; does not configure the planner or service automatically; actual parameters must also be synced into this round's spec/request |
| `init_state.json` | Measured joints of both arms, model FK, gripper and source records; empty at release | round_tools reads the init XYZ/RPY for the physical executing hand; a complete plan cannot be generated until it is filled in |
| `board_parameters.json` | Blank measurement record of board and piece dimensions | Descriptive record; not applied automatically to the planner or service |
| `scene.example.json` | Surface/tool-envelope/clearance parameters to be filled in | Not loaded automatically; generate a request containing only the `scene` key, then call set_scene |
| `xy_error_learning.json` | Compatible-environment context_id to be filled in | Read when no explicit spec/CLI context is given; null raises an error, and the old group is not reused by default |

For scene values, prefer data supplied by the user that is compatible with the current coordinate frame and tool. When the user supplies no on-site information, the model estimates all scene fields itself; when some are supplied, the model fills in the remaining fields. Use available images, robot state, tool models, dimension data and reasonable assumptions, and record sources, methods and uncertainty; do not make additional user material or extra confirmation a precondition. For the specific method and post-setting verification see [first setup section 5](../docs/first_game_setup.md#5-configure-on-site-geometry-both-arms-init-and-motion-parameters). The model may proactively collect available material within the task scope and complete the estimate after recording assumptions and error ranges; estimated values and adopted defaults must be labelled with their source and must not be called measured. Evaluation results and requests are saved in this run; the general templates stay blank.

`null` has two clearly distinguished uses: in on-site parameter fields it means not yet filled in; in `max_high_offset_m`, `max_target_offset_m` and `policy.xy_correction_distance_limit_m` it means the existing policy sets no distance limit. Do not handle them with a blanket "fill every null with a number" rule.

The environment field `interface` records the target machine's DDS network interface. `python_executable` records the interpreter path of the user's environment; at runtime, activate that environment or call this interpreter explicitly; the configuration JSON does not switch it automatically. Both environment JSON files use the same template structure; the actual deployment follows the record you fill in, not an inference from the file name that it is in effect.

On-site information may be written to a local copy or to this run's records. If the local copy is at another path, the existing camera policy and error tools will not look for it automatically; it must be explicitly synced to the actually read files listed above, or passed through the tools' existing explicit context parameter.
