#!/usr/bin/env bash
# start.sh -- one-command launcher for Unitree Navi Agent (DeepSeek-Harness host + layered patches)
#
# Usage:
#   scripts/start.sh                   # OpenAI GPT model + stub robot (offline, default)
#   scripts/start.sh bailian           # Alibaba Cloud Bailian model + stub
#   scripts/start.sh deepseek          # DeepSeek model + stub
#   scripts/start.sh real              # GPT model + real robot [robot and image_server online, operator on site]
#   scripts/start.sh g1d real          # Use the G1D prompt (default r1d; the sidecar auto-detects the robot model; keep the persona consistent with it)
#
# Options:
#   --dry-run          Print the commands without launching
#   --headless "msg"   One-shot headless run instead (for automated verification)
#
# Environment variables:
#   DSH_DIR        Path to the DeepSeek-Harness repo (required unless at <skill dir>/deepseek-harness)
#   NAVI_PYTHON    Python for the MCP sidecar (with the root navigation extra installed; default <skill dir>/.venv/bin/python, else python3)
#   OPENAI_API_KEY / DASHSCOPE_API_KEY / DEEPSEEK_API_KEY   Key for the chosen provider (prompted at launch if unset)
#   NAVI_LOG=0           Do not write the process output log (default <skill dir>/logs/navi-<timestamp>.log)
#   NAVI_RUN_LOG         Run log path (per-tool-call argument/result summaries, shown below the camera page view;
#                        default <skill dir>/logs/run-<timestamp>.jsonl, =0 shows it on the page only, never saved)
#   NAVI_CAMERA_VIEW=0   In real mode, do not start the browser camera view (default serves :3081)
#   NAVI_OBSERVER_PORT=0 Disable the observation-push plugin
#
# Local environment: <skill dir>/navi.env is loaded first if present (not committed). Python / Node installs differ
#   per machine; put PATH, DSH_DIR and NAVI_PYTHON there instead of exporting each time. Example (conda env with node/pnpm):
#     export PATH="$HOME/miniforge3/envs/navi/bin:$PATH"
#     export DSH_DIR="$HOME/deepseek-harness"
#     export NAVI_PYTHON="$HOME/miniforge3/envs/navi/bin/python"
#   Do not put keys there (they are still entered at launch).
#
# Session isolation: DSH_HOME points to <skill dir>/.dsh-home, so robot sessions stay out of the global ~/.dsh.

set -euo pipefail

PROVIDER="gpt"
ROBOT="stub"
MODEL="r1d"
DRY_RUN=0
HEADLESS_MSG=""
while [ $# -gt 0 ]; do
    case "$1" in
        bailian|deepseek|gpt) PROVIDER="$1"; shift ;;
        stub|real)            ROBOT="$1"; shift ;;
        g1d|r1d)              MODEL="$1"; shift ;;
        --dry-run)            DRY_RUN=1; shift ;;
        --headless)           [ $# -ge 2 ] || { echo "--headless requires a message" >&2; exit 2; }
                              HEADLESS_MSG="$2"; shift 2 ;;
        -h|--help)            sed -n '2,32p' "$0"; exit 0 ;;
        *) echo "Unknown argument: $1 (valid: gpt|bailian|deepseek, stub|real, g1d|r1d, --dry-run, --headless \"msg\")" >&2; exit 2 ;;
    esac
done

ROOT="$(cd "$(dirname "$0")/.." && pwd)"   # skill dir unitree_gpt6/g1d_r1d_navigation
SRC="$ROOT/src/navi_agent"
HOST="$ROOT/host"
# Auto-export navi.env variables (NAVI_OBSERVER_PORT etc. must reach the plugins/sidecar spawned by DSH)
# shellcheck disable=SC1091
if [ -f "$ROOT/navi.env" ]; then set -a; . "$ROOT/navi.env"; set +a; fi
PY="${NAVI_PYTHON:-}"
[ -z "$PY" ] && { [ -x "$ROOT/.venv/bin/python" ] && PY="$ROOT/.venv/bin/python" || PY=python3; }
DSH_DIR="${DSH_DIR:-$ROOT/deepseek-harness}"

if [ ! -f "$DSH_DIR/package.json" ]; then
    echo "DeepSeek-Harness not found: $DSH_DIR" >&2
    echo "  Clone it and run pnpm install && pnpm run build, then export DSH_DIR=<path> (see README \"Quick start\")" >&2
    exit 2
fi
command -v "$PY" >/dev/null 2>&1 || { echo "Python not found: $PY (set NAVI_PYTHON)" >&2; exit 2; }
PY="$(command -v "$PY")"
case "$PY" in /*) ;; *) PY="$PWD/$PY" ;; esac   # Make absolute: we cd to DSH_DIR later and generated patches need absolute paths
# If pnpm is not on PATH, look next to Python (conda envs often install node/pnpm in the same bin);
# fail before launching anything rather than at the last step after the camera view is already up
if ! command -v pnpm >/dev/null 2>&1; then
    for d in "$(dirname "$PY")" "$HOME/.local/node/bin"; do
        [ -x "$d/pnpm" ] && { export PATH="$d:$PATH"; break; }
    done
fi
if ! command -v pnpm >/dev/null 2>&1 && [ "$DRY_RUN" != "1" ]; then
    echo "pnpm not found: add the Node/pnpm directory to PATH (can go in $ROOT/navi.env)" >&2
    exit 2
fi

export DSH_HOME="$ROOT/.dsh-home"
mkdir -p "$DSH_HOME"

# provider -> patch; keys are read from the environment only
case "$PROVIDER" in
    bailian)  PROVIDER_PATCH="$HOST/navi-bailian.patch.yaml";  KEY_VAR=DASHSCOPE_API_KEY ;;
    deepseek) PROVIDER_PATCH="$HOST/navi-deepseek.patch.yaml"; KEY_VAR=DEEPSEEK_API_KEY ;;
    gpt)      PROVIDER_PATCH="$HOST/navi-gpt.patch.yaml";      KEY_VAR=OPENAI_API_KEY ;;
esac
# Key not exported: prompt for it (no echo; lives only in this process environment, never written to disk)
if [ -z "${!KEY_VAR:-}" ] && [ "$DRY_RUN" != "1" ]; then
    if [ -t 0 ]; then
        read -rsp "Enter $KEY_VAR (input hidden): " KEY_INPUT; echo >&2
        [ -n "$KEY_INPUT" ] || { echo "No key entered, exiting" >&2; exit 2; }
        export "$KEY_VAR=$KEY_INPUT"; unset KEY_INPUT
    else
        echo "Missing API key: export $KEY_VAR=... first" >&2
        exit 2
    fi
fi

case "$ROBOT" in
    stub) SERVER="$SRC/tools/stub_mcp_server.py" ;;
    real) SERVER="$SRC/navi_mcp_server.py"
          [ -f "$SRC/config.yaml" ] || echo "Note: src/navi_agent/config.yaml not found; using built-in safe defaults (map navigation disabled)." >&2
          echo "⚠️  Real-robot mode: make sure the robot and image_server are online, an operator is on site, and the e-stop is within reach." >&2 ;;
esac

# Generate the two patches that depend on local absolute paths (not committed)
GEN="$HOST/generated"
mkdir -p "$GEN"
ROBOT_PATCH="$GEN/robot.patch.yaml"
cat > "$ROBOT_PATCH" <<EOF
# Generated by start.sh; do not edit. Mounts the MCP sidecar into DSH as a stdio child; tools are named mcp__robot__*.
- insert:
    - id: mcp-robot
      name: '@deepseek-ai/dsh-mcp-client'
      config:
        serverName: robot
        transport: stdio
        command: '$PY'
        args:
          - '$SERVER'
        toolCallTimeoutMs: 60000
EOF
# One persona per robot model: host/navi-persona-<g1d|r1d>.txt, injected into a different patch per mode.
# awk indents non-empty lines and leaves empty lines empty, i.e. embeds the file as a YAML literal block (|-).
PERSONA_FILE="$HOST/navi-persona-$MODEL.txt"
OBSERVER_PATCH="$HOST/navi-observer.patch.yaml"
for f in "$PROVIDER_PATCH" "$PERSONA_FILE" "$OBSERVER_PATCH"; do
    [ -f "$f" ] || { echo "Missing file: $f" >&2; exit 2; }
done
inject_persona() { awk -v ind="$1" '{print (length($0)?ind $0:"")}' "$PERSONA_FILE"; }

# web profile: declare the navi agent preset (persona + compaction chain) and make navi the registry default.
# This DSH version has no directory/roots-style presets: a preset is an inline @deepseek-ai/dsh-agent-preset row,
# and the default lives in agent-preset-registry.config.default (replaced wholesale; setting default suffices).
# Robot tools come from the host-layer mcp-robot row and are visible to preset-built sessions, so the preset does
# not mount them again. Coding tools (bash/fs/subagents/plan/todo/ask_user) are disabled at the web host layer and
# only provided by presets, so they are omitted here.
# Exception: task goals, tool-goal (create_goal/update_goal) + command-goal (/goal). The host-layer goal service
# and goal-round-driver always run; once the model creates a goal, the driver starts another round whenever it
# stops before finishing (otherwise the model often halts mid-task saying it is unfinished and waits for "continue").
# The compaction group must have an isolate realm, otherwise its services leak into the global realm and mounting is rejected.
WEB_PRESET_PATCH="$GEN/navi-preset.patch.yaml"
gen_web_preset() {
cat > "$WEB_PRESET_PATCH" <<EOF
# Generated by start.sh from host/navi-persona-$MODEL.txt; do not edit.
- insert:
    - id: preset-navi
      name: '@deepseek-ai/dsh-agent-preset'
      config:
        id: navi
        name: Navi 机器人模式
        description: Unitree Navi Agent——persona、压缩链与任务目标（自动续行）；机器人工具由宿主层提供，不含编码工具。
        order: 0
        plugins:
          - id: persona
            name: '@deepseek-ai/dsh-persona'
            config:
              prefix: |-
$(inject_persona '                ')
          - id: tool-goal
            name: '@deepseek-ai/dsh-tool-goal'
          - id: command-goal
            name: '@deepseek-ai/dsh-command-goal'
          - id: compaction
            name: cordis:group
            group: true
            isolate:
              compaction: true
              toolResultPruner: true
            config:
              - id: compaction-basic
                name: '@deepseek-ai/dsh-compaction-basic'
                config:
                  auto: true
                  thresholdRatio: 0.8
                  retainRatio: 0.16
                  maxTokens: 8192
                  compactionRetries: 1
                  maxOverflowRetries: 1
              - id: command-compact
                name: '@deepseek-ai/dsh-command-compact'
              - id: tool-result-pruner
                name: '@deepseek-ai/dsh-compaction-tool-result-pruner'
                config:
                  thresholdChars: 8192
                  headChars: 4096
                  tailChars: 1024
- id: agent-preset-registry
  name: '@deepseek-ai/dsh-agent-preset-registry'
  config:
    default: navi
EOF
}

# headless profile: no preset mechanism (adding a registry trips the "registry without session binding" invariant).
# The persona goes straight into deployment-level system-prompt.personaPrefix (replacing it wholesale drops the
# bundle's coding persona and cwd suffix).
HEADLESS_PATCH="$GEN/navi-headless.patch.yaml"
gen_headless_patch() {
cat > "$HEADLESS_PATCH" <<EOF
# Generated by start.sh from host/navi-persona-$MODEL.txt; do not edit.
- id: system-prompt
  name: '@deepseek-ai/dsh-system-prompt'
  config:
    personaPrefix: |-
$(inject_persona '      ')
- id: compaction-basic
  name: '@deepseek-ai/dsh-compaction-basic'
  config:
    auto: true
    thresholdRatio: 0.8
    retainRatio: 0.16
    maxTokens: 8192
    compactionRetries: 1
    maxOverflowRetries: 1
- id: tool-result-pruner
  name: '@deepseek-ai/dsh-compaction-tool-result-pruner'
  config:
    thresholdChars: 8192
    headChars: 4096
    tailChars: 1024
EOF
}

# One timestamp per run, pairing process output navi-<TS>.log with run log run-<TS>.jsonl
TS="$(date +%Y%m%d-%H%M%S)"
# Run log: the MCP sidecar records per-tool-call argument/result summaries (shown below the camera view);
# the sidecar is spawned by DSH and inherits this variable
export NAVI_RUN_LOG="${NAVI_RUN_LOG:-$ROOT/logs/run-$TS.jsonl}"
[ "$NAVI_RUN_LOG" != "0" ] && [ "$DRY_RUN" != "1" ] && mkdir -p "$(dirname "$NAVI_RUN_LOG")"

# Real mode: start the browser camera view (separate process, read-only subscription, independent of the MCP path)
CAMERA_VIEW_PID=""
if [ "$ROBOT" = "real" ] && [ "$DRY_RUN" != "1" ] && [ "${NAVI_CAMERA_VIEW:-1}" != "0" ]; then
    "$PY" "$SRC/tools/camera_view.py" --port 3081 &
    CAMERA_VIEW_PID=$!
    trap '[ -n "$CAMERA_VIEW_PID" ] && kill "$CAMERA_VIEW_PID" 2>/dev/null || true' EXIT INT TERM
    echo "→ Camera view: http://127.0.0.1:3081/"
fi

cd "$DSH_DIR"
echo "→ Navi launch: provider=${PROVIDER}, robot=${ROBOT}, persona=${MODEL}"
[ "$NAVI_RUN_LOG" != "0" ] && echo "→ Run log: $NAVI_RUN_LOG"

# Process output log: stdout/stderr of the DSH host and MCP sidecar (sidecar model/camera switch logs appear only here)
LOG="$ROOT/logs/navi-$TS.log"
if [ "${NAVI_LOG:-1}" = "0" ]; then
    run() { "$@"; }
else
    mkdir -p "$ROOT/logs"
    echo "→ Process output: $LOG"
    run() { "$@" 2>&1 | tee -a "$LOG"; }     # pipefail is on: the exit code is still dsh's
fi

# Argument order: --patch is a launcher-global option and must precede app arguments (message / --no-open).
# verify-deps-before-run=false: deps are installed; stop pnpm from re-running install (lefthook postinstall fails on old git).
# Layer order: bundle -> profile -> --patch (in the order given); the mode patch goes last so it can override the registry default.
PATCHES=(--patch "$PROVIDER_PATCH" --patch "$ROBOT_PATCH" --patch "$OBSERVER_PATCH")
if [ -n "$HEADLESS_MSG" ]; then
    gen_headless_patch
    CMD=(pnpm --config.verify-deps-before-run=false dsh --profile headless "${PATCHES[@]}" --patch "$HEADLESS_PATCH")
    echo "  ${CMD[*]} \"${HEADLESS_MSG}\""
    [ "$DRY_RUN" = "1" ] && exit 0
    run "${CMD[@]}" "$HEADLESS_MSG"
else
    gen_web_preset
    CMD=(pnpm --config.verify-deps-before-run=false dsh --profile web "${PATCHES[@]}" --patch "$WEB_PRESET_PATCH" --no-open)
    echo "  ${CMD[*]}"
    echo "  Once started, open the tokenized URL (:3080) from the log and pick an image-capable model in the UI (new sessions use the navi preset)."
    [ "$DRY_RUN" = "1" ] && exit 0
    run "${CMD[@]}"
fi
