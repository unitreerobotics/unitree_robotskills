#!/usr/bin/env bash
# Stop old dex1_internal services, then start one service.
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
ROOT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd -P)"
SERVICE_SCRIPT="$(readlink -f -- "${SCRIPT_DIR}/internal_service.py")"
PYTHON_BIN="${G1_PYTHON:-python}"
ENDPOINT="${CHESS_ENDPOINT:-ipc:///tmp/g1d_chess.sock}"
INTERFACE="${CHESS_INTERFACE:-}"
CAMERA_HOST="${CHESS_CAMERA_HOST:-}"
GRIPPER_OPEN_SIGN="${CHESS_GRIPPER_SIGN:-1}"
LOG_DIR=""
LOG_FILE="${ROOT_DIR}/runs/g1-robot-service.log"
KILL_WAIT="5"
START_TIMEOUT="15"
STATUS_ONLY=0
FOREGROUND=0
ACTIVE_ARM="right"

usage() {
    cat <<'EOF'
Usage: start_service.sh [options]

Kill every existing Python internal_service.py process, confirm none
remain, then start one detached service and confirm exactly one is running.

Options:
  --arm left|right        Select the actor entry before planning (default right).
  --status                 Check the matching process count; do not start.
  --foreground             Keep the launcher attached until the service exits.
  --python PATH            Python interpreter (default: G1_PYTHON or python on PATH).
  --endpoint ENDPOINT      Local ZMQ endpoint.
  --interface IFACE        DDS network interface (or CHESS_INTERFACE).
  --camera-host HOST       Camera server host (or CHESS_CAMERA_HOST).
  --gripper-open-sign N    Dex1 opening sign, -1 or 1 (default: CHESS_GRIPPER_SIGN or 1).
  --log-dir DIR            Observation log directory passed to the service.
  --log-file FILE          Launcher/service stderr log (default: runs/g1-robot-service.log).
  --kill-wait SEC          Seconds to confirm SIGKILL exit (default: 5).
  --start-timeout SEC      Seconds to confirm the new service (default: 15).
  -h, --help               Show this help.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --arm) ACTIVE_ARM="$2"; shift 2 ;;
        --status) STATUS_ONLY=1; shift ;;
        --foreground) FOREGROUND=1; shift ;;
        --python) PYTHON_BIN="$2"; shift 2 ;;
        --endpoint) ENDPOINT="$2"; shift 2 ;;
        --interface) INTERFACE="$2"; shift 2 ;;
        --camera-host) CAMERA_HOST="$2"; shift 2 ;;
        --gripper-open-sign) GRIPPER_OPEN_SIGN="$2"; shift 2 ;;
        --log-dir) LOG_DIR="$2"; shift 2 ;;
        --log-file) LOG_FILE="$2"; shift 2 ;;
        --kill-wait) KILL_WAIT="$2"; shift 2 ;;
        --start-timeout) START_TIMEOUT="$2"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
done

case "${ACTIVE_ARM}" in
    right) ;;
    left) SERVICE_SCRIPT="${SCRIPT_DIR}/internal_service_left.py" ;;
    *) echo "--arm must be left or right" >&2; exit 2 ;;
esac

if ! [[ "${KILL_WAIT}" =~ ^[0-9]+$ && "${START_TIMEOUT}" =~ ^[0-9]+$ ]]; then
    echo "--kill-wait and --start-timeout must be whole seconds" >&2
    exit 2
fi

# Match Python service arguments, not shells/grep merely mentioning the name.
service_pids() {
    ps -eo pid=,comm=,args= | awk '
        $2 ~ /^python/ {
            for (i = 4; i <= NF; i++) {
                if ($i == "-c") break
                if ($i ~ /(^|\/)internal_service(_left)?[.]py$/) {
                    print $1
                    break
                }
            }
        }'
}

read_service_pids() {
    local -n result=$1
    result=()
    mapfile -t result < <(service_pids)
}

kill_existing() {
    local -a pids=()
    local deadline
    read_service_pids pids
    if ((${#pids[@]} == 0)); then
        return 0
    fi

    echo "killing existing g1-robot-service PIDs: ${pids[*]}" >&2
    kill -KILL "${pids[@]}" 2>/dev/null || true
    deadline=$((SECONDS + KILL_WAIT))
    while (( SECONDS < deadline )); do
        read_service_pids pids
        ((${#pids[@]} == 0)) && return 0
        sleep 0.1
    done
    read_service_pids pids
    ((${#pids[@]} == 0)) && return 0
    echo "internal_service.py processes remain after SIGKILL: ${pids[*]}" >&2
    return 1
}

if (( STATUS_ONLY )); then
    pids=()
    read_service_pids pids
    printf 'g1-robot-service process count: %d' "${#pids[@]}"
    ((${#pids[@]} > 0)) && printf ' (PIDs: %s)' "${pids[*]}"
    printf '\n'
    ((${#pids[@]} == 1))
    exit $?
fi

if [[ -z "${INTERFACE}" || -z "${CAMERA_HOST}" ]]; then
    echo "Set CHESS_INTERFACE/CHESS_CAMERA_HOST or pass --interface/--camera-host before startup" >&2
    exit 2
fi
if [[ "${GRIPPER_OPEN_SIGN}" != "-1" && "${GRIPPER_OPEN_SIGN}" != "1" ]]; then
    echo "--gripper-open-sign must be -1 or 1" >&2
    exit 2
fi

kill_existing

mkdir -p -- "$(dirname -- "${LOG_FILE}")"
cmd=("${PYTHON_BIN}" -u "${SERVICE_SCRIPT}"
     --interface "${INTERFACE}"
     --endpoint "${ENDPOINT}"
     --camera-host "${CAMERA_HOST}"
     --gripper-open-sign "${GRIPPER_OPEN_SIGN}")
if [[ -n "${LOG_DIR}" ]]; then
    cmd+=(--log-dir "${LOG_DIR}")
fi

nohup "${cmd[@]}" >>"${LOG_FILE}" 2>&1 </dev/null &
child_pid=$!

deadline=$((SECONDS + START_TIMEOUT))
while (( SECONDS < deadline )); do
    pids=()
    read_service_pids pids
    if ((${#pids[@]} == 1)) && [[ "${pids[0]}" == "${child_pid}" ]]; then
        echo "g1-robot-service started: PID=${pids[0]} endpoint=${ENDPOINT} log=${LOG_FILE}"
        if (( FOREGROUND )); then
            wait "${child_pid}"
        fi
        exit 0
    fi
    if ! kill -0 "${child_pid}" 2>/dev/null && ((${#pids[@]} == 0)); then
        echo "g1-robot-service exited during startup; see ${LOG_FILE}" >&2
        tail -n 40 -- "${LOG_FILE}" >&2 || true
        exit 1
    fi
    sleep 0.1
done

read_service_pids pids
echo "startup did not produce exactly one g1-robot-service (count=${#pids[@]})" >&2
kill_existing || true
tail -n 40 -- "${LOG_FILE}" >&2 || true
exit 1
