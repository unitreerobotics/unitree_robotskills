#!/usr/bin/env bash
# Start the R1-D sandwich server (simulation, shadow or real) with the skill's interpreter.
set -euo pipefail

SKILL_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
# shellcheck source=_env.sh
source "$SKILL_ROOT/scripts/_env.sh"

# The robot LAN must never go through the desktop's proxy.
unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY all_proxy

exec "$SKILL_PYTHON" -B -u -m service.server "$@"
