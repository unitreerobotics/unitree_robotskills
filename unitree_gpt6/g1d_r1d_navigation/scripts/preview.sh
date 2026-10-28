#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3}"
cd "$REPO_ROOT"
"$PYTHON_BIN" -m unitree_gpt6.g1d_r1d_navigation.src.main --instruction "${1:-home}" --mode preview
