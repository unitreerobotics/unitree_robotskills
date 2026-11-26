# Sourced by r1d and scripts/serve.sh: sets SKILL_ROOT's interpreter (SKILL_PYTHON) and PYTHONPATH.
# Interpreter order: R1D_SANDWICH_PYTHON (environment, then <package root>/.env),
# <package root>/.venv/bin/python, python3.
if [ -z "${R1D_SANDWICH_PYTHON:-}" ] && [ -f "$SKILL_ROOT/.env" ]; then
  # same rules as src/service/paths.py: optional `export`, optional quotes, last assignment wins
  R1D_SANDWICH_PYTHON=$(sed -n 's/^[[:space:]]*\(export[[:space:]]\+\)\?R1D_SANDWICH_PYTHON[[:space:]]*=[[:space:]]*//p' \
    "$SKILL_ROOT/.env" | tail -n 1 | sed -e 's/[[:space:]]*$//' -e 's/^"\(.*\)"$/\1/' -e "s/^'\(.*\)'\$/\1/")
fi
if [ -n "${R1D_SANDWICH_PYTHON:-}" ]; then
  SKILL_PYTHON=$R1D_SANDWICH_PYTHON
elif [ -x "$SKILL_ROOT/.venv/bin/python" ]; then
  SKILL_PYTHON=$SKILL_ROOT/.venv/bin/python
else
  SKILL_PYTHON=python3
fi
export PYTHONPATH="$SKILL_ROOT/src${PYTHONPATH:+:$PYTHONPATH}"
