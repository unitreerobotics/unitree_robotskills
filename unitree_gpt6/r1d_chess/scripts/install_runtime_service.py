#!/usr/bin/env python3
"""Create the systemd user unit r1d-runtime.service that runs the bundled robot runtime (third_party/robot_runtime) from this package.

  python scripts/install_runtime_service.py                 print the unit, write nothing
  python scripts/install_runtime_service.py --write         write ~/.config/systemd/user/r1d-runtime.service
  python scripts/install_runtime_service.py --write --python /path/to/env/bin/python

The Runtime is the only DDS writer; the chess service talks to it over local IPC. It reads
config/runtime.json (runtime_root, experiment, network_interface, camera_host, joint_gains), so the
waist/head gains installed by setup/arm-calibration load when this unit is restarted.
An existing unit is never replaced unless --force is given. Nothing is started.
"""
import argparse
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
UNIT = 'r1d-runtime.service'


def unit_text(python):
    return f'''[Unit]
Description=R1-D robot runtime (local DDS and IPC)
After=network-online.target

[Service]
Type=simple
WorkingDirectory={ROOT}
Environment=PYTHONDONTWRITEBYTECODE=1
ExecStart={python} -B {ROOT/'scripts/r1d_runtime.py'} serve
KillSignal=SIGINT
TimeoutStopSec=10
Restart=no

[Install]
WantedBy=default.target
'''


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--python', default=sys.executable,
                   help='interpreter with the Runtime dependencies (default: the one running this script)')
    p.add_argument('--write', action='store_true')
    p.add_argument('--force', action='store_true', help='replace an existing unit')
    a = p.parse_args()
    text = unit_text(a.python)
    target = Path.home()/'.config/systemd/user'/UNIT
    if not a.write:
        print(text)
        print(f'# dry run; --write would create {target}' + (' (exists: needs --force)' if target.exists() else ''))
        return
    if target.exists() and not a.force:
        raise SystemExit(f'{target} already exists; compare it with the text above and use --force to replace it')
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(text)
    subprocess.run(['systemctl', '--user', 'daemon-reload'], check=False)
    print(f'written {target}\nstart with: systemctl --user start {UNIT}')


if __name__ == '__main__':
    main()
