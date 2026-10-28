"""Use only a physically reviewed job and deployment configuration."""

import argparse

from unitree_gpt6.g1d_draw.src import Executor, load_config, load_job
from unitree_gpt6.g1d_draw.src.cli import serve
from unitree_gpt6.g1d_draw.src.g1 import G1DDSBackend
from unitree_gpt6.g1d_draw.src.kinematics import PinocchioKinematics


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--job", required=True)
    args = parser.parse_args(argv)
    config = load_config(args.config)
    executor = Executor(config, G1DDSBackend(config), PinocchioKinematics(config))
    # Reuse the CLI lifecycle: retain hold until an explicit release request.
    return serve(executor, load_job(args.job))


if __name__ == "__main__":
    raise SystemExit(main())
