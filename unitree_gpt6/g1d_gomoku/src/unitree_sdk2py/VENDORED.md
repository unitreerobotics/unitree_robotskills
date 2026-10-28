# Unitree SDK2 Python

Source: https://github.com/unitreerobotics/unitree_sdk2_python

Upstream package version: `1.0.1`. Licensed under [BSD-3-Clause](LICENSE).
This SDK is bundled inside the Gomoku skill at `src/unitree_sdk2py/`.
Install dependencies from the repository root with `python -m pip install -r unitree_gpt6/g1d_gomoku/requirements.txt`. Use the bundled SDK directly from the checkout.
The SDK package, IDL messages and Linux x86_64 / aarch64 CRC libraries ship together.

Python imports use `unitree_gpt6.g1d_gomoku.src.unitree_sdk2py`.
No top-level SDK alias or external SDK installation is required.
The namespace adaptation updates generated IDL Python forward references;
DDS wire type names, relative imports, CRC binaries and license are unchanged.
Importing modules does not initialize DDS.

[manifest.json](manifest.json) records SHA-256 hashes of the bundled files and
retains the upstream snapshot hashes under `upstream_files` for comparison.
Upstream tests, Python caches and IDL generator metadata are omitted.
