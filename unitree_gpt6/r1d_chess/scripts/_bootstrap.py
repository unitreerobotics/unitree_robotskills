"""Run existing src modules from a source checkout, without pip-installing the repo."""
from pathlib import Path
import runpy
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))

def run(module):
    runpy.run_module(module, run_name='__main__')
