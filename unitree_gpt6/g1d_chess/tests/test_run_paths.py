import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

"""Run-root migration checks without importing the hardware service."""
import ast
from pathlib import Path
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


def load_path_guard(source, name, root):
    tree = ast.parse((ROOT / source).read_text())
    function = next(node for node in ast.walk(tree)
                    if isinstance(node, ast.FunctionDef) and node.name == name)
    function.decorator_list = []
    namespace = {'Path': Path, 'ROOT': root}
    exec(compile(ast.Module(body=[function], type_ignores=[]), source, 'exec'), namespace)
    return namespace[name]


class RunPathsTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name) / 'package'
        self.root.mkdir()
        self.guards = [
            load_path_guard('src/g1d_chess/chess/round_tools.py', 'within_run', self.root),
            load_path_guard('src/g1d_chess/robot/internal_service.py',
                            '_cached_archive_directory', self.root),
        ]

    def test_fresh_run_is_accepted_before_directories_exist(self):
        destination = self.root / 'runs' / 'new_game' / 'round_1'
        for guard in self.guards:
            self.assertEqual(guard(destination), destination)
        self.assertFalse((self.root / 'runs').exists())

    def test_retired_roots_and_external_paths_rejected(self):
        for destination in [*(self.root / name / 'round' for name in
                              ('observation', 'observations', 'evidence', 'runs_other')),
                            self.root / 'runs' / '..' / 'outside',
                            self.root.parent / 'external']:
            for guard in self.guards:
                with self.subTest(destination=destination, guard=guard.__name__):
                    with self.assertRaises(ValueError):
                        guard(destination)

    def test_symlink_cannot_escape_run_root(self):
        (self.root / 'runs').mkdir()
        (self.root / 'runs' / 'link').symlink_to(self.root.parent, target_is_directory=True)
        for guard in self.guards:
            with self.assertRaises(ValueError):
                guard(self.root / 'runs' / 'link' / 'round')
