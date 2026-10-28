#!/usr/bin/env python3
"""Verify published files, skill links and source imports without hardware imports."""
import ast
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[1]
RETIRED = {'vendor', 'skills', 'calibration', 'calibration_inputs', 'observation', 'observations', 'evidence', 'reference'}


def source_files():
    excluded = {'.git', '.venv', '__pycache__', '.pytest_cache', 'runs'}
    local_configs = {'config/cam_config_client.yaml', 'config/cam_config_server.yaml'}
    for path in sorted(ROOT.rglob('*')):
        relative = path.relative_to(ROOT)
        if any(part in excluded for part in relative.parts):
            continue
        if path.name == 'manifest.json' or str(relative) in local_configs:
            continue
        if path.name == '.env' or (path.name.startswith('.env.') and path.name != '.env.example'):
            continue
        if path.is_file() or path.is_symlink():
            yield {'path': str(relative)}


def verify():
    errors = []
    count = 0
    python_count = 0
    manifest_path = ROOT / 'manifest.json'
    has_manifest = manifest_path.is_file()
    entries = json.loads(manifest_path.read_text())['files'] if has_manifest else source_files()
    for item in entries:
        relative = Path(item['path'])
        path = ROOT / relative
        if relative.is_absolute() or '..' in relative.parts or path.is_symlink() or not path.is_file():
            errors.append(str(relative) + ': invalid/missing file')
            continue
        raw = path.read_bytes()
        count += 1
        if has_manifest and (len(raw) != item['size_bytes'] or hashlib.sha256(raw).hexdigest() != item['sha256']):
            errors.append(str(relative) + ': hash/size mismatch')
        if relative.parts[0] in RETIRED:
            errors.append(str(relative) + ': retired directory')
        try:
            if path.suffix == '.json':
                json.loads(raw)
            if path.suffix == '.py':
                ast.parse(raw, filename=str(relative))
                python_count += 1
            if path.suffix == '.sh':
                env = {k:v for k,v in os.environ.items() if k not in {'BASH_ENV', 'ENV'}}
                subprocess.run(['bash', '--noprofile', '--norc', '-n', str(path)],
                               check=True, capture_output=True, env=env)
            if path.suffix == '.md':
                text = raw.decode()
                if text.count('```') % 2:
                    errors.append(str(relative) + ': unbalanced code fence')
                if re.search(r'/(?:home|Users)/|~/(?:codes|\.codex)/', text):
                    errors.append(str(relative) + ': machine-specific path')
                if re.search(r'(?<![\w.])(?:observation|observations|evidence|reference)/', text):
                    errors.append(str(relative) + ': retired directory reference')
                for url in re.findall(r'\]\(([^)]+)\)', text):
                    target = url.split('#')[0]
                    if not target or target.startswith(('https:', 'http:', 'mailto:')):
                        continue
                    local = (path.parent / target).resolve()
                    if not local.is_relative_to(ROOT) or not local.exists():
                        errors.append(str(relative) + ': broken/external link ' + url)
        except Exception as exc:
            errors.append(str(relative) + ': ' + str(exc))
    index = json.loads((ROOT / 'docs/round_script_inventory.json').read_text())
    declared = [name for paths in index['groups'].values() for name in paths]
    if len(declared) != len(set(declared)) or len(declared) != index['verification']['local_script_files']:
        errors.append('inventory count/uniqueness mismatch')
    for name in declared:
        path = (ROOT / name).resolve()
        if not path.is_relative_to(ROOT) or not path.is_file():
            errors.append('inventory path missing/external: ' + name)
    for source, dependencies in index['local_import_edges'].items():
        for name in [source, *dependencies]:
            if name not in declared:
                errors.append('undeclared local import: ' + name)
    for name in RETIRED:
        if (ROOT / name).exists():
            errors.append('retired root directory: ' + name)
    skill = (ROOT / 'SKILL.md').read_text()
    if not skill.startswith('---\n') or not re.search(r'^name: g1d-chess$', skill, re.M) or not re.search(r'^description: .+', skill, re.M):
        errors.append('skill frontmatter missing or invalid')
    return {'status': 'failed' if errors else 'passed', 'files_verified': count,
            'hashes_verified': has_manifest,
            'python_syntax_files': python_count, 'robot_calls': False, 'errors': errors}


if __name__ == '__main__':
    result = verify()
    print(json.dumps(result, ensure_ascii=False, indent=2))
    raise SystemExit(bool(result['errors']))
