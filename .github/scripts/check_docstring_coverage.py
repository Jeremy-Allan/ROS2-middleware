#!/usr/bin/env python3
"""Fail if any non-test kinova_interface Python file is missing a docstring.

Checks every module, class, and function/method under kinova_interface/'s
Python package, launch file, and scripts for a docstring. Test files
(test/) and setup.py are excluded, since they're covered by the test
suite's own naming/structure rather than this convention.

Usage:
    python3 .github/scripts/check_docstring_coverage.py [git-ref]

With no argument, checks the working tree. Exits 0 if every item has a
docstring, 1 otherwise (with the missing items listed).
"""
import ast
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]


def _should_check(path: str) -> bool:
    if not path.endswith('.py'):
        return False
    if not path.startswith('kinova_interface/'):
        return False
    if '/test/' in path:
        return False
    if path.endswith('setup.py'):
        return False
    return True


def _files_from_ref(ref: str) -> list[str]:
    out = subprocess.check_output(
        ['git', 'ls-tree', '-r', '--name-only', ref], cwd=REPO_ROOT, text=True
    )
    return [f for f in out.splitlines() if _should_check(f)]


def _files_from_working_tree() -> list[str]:
    files = []
    for sub in ('kinova_interface', 'kinova_interface/launch', 'kinova_interface/scripts'):
        base = REPO_ROOT / sub
        if not base.is_dir():
            continue
        for p in base.rglob('*.py'):
            rel = str(p.relative_to(REPO_ROOT))
            if _should_check(rel):
                files.append(rel)
    return sorted(set(files))


def _read(path: str, ref: str | None) -> str:
    if ref is None:
        return (REPO_ROOT / path).read_text()
    return subprocess.check_output(['git', 'show', f'{ref}:{path}'], cwd=REPO_ROOT, text=True)


def check(ref: str | None) -> int:
    files = _files_from_ref(ref) if ref else _files_from_working_tree()
    missing_total = 0
    for path in sorted(files):
        src = _read(path, ref)
        try:
            tree = ast.parse(src, filename=path)
        except SyntaxError as e:
            print(f"SYNTAX ERROR in {path}: {e}")
            missing_total += 1
            continue

        missing = []
        if not ast.get_docstring(tree):
            missing.append('<module>')
        for node in ast.walk(tree):
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
                if not ast.get_docstring(node):
                    missing.append(f'{node.name} (line {node.lineno})')

        if missing:
            missing_total += len(missing)
            print(f"{path}: missing docstrings for {', '.join(missing)}")

    if missing_total:
        print(f"\n{missing_total} item(s) missing a docstring.")
        return 1

    print("All checked files have full docstring coverage.")
    return 0


if __name__ == '__main__':
    ref = sys.argv[1] if len(sys.argv) > 1 else None
    sys.exit(check(ref))
