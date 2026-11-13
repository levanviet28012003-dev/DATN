#!/usr/bin/env python3
"""Xuất lệnh GCC thực tế để VS Code tìm đúng header và kiểu dữ liệu ARM."""

import argparse
import json
from pathlib import Path
import shlex
import shutil


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler', required=True)
    parser.add_argument('--build-dir', required=True)
    parser.add_argument('--flags', required=True)
    parser.add_argument('sources', nargs='+')
    args = parser.parse_args()
    root = Path.cwd()
    build = Path(args.build_dir).resolve()
    compiler = shutil.which(args.compiler) or args.compiler
    commands = []
    for source in args.sources:
        output = build / Path(source).with_suffix('.o')
        commands.append({
            'directory': str(root),
            'file': str((root / source).resolve()),
            'arguments': [compiler, *shlex.split(args.flags), '-c', source, '-o', str(output)],
            'output': str(output),
        })
    build.mkdir(parents=True, exist_ok=True)
    destination = build / 'compile_commands.json'
    content = json.dumps(commands, indent=2) + '\n'
    if not destination.exists() or destination.read_text() != content:
        destination.write_text(content)


if __name__ == '__main__':
    main()
