#!/usr/bin/env python3
"""Normalize auto-generated sequence JSON into canonical corgi_panel format.

Usage:
  python3 normalize_sequence_json.py input.json [output.json]
"""
from __future__ import annotations

import os
import sys


def main() -> int:
    if len(sys.argv) < 2:
        print('Usage: python3 normalize_sequence_json.py input.json [output.json]')
        return 2

    in_path = sys.argv[1]
    if len(sys.argv) >= 3:
        out_path = sys.argv[2]
    else:
        base, ext = os.path.splitext(in_path)
        out_path = f'{base}.normalized{ext or ".json"}'

    # Local import so script can run from package root without install.
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    if repo_root not in sys.path:
        sys.path.insert(0, repo_root)

    from corgi_ui.core.sequence_model import load_sequence, save_sequence

    seq = load_sequence(in_path)
    save_sequence(out_path, seq)

    print(f'Input : {in_path}')
    print(f'Output: {out_path}')
    print(f'Nodes : {len(seq.nodes)}')
    print('Normalize done.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
