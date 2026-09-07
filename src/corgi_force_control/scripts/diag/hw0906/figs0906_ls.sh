#!/usr/bin/env bash
D=/home/alexc/corgi_runs/hw_2026-09-06
echo "== bags"
ls "$D/bags" | grep -E '^s2_(l0r|ol10|ol15|l0_a|l10_a|l15)'
echo "== vicon"
ls "$D/vicon" | grep -E '^(L0|OL10|OL15|L10|L15)'
echo "== python"
which python3; python3 -c "import numpy, matplotlib, c3d; print(numpy.__version__, matplotlib.__version__)"
