#!/usr/bin/env python3
"""Can the staleness banner still move the window? Both axes.

Reproduces the panel's geometry: a top bar of fixed power boxes, a stretch,
two buttons -- and the banner, in three arrangements. Measures the WINDOW's
minimum width AND height in each state, because that is what forced the
resize: Qt cannot draw a window smaller than its layout demands.

Also measures the width the label is actually allocated, because a warning
that fits in 63 px is not a warning, and swapping an oversized window for an
invisible warning would be a worse bug than the one being fixed.

Run headless: QT_QPA_PLATFORM=offscreen
"""
import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout,
                             QLabel, QPushButton, QSizePolicy)
from PyQt5.QtGui import QFontMetrics
from PyQt5.QtCore import Qt

LONG = ("⚠  power/state and motor/state STOPPED 21.1 s ago — "
        "the values below are frozen, not live")

TOP_BAR = "top bar, shown on demand"
OWN_ROW = "own row, always reserved"


def build(arrangement, ignored):
    w = QWidget()
    root = QVBoxLayout()
    top = QHBoxLayout()
    for _ in range(4):
        b = QLabel("47.9 V  0.77 A  36.8 W")
        b.setFixedWidth(160)
        top.addWidget(b)

    lbl = QLabel("")
    lbl.setStyleSheet("font-weight: bold;")
    if ignored:
        lbl.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        lbl.setMinimumWidth(0)
    lbl.setWordWrap(False)

    if arrangement == TOP_BAR:
        lbl.setVisible(False)
        top.addWidget(lbl, 1)
        top.addStretch(1)
    else:
        top.addStretch(1)

    for name in ("Stop Gait", "E-STOP"):
        btn = QPushButton(name)
        btn.setMinimumWidth(100)
        top.addWidget(btn)
    root.addLayout(top)

    if arrangement == OWN_ROW:
        fm = QFontMetrics(lbl.font())
        lbl.setFixedHeight(fm.height() + 4)
        lbl.setVisible(True)          # reserved, blank when healthy
        root.addWidget(lbl)

    body = QLabel("(middle area)")
    body.setMinimumHeight(300)
    root.addWidget(body)
    w.setLayout(root)
    return w, lbl, arrangement


def measure(title, arrangement, ignored):
    w, lbl, arr = build(arrangement, ignored)
    w.resize(1024, 768)
    w.show()
    app.processEvents()
    hw, hh = w.minimumSizeHint().width(), w.minimumSizeHint().height()

    if arr == TOP_BAR:
        lbl.setVisible(True)
    lbl.setText(LONG)
    w.layout().activate()
    app.processEvents()
    sw, sh = w.minimumSizeHint().width(), w.minimumSizeHint().height()
    got = lbl.width()

    print("  %-26s  width %4d->%4d (%+4d)   height %4d->%4d (%+3d)   "
          "label %4d px" % (title, hw, sw, sw - hw, hh, sh, sh - hh, got))
    return sw - hw, sh - hh, got


app = QApplication([])
print("\nwindow minimum size, healthy -> stale:\n")
dw_old, dh_old, g_old = measure("BEFORE", TOP_BAR, ignored=False)
dw_mid, dh_mid, g_mid = measure("Ignored, top bar", TOP_BAR, ignored=True)
dw_new, dh_new, g_new = measure("AFTER (own row)", OWN_ROW, ignored=True)
print()

fails = []
if dw_old <= 50:
    fails.append("the BEFORE case did not reproduce the growth (%+d px)" % dw_old)
else:
    print("reproduced: the original arrangement grew the window %+d px" % dw_old)

if g_mid >= 250:
    print("NOTE: the top-bar fix would also have been readable (%d px)" % g_mid)
else:
    print("confirmed: in the top bar it got only %d px -- unreadable, which "
          "is why it moved" % g_mid)

if dw_new != 0:
    fails.append("width still moves by %+d px" % dw_new)
if dh_new != 0:
    fails.append("height still moves by %+d px" % dh_new)
if g_new < 400:
    fails.append("label only %d px wide" % g_new)

print()
if fails:
    for f in fails:
        print("FAIL: " + f)
else:
    print("PASS: window minimum size unchanged in BOTH axes, and the banner "
          "gets %d px" % g_new)
raise SystemExit(1 if fails else 0)
