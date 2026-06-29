#!/usr/bin/env python3
"""
Custom Command Sequence Window.

Non-modal window for creating / editing multi-node position sequences and
triggering their execution via the main CorgiControlPanel executor.
"""
from __future__ import annotations

import os
import csv
from typing import Callable

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QLabel, QLineEdit, QDoubleSpinBox,
    QGroupBox, QListWidget, QListWidgetItem, QTextEdit,
    QProgressBar, QSplitter, QFrame, QComboBox, QFileDialog,
    QMessageBox, QSizePolicy,
    QScrollArea,
)
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QColor

from corgi_ui.core.constants import LOGLEVEL, COLORS, PATHS, format_log_html
from corgi_ui.core.sequence_model import (
    ExecutionState, JointTarget, SequenceNode, Sequence, LimitProfile,
    LEGS, JOINTS, BUILTIN_TEMPLATES,
    save_sequence, load_sequence,
    save_limit_profile, load_limit_profile,
)

from datetime import datetime


def _default_limit_profile() -> LimitProfile:
    profile = LimitProfile(profile_name='default_template')
    for leg in LEGS:
        profile.limits[(leg, 'theta')].min_deg = 17.0
        profile.limits[(leg, 'theta')].max_deg = 120.0
        profile.limits[(leg, 'theta')].max_speed_deg_per_sec = 300.0
        profile.limits[(leg, 'beta')].min_deg = -90.0
        profile.limits[(leg, 'beta')].max_deg = 90.0
        profile.limits[(leg, 'beta')].max_speed_deg_per_sec = 300.0
        profile.limits[(leg, 'gamma')].min_deg = -45.0
        profile.limits[(leg, 'gamma')].max_deg = 45.0
        profile.limits[(leg, 'gamma')].max_speed_deg_per_sec = 300.0
    return profile


class CustomSequenceWindow(QWidget):
    """
    Standalone (non-modal) window for editing and running custom position
    command sequences.

    Signals
    -------
    run_requested(sequence, limit_profile, dry_run)
        Emitted when the user clicks Run or Dry-run.  The panel connects
        this to its executor.
    stop_requested()
        Emitted when the user clicks Stop.
    """

    run_requested = pyqtSignal(object, object, bool)   # Sequence, LimitProfile|None, dry_run
    stop_requested = pyqtSignal()

    def __init__(
        self,
        get_current_pose_fn: Callable[[], dict | None],
        output_dir: str = PATHS.DEFAULT_OUTPUT_DIR,
        parent: QWidget | None = None,
    ):
        """
        Parameters
        ----------
        get_current_pose_fn:
            Called when "Grab Current Pose" is clicked.
            Must return dict[leg][joint] = float (degrees) or None.
        output_dir:
            Base directory for auto-saving execution records.
        """
        super().__init__(parent, Qt.Window)

        self._get_current_pose = get_current_pose_fn
        self._output_dir = output_dir

        # Current working sequence
        self._sequence = Sequence(name="new_sequence")
        # Currently selected node index (-1 = none)
        self._selected_node_idx: int = -1
        # Currently loaded limit profile
        self._limit_profile: LimitProfile | None = _default_limit_profile()
        # Whether execution is in progress (locked state)
        self._exec_locked = False
        self._template_file = os.path.join(PATHS.DEFAULT_OUTPUT_DIR, 'sequence_templates.json')
        self._custom_templates: dict[str, dict] = {}
        self._load_custom_templates()

        self._init_ui()
        self.lbl_limit_status.setText(f"✔ {self._limit_profile.profile_name} (editable template)")
        self.lbl_limit_status.setStyleSheet("color: #00e676; font-size: 11px;")
        self._refresh_node_list()

    # =========================================================================
    # UI construction
    # =========================================================================

    def _init_ui(self) -> None:
        self.setWindowTitle("Custom Command Sequence")
        self.setMinimumSize(900, 680)
        self.resize(1100, 760)

        root = QVBoxLayout(self)
        root.setSpacing(8)
        root.setContentsMargins(10, 10, 10, 10)

        root.addLayout(self._build_header_bar())

        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(self._build_node_list_panel())
        splitter.addWidget(self._build_node_editor_panel())
        splitter.setSizes([280, 620])
        root.addWidget(splitter, stretch=1)

        root.addLayout(self._build_run_bar())
        root.addWidget(self._build_log_panel())

    # ---- header bar --------------------------------------------------------

    def _build_header_bar(self) -> QHBoxLayout:
        bar = QHBoxLayout()
        bar.setSpacing(8)

        bar.addWidget(QLabel("Name:"))
        self.edit_seq_name = QLineEdit(self._sequence.name)
        self.edit_seq_name.setMaximumWidth(200)
        self.edit_seq_name.textChanged.connect(self._on_seq_name_changed)
        bar.addWidget(self.edit_seq_name)

        bar.addWidget(QLabel("Notes:"))
        self.edit_seq_notes = QLineEdit(self._sequence.notes)
        self.edit_seq_notes.setPlaceholderText("(optional)")
        self.edit_seq_notes.textChanged.connect(lambda t: setattr(self._sequence, 'notes', t))
        bar.addWidget(self.edit_seq_notes, stretch=1)

        for label, slot, style in [
            ("Save Seq",    self._on_save_seq,     ""),
            ("Load Seq",    self._on_load_seq,     ""),
            ("Gen CSV",     self._on_gen_csv,      "background:#c41c3b;"),
            ("Validate",    self._on_validate,     "background:#1565c0;"),
            ("Dry-run",     self._on_dry_run,      "background:#4a148c;"),
        ]:
            btn = QPushButton(label)
            if style:
                btn.setStyleSheet(btn.styleSheet() + style)
            btn.clicked.connect(slot)
            bar.addWidget(btn)
            setattr(self, f"btn_{label.lower().replace(' ', '_').replace('-', '_')}", btn)

        return bar

    # ---- node list panel ---------------------------------------------------

    def _build_node_list_panel(self) -> QWidget:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 4, 0)

        lbl = QLabel("Nodes")
        lbl.setStyleSheet("font-weight: bold; font-size: 13px;")
        layout.addWidget(lbl)

        self.list_nodes = QListWidget()
        self.list_nodes.currentRowChanged.connect(self._on_node_selected)
        layout.addWidget(self.list_nodes, stretch=1)

        # Node operation buttons
        btn_grid = QGridLayout()
        btn_grid.setSpacing(4)
        self.btn_node_add  = QPushButton("Add")
        self.btn_node_del  = QPushButton("Delete")
        self.btn_node_dup  = QPushButton("Duplicate")
        self.btn_node_up   = QPushButton("↑ Up")
        self.btn_node_down = QPushButton("↓ Down")
        self.btn_node_add.clicked.connect(self._on_node_add)
        self.btn_node_del.clicked.connect(self._on_node_del)
        self.btn_node_dup.clicked.connect(self._on_node_dup)
        self.btn_node_up.clicked.connect(self._on_node_up)
        self.btn_node_down.clicked.connect(self._on_node_down)
        btn_grid.addWidget(self.btn_node_add,  0, 0)
        btn_grid.addWidget(self.btn_node_del,  0, 1)
        btn_grid.addWidget(self.btn_node_dup,  1, 0)
        btn_grid.addWidget(self.btn_node_up,   1, 1)
        btn_grid.addWidget(self.btn_node_down, 2, 0, 1, 2)
        layout.addLayout(btn_grid)

        # Limit profile section
        sep = QFrame()
        sep.setFrameShape(QFrame.HLine)
        sep.setStyleSheet("color: #555;")
        layout.addWidget(sep)

        lbl_lim = QLabel("Limit Profile")
        lbl_lim.setStyleSheet("font-weight: bold; font-size: 11px; color: #aaa;")
        layout.addWidget(lbl_lim)

        self.lbl_limit_status = QLabel("Not loaded")
        self.lbl_limit_status.setStyleSheet("color: #ffea00; font-size: 11px;")
        self.lbl_limit_status.setWordWrap(True)
        layout.addWidget(self.lbl_limit_status)

        lim_btns = QHBoxLayout()
        self.btn_load_limit   = QPushButton("Load Profile")
        self.btn_save_limit   = QPushButton("Save Profile")
        self.btn_new_limit    = QPushButton("New Template")
        self.btn_clear_limit  = QPushButton("Clear")
        self.btn_load_limit.clicked.connect(self._on_load_limit)
        self.btn_save_limit.clicked.connect(self._on_save_limit)
        self.btn_new_limit.clicked.connect(self._on_new_limit_template)
        self.btn_clear_limit.clicked.connect(self._on_clear_limit)
        lim_btns.addWidget(self.btn_load_limit)
        lim_btns.addWidget(self.btn_save_limit)
        lim_btns.addWidget(self.btn_new_limit)
        lim_btns.addWidget(self.btn_clear_limit)
        layout.addLayout(lim_btns)

        # Limit editor (collapsible + scrollable to avoid layout overflow)
        self.btn_limit_editor_toggle = QPushButton('▶ Limit Template Editor')
        self.btn_limit_editor_toggle.setCheckable(True)
        self.btn_limit_editor_toggle.setChecked(False)
        layout.addWidget(self.btn_limit_editor_toggle)

        grp_lim_edit = QGroupBox('')
        lim_grid = QGridLayout()
        lim_grid.setSpacing(4)
        lim_grid.addWidget(QLabel("Joint"), 0, 0)
        lim_grid.addWidget(QLabel("Min"), 0, 1)
        lim_grid.addWidget(QLabel("Max"), 0, 2)
        lim_grid.addWidget(QLabel("Max Speed"), 0, 3)

        self.limit_profile_name = QLineEdit('default_template')
        self.limit_profile_name.setPlaceholderText('profile name')
        lim_grid.addWidget(QLabel('Profile Name'), 1, 0)
        lim_grid.addWidget(self.limit_profile_name, 1, 1, 1, 3)

        self.limit_spins: dict[tuple[str, str, str], QDoubleSpinBox] = {}
        row = 2
        for leg in LEGS:
            leg_label = QLabel(f'Leg {leg}')
            leg_label.setStyleSheet('font-weight: bold; color: #ccc;')
            lim_grid.addWidget(leg_label, row, 0, 1, 4)
            row += 1
            for joint in JOINTS:
                lim_grid.addWidget(QLabel(f'  {joint}'), row, 0)
                for col, key in enumerate(('min', 'max', 'max_speed'), start=1):
                    spin = QDoubleSpinBox()
                    spin.setDecimals(2)
                    spin.setRange(-2000.0, 2000.0)
                    spin.setSingleStep(1.0)
                    if key == 'max_speed':
                        spin.setRange(0.0, 5000.0)
                    self.limit_spins[(leg, joint, key)] = spin
                    lim_grid.addWidget(spin, row, col)
                row += 1

        lim_btn_row = QHBoxLayout()
        self.btn_apply_limit_editor = QPushButton('Apply Limits To Profile')
        self.btn_apply_limit_editor.clicked.connect(self._on_apply_limit_editor)
        lim_btn_row.addWidget(self.btn_apply_limit_editor)
        lim_btn_row.addStretch()

        grp_lim_layout = QVBoxLayout(grp_lim_edit)
        grp_lim_layout.setContentsMargins(6, 6, 6, 6)
        grp_lim_layout.addLayout(lim_grid)
        grp_lim_layout.addLayout(lim_btn_row)

        self.limit_editor_scroll = QScrollArea()
        self.limit_editor_scroll.setWidgetResizable(True)
        self.limit_editor_scroll.setWidget(grp_lim_edit)
        self.limit_editor_scroll.setVisible(False)
        self.limit_editor_scroll.setMinimumHeight(180)
        self.limit_editor_scroll.setMaximumHeight(260)
        layout.addWidget(self.limit_editor_scroll)

        def _toggle_limit_editor(checked: bool):
            self.limit_editor_scroll.setVisible(checked)
            arrow = '▼' if checked else '▶'
            self.btn_limit_editor_toggle.setText(f'{arrow} Limit Template Editor')

        self.btn_limit_editor_toggle.toggled.connect(_toggle_limit_editor)
        self._sync_limit_editor_from_profile()

        return panel

    # ---- node editor panel -------------------------------------------------

    def _build_node_editor_panel(self) -> QWidget:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(4, 0, 0, 0)

        # --- node meta row ---
        meta_layout = QHBoxLayout()
        meta_layout.addWidget(QLabel("Name:"))
        self.edit_node_name = QLineEdit()
        self.edit_node_name.setPlaceholderText("node name")
        self.edit_node_name.setMaximumWidth(180)
        meta_layout.addWidget(self.edit_node_name)

        meta_layout.addWidget(QLabel("Duration (s):"))
        self.spin_duration = QDoubleSpinBox()
        self.spin_duration.setDecimals(2)
        self.spin_duration.setRange(0.01, 3600.0)
        self.spin_duration.setSingleStep(0.5)
        self.spin_duration.setValue(1.0)
        self.spin_duration.setMaximumWidth(90)
        meta_layout.addWidget(self.spin_duration)
        meta_layout.addStretch()
        layout.addLayout(meta_layout)

        # --- node notes ---
        notes_layout = QHBoxLayout()
        notes_layout.addWidget(QLabel("Notes:"))
        self.edit_node_notes = QLineEdit()
        self.edit_node_notes.setPlaceholderText("(optional)")
        notes_layout.addWidget(self.edit_node_notes, stretch=1)
        layout.addLayout(notes_layout)

        # --- quick-action row ---
        quick = QHBoxLayout()
        self.btn_grab_pose = QPushButton("⟳ Grab Current Pose")
        self.btn_grab_pose.setToolTip("Fill all 12 joints from current motor state")
        self.btn_grab_pose.clicked.connect(self._on_grab_pose)
        quick.addWidget(self.btn_grab_pose)

        self.btn_copy_a_to_all = QPushButton("Copy A → B,C,D")
        self.btn_copy_a_to_all.setToolTip("Copy leg A targets to B, C and D")
        self.btn_copy_a_to_all.clicked.connect(self._on_copy_a_to_all)
        quick.addWidget(self.btn_copy_a_to_all)

        quick.addWidget(QLabel("Template:"))
        self.combo_template = QComboBox()
        self.combo_template.addItem("-- select --")
        quick.addWidget(self.combo_template)

        self.btn_apply_template = QPushButton("Apply")
        self.btn_apply_template.clicked.connect(self._on_apply_template)
        quick.addWidget(self.btn_apply_template)

        self.btn_save_template = QPushButton("Save As Template")
        self.btn_save_template.clicked.connect(self._on_save_template)
        quick.addWidget(self.btn_save_template)

        self.btn_del_template = QPushButton("Delete Template")
        self.btn_del_template.clicked.connect(self._on_delete_template)
        quick.addWidget(self.btn_del_template)

        quick.addStretch()
        layout.addLayout(quick)

        # --- joint spinbox grid ---
        grp_joints = QGroupBox("Joint Targets (degrees)")
        joints_grid = QGridLayout()
        joints_grid.setSpacing(6)

        # Header row
        for col, jname in enumerate(('', 'θ  theta', 'β  beta', 'γ  gamma'), start=0):
            hdr = QLabel(jname)
            hdr.setAlignment(Qt.AlignCenter)
            hdr.setStyleSheet("font-weight: bold; color: #ccc;")
            joints_grid.addWidget(hdr, 0, col)

        self._joint_spins: dict[tuple[str, str], QDoubleSpinBox] = {}
        for row, leg in enumerate(LEGS, start=1):
            lbl = QLabel(f"Leg {leg}")
            lbl.setStyleSheet("color: #aaa;")
            joints_grid.addWidget(lbl, row, 0)
            for col, joint in enumerate(JOINTS, start=1):
                spin = QDoubleSpinBox()
                spin.setDecimals(2)
                spin.setSingleStep(0.5)
                spin.setRange(-720.0, 720.0)
                spin.setValue(0.0)
                self._joint_spins[(leg, joint)] = spin
                joints_grid.addWidget(spin, row, col)

        grp_joints.setLayout(joints_grid)
        layout.addWidget(grp_joints)

        # --- per-node gains ---
        grp_pd = QGroupBox('Node PD Gains')
        pd_grid = QGridLayout()
        pd_grid.addWidget(QLabel('θ/β Kp'), 0, 0)
        self.spin_node_leg_kp = QDoubleSpinBox()
        self.spin_node_leg_kp.setDecimals(2)
        self.spin_node_leg_kp.setRange(0.0, 10000.0)
        self.spin_node_leg_kp.setValue(120.0)
        pd_grid.addWidget(self.spin_node_leg_kp, 0, 1)

        pd_grid.addWidget(QLabel('θ/β Kd'), 0, 2)
        self.spin_node_leg_kd = QDoubleSpinBox()
        self.spin_node_leg_kd.setDecimals(2)
        self.spin_node_leg_kd.setRange(0.0, 1000.0)
        self.spin_node_leg_kd.setValue(0.25)
        pd_grid.addWidget(self.spin_node_leg_kd, 0, 3)

        pd_grid.addWidget(QLabel('γ Kp'), 1, 0)
        self.spin_node_gamma_kp = QDoubleSpinBox()
        self.spin_node_gamma_kp.setDecimals(2)
        self.spin_node_gamma_kp.setRange(0.0, 10000.0)
        self.spin_node_gamma_kp.setValue(150.0)
        pd_grid.addWidget(self.spin_node_gamma_kp, 1, 1)

        pd_grid.addWidget(QLabel('γ Kd'), 1, 2)
        self.spin_node_gamma_kd = QDoubleSpinBox()
        self.spin_node_gamma_kd.setDecimals(2)
        self.spin_node_gamma_kd.setRange(0.0, 1000.0)
        self.spin_node_gamma_kd.setValue(1.75)
        pd_grid.addWidget(self.spin_node_gamma_kd, 1, 3)
        grp_pd.setLayout(pd_grid)
        layout.addWidget(grp_pd)

        # --- apply / discard node edits ---
        apply_bar = QHBoxLayout()
        self.btn_node_apply = QPushButton("Apply Changes to Node")
        self.btn_node_apply.setStyleSheet("background: #2e7d32;")
        self.btn_node_apply.clicked.connect(self._on_node_apply)
        self.btn_node_discard = QPushButton("Discard Changes")
        self.btn_node_discard.clicked.connect(self._on_node_discard)
        apply_bar.addWidget(self.btn_node_apply)
        apply_bar.addWidget(self.btn_node_discard)
        apply_bar.addStretch()
        layout.addLayout(apply_bar)

        layout.addStretch()
        self._set_editor_enabled(False)
        self._refresh_template_combo()
        return panel

    # ---- run / progress bar ------------------------------------------------

    def _build_run_bar(self) -> QVBoxLayout:
        vbox = QVBoxLayout()

        # Progress row
        prog_row = QHBoxLayout()
        self.lbl_progress = QLabel("Ready")
        self.lbl_progress.setMinimumWidth(200)
        self.lbl_progress.setStyleSheet("color: #aaa;")
        prog_row.addWidget(self.lbl_progress)

        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        self.progress_bar.setTextVisible(True)
        self.progress_bar.setFormat("Node %v / %m")
        self.progress_bar.setMaximumHeight(18)
        prog_row.addWidget(self.progress_bar, stretch=1)

        self.lbl_eta = QLabel("ETA: --")
        self.lbl_eta.setStyleSheet("color: #aaa;")
        prog_row.addWidget(self.lbl_eta)

        vbox.addLayout(prog_row)

        # Button row
        btn_row = QHBoxLayout()
        self.btn_run = QPushButton("▶  Run Sequence")
        self.btn_run.setStyleSheet("background: #1b5e20; font-weight: bold; padding: 6px 20px;")
        self.btn_run.clicked.connect(self._on_run)

        self.btn_stop = QPushButton("■  Stop")
        self.btn_stop.setStyleSheet("background: #b71c1c; font-weight: bold; padding: 6px 20px;")
        self.btn_stop.setEnabled(False)
        self.btn_stop.clicked.connect(self._on_stop)

        btn_row.addWidget(self.btn_run)
        btn_row.addWidget(self.btn_stop)
        btn_row.addStretch()
        vbox.addLayout(btn_row)

        return vbox

    # ---- embedded log ------------------------------------------------------

    def _build_log_panel(self) -> QWidget:
        grp = QGroupBox("Sequence Log")
        grp.setMaximumHeight(160)
        layout = QVBoxLayout(grp)
        layout.setContentsMargins(4, 4, 4, 4)
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setStyleSheet(
            "QTextEdit { background:#1e1e1e; color:#ddd; "
            "font-family: 'Consolas','Courier New',monospace; font-size: 11px; }"
        )
        layout.addWidget(self.log_text)
        return grp

    # =========================================================================
    # Node list helpers
    # =========================================================================

    def _refresh_node_list(self) -> None:
        prev_row = self.list_nodes.currentRow()
        self.list_nodes.blockSignals(True)
        self.list_nodes.clear()
        for i, node in enumerate(self._sequence.nodes):
            item = QListWidgetItem(f"{i:02d}  {node.name}  ({node.duration_sec:.2f}s)")
            self.list_nodes.addItem(item)
        self.list_nodes.blockSignals(False)

        # Restore selection
        if self._sequence.nodes:
            row = max(0, min(prev_row, len(self._sequence.nodes) - 1))
            self.list_nodes.setCurrentRow(row)
        else:
            self._selected_node_idx = -1
            self._set_editor_enabled(False)

        self.progress_bar.setMaximum(max(1, len(self._sequence.nodes)))

    def _load_node_into_editor(self, idx: int) -> None:
        if idx < 0 or idx >= len(self._sequence.nodes):
            return
        node = self._sequence.nodes[idx]
        self.edit_node_name.setText(node.name)
        self.spin_duration.setValue(node.duration_sec)
        self.edit_node_notes.setText(node.notes)
        self.spin_node_leg_kp.setValue(float(node.gains.get('leg_kp', 120.0)))
        self.spin_node_leg_kd.setValue(float(node.gains.get('leg_kd', 0.25)))
        self.spin_node_gamma_kp.setValue(float(node.gains.get('gamma_kp', 150.0)))
        self.spin_node_gamma_kd.setValue(float(node.gains.get('gamma_kd', 1.75)))
        for leg in LEGS:
            for joint in JOINTS:
                self._joint_spins[(leg, joint)].setValue(
                    getattr(node.targets[leg], joint)
                )
        self._set_editor_enabled(not self._exec_locked)

    def _set_editor_enabled(self, enabled: bool) -> None:
        for w in (
            self.edit_node_name, self.spin_duration, self.edit_node_notes,
            self.btn_node_apply, self.btn_node_discard,
            self.btn_grab_pose, self.btn_copy_a_to_all,
            self.btn_apply_template, self.combo_template,
            self.btn_save_template, self.btn_del_template,
            self.spin_node_leg_kp, self.spin_node_leg_kd,
            self.spin_node_gamma_kp, self.spin_node_gamma_kd,
        ):
            w.setEnabled(enabled)
        for spin in self._joint_spins.values():
            spin.setEnabled(enabled)

    def _set_exec_locked(self, locked: bool) -> None:
        self._exec_locked = locked
        if locked and self.btn_limit_editor_toggle.isChecked():
            self.btn_limit_editor_toggle.setChecked(False)
        # Node list + operation buttons
        for w in (
            self.list_nodes, self.btn_node_add, self.btn_node_del,
            self.btn_node_dup, self.btn_node_up, self.btn_node_down,
        ):
            w.setEnabled(not locked)
        # Header buttons
        for w in (
            self.edit_seq_name, self.edit_seq_notes,
            self.btn_save_seq, self.btn_load_seq, self.btn_gen_csv,
            self.btn_validate, self.btn_dry_run,
            self.btn_load_limit, self.btn_save_limit, self.btn_clear_limit,
            self.btn_new_limit, self.btn_apply_limit_editor,
            self.limit_profile_name, self.btn_limit_editor_toggle,
        ):
            w.setEnabled(not locked)
        for spin in self.limit_spins.values():
            spin.setEnabled(not locked)
        # Editor
        self._set_editor_enabled(
            (not locked) and self._selected_node_idx >= 0
        )
        # Run / Stop
        self.btn_run.setEnabled(not locked)
        self.btn_stop.setEnabled(locked)

    # =========================================================================
    # Node list slots
    # =========================================================================

    def _on_node_selected(self, row: int) -> None:
        self._selected_node_idx = row
        if row >= 0:
            self._load_node_into_editor(row)
        else:
            self._set_editor_enabled(False)

    def _on_node_add(self) -> None:
        node = SequenceNode(name=f"node_{len(self._sequence.nodes)}")
        self._sequence.nodes.append(node)
        self._refresh_node_list()
        self.list_nodes.setCurrentRow(len(self._sequence.nodes) - 1)
        self._add_log(f"Added node '{node.name}'", LOGLEVEL.INFO)

    def _on_node_del(self) -> None:
        idx = self._selected_node_idx
        if idx < 0 or idx >= len(self._sequence.nodes):
            return
        name = self._sequence.nodes[idx].name
        del self._sequence.nodes[idx]
        self._refresh_node_list()
        self._add_log(f"Deleted node '{name}'", LOGLEVEL.INFO)

    def _on_node_dup(self) -> None:
        idx = self._selected_node_idx
        if idx < 0 or idx >= len(self._sequence.nodes):
            return
        copy = self._sequence.nodes[idx].copy()
        self._sequence.nodes.insert(idx + 1, copy)
        self._refresh_node_list()
        self.list_nodes.setCurrentRow(idx + 1)
        self._add_log(f"Duplicated node '{copy.name}'", LOGLEVEL.INFO)

    def _on_node_up(self) -> None:
        idx = self._selected_node_idx
        if idx <= 0:
            return
        nodes = self._sequence.nodes
        nodes[idx - 1], nodes[idx] = nodes[idx], nodes[idx - 1]
        self._refresh_node_list()
        self.list_nodes.setCurrentRow(idx - 1)

    def _on_node_down(self) -> None:
        idx = self._selected_node_idx
        nodes = self._sequence.nodes
        if idx < 0 or idx >= len(nodes) - 1:
            return
        nodes[idx], nodes[idx + 1] = nodes[idx + 1], nodes[idx]
        self._refresh_node_list()
        self.list_nodes.setCurrentRow(idx + 1)

    # =========================================================================
    # Node editor slots
    # =========================================================================

    def _on_node_apply(self) -> None:
        idx = self._selected_node_idx
        if idx < 0 or idx >= len(self._sequence.nodes):
            return
        node = self._sequence.nodes[idx]
        node.name        = self.edit_node_name.text().strip() or f"node_{idx}"
        node.duration_sec = self.spin_duration.value()
        node.notes       = self.edit_node_notes.text()
        node.gains = {
            'leg_kp': self.spin_node_leg_kp.value(),
            'leg_kd': self.spin_node_leg_kd.value(),
            'gamma_kp': self.spin_node_gamma_kp.value(),
            'gamma_kd': self.spin_node_gamma_kd.value(),
        }
        for leg in LEGS:
            for joint in JOINTS:
                setattr(node.targets[leg], joint,
                        self._joint_spins[(leg, joint)].value())
        self._refresh_node_list()
        self.list_nodes.setCurrentRow(idx)
        self._add_log(f"Node [{idx}] '{node.name}' updated", LOGLEVEL.INFO)

    def _on_node_discard(self) -> None:
        self._load_node_into_editor(self._selected_node_idx)
        self._add_log("Edits discarded", LOGLEVEL.INFO)

    def _on_grab_pose(self) -> None:
        pose = self._get_current_pose()
        if pose is None:
            self._add_log("Current pose not available (motor state not received)", LOGLEVEL.WARN)
            return
        for leg in LEGS:
            for joint in JOINTS:
                val = pose.get(leg, {}).get(joint, 0.0)
                self._joint_spins[(leg, joint)].setValue(float(val))
        self._add_log("Grabbed current pose into editor", LOGLEVEL.INFO)

    def _on_copy_a_to_all(self) -> None:
        for joint in JOINTS:
            val = self._joint_spins[('A', joint)].value()
            for leg in ('B', 'C', 'D'):
                self._joint_spins[(leg, joint)].setValue(val)
        self._add_log("Copied Leg A targets to B, C, D", LOGLEVEL.INFO)

    def _on_apply_template(self) -> None:
        name = self.combo_template.currentText()
        if name in BUILTIN_TEMPLATES:
            tmpl = BUILTIN_TEMPLATES[name]
        elif name in self._custom_templates:
            data = self._custom_templates[name]
            try:
                tmpl = SequenceNode(
                    name=name,
                    duration_sec=float(data.get('duration_sec', 1.0)),
                    targets={
                        leg: JointTarget(
                            theta=float(data.get('targets', {}).get(leg, {}).get('theta', 0.0)),
                            beta=float(data.get('targets', {}).get(leg, {}).get('beta', 0.0)),
                            gamma=float(data.get('targets', {}).get(leg, {}).get('gamma', 0.0)),
                        )
                        for leg in LEGS
                    },
                    gains={
                        'leg_kp': float(data.get('gains', {}).get('leg_kp', 120.0)),
                        'leg_kd': float(data.get('gains', {}).get('leg_kd', 0.25)),
                        'gamma_kp': float(data.get('gains', {}).get('gamma_kp', 150.0)),
                        'gamma_kd': float(data.get('gains', {}).get('gamma_kd', 1.75)),
                    },
                )
            except Exception as e:
                self._add_log(f"Template parse failed: {e}", LOGLEVEL.ERROR)
                return
        else:
            self._add_log('Select a valid template first', LOGLEVEL.WARN)
            return

        for leg in LEGS:
            for joint in JOINTS:
                self._joint_spins[(leg, joint)].setValue(
                    getattr(tmpl.targets[leg], joint)
                )
        self.spin_duration.setValue(tmpl.duration_sec)
        self.spin_node_leg_kp.setValue(float(tmpl.gains.get('leg_kp', 120.0)))
        self.spin_node_leg_kd.setValue(float(tmpl.gains.get('leg_kd', 0.25)))
        self.spin_node_gamma_kp.setValue(float(tmpl.gains.get('gamma_kp', 150.0)))
        self.spin_node_gamma_kd.setValue(float(tmpl.gains.get('gamma_kd', 1.75)))
        self._on_node_apply()
        self._add_log(f"Applied template '{name}' to current node", LOGLEVEL.INFO)

    def _on_save_template(self) -> None:
        idx = self._selected_node_idx
        if idx < 0 or idx >= len(self._sequence.nodes):
            self._add_log('Select a node before saving template', LOGLEVEL.WARN)
            return
        self._on_node_apply()
        node = self._sequence.nodes[idx]
        name = node.name.strip() or f'template_{idx}'
        self._custom_templates[name] = {
            'duration_sec': node.duration_sec,
            'targets': {
                leg: {
                    'theta': node.targets[leg].theta,
                    'beta': node.targets[leg].beta,
                    'gamma': node.targets[leg].gamma,
                } for leg in LEGS
            },
            'gains': dict(node.gains),
        }
        self._save_custom_templates()
        self._refresh_template_combo()
        self.combo_template.setCurrentText(name)
        self._add_log(f"Template '{name}' saved", LOGLEVEL.INFO)

    def _on_delete_template(self) -> None:
        name = self.combo_template.currentText()
        if name not in self._custom_templates:
            self._add_log('Only custom templates can be deleted', LOGLEVEL.WARN)
            return
        del self._custom_templates[name]
        self._save_custom_templates()
        self._refresh_template_combo()
        self._add_log(f"Template '{name}' deleted", LOGLEVEL.INFO)

    def _on_seq_name_changed(self, text: str) -> None:
        self._sequence.name = text.strip() or "my_sequence"

    # =========================================================================
    # Save / Load sequence
    # =========================================================================

    def _on_save_seq(self) -> None:
        path, _ = QFileDialog.getSaveFileName(
            self, "Save Sequence",
            os.path.join(PATHS.DEFAULT_OUTPUT_DIR, f"{self._sequence.name}.json"),
            "JSON Files (*.json);;All Files (*)",
        )
        if not path:
            return
        try:
            save_sequence(path, self._sequence)
            self._add_log(f"Sequence saved → {path}", LOGLEVEL.INFO)
        except Exception as e:
            self._add_log(f"Save failed: {e}", LOGLEVEL.ERROR)

    def _on_load_seq(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "Load Sequence",
            PATHS.DEFAULT_OUTPUT_DIR,
            "JSON Files (*.json);;All Files (*)",
        )
        if not path:
            return
        try:
            seq = load_sequence(path)
            self._sequence = seq
            self.edit_seq_name.setText(seq.name)
            self.edit_seq_notes.setText(seq.notes)
            
            # Auto-load matching profile if one was found
            if seq.limit_profile_path and os.path.exists(seq.limit_profile_path):
                try:
                    self._limit_profile = load_limit_profile(seq.limit_profile_path)
                    self._sync_limit_editor_from_profile()
                    self.lbl_limit_status.setText(
                        f"✔ {self._limit_profile.profile_name}\n({os.path.basename(seq.limit_profile_path)})"
                    )
                    self.lbl_limit_status.setStyleSheet("color: #00e676; font-size: 11px;")
                    self._add_log(
                        f"Profile auto-loaded: {self._limit_profile.profile_name}",
                        LOGLEVEL.INFO
                    )
                except Exception as profile_err:
                    self._add_log(
                        f"Profile auto-load failed: {profile_err}",
                        LOGLEVEL.WARN
                    )
            
            self._refresh_node_list()
            self._add_log(f"Sequence loaded ← {path}  ({len(seq.nodes)} nodes)", LOGLEVEL.INFO)
        except Exception as e:
            self._add_log(f"Load failed: {e}", LOGLEVEL.ERROR)
            QMessageBox.warning(self, "Load Sequence Failed", str(e))

    def _on_gen_csv(self) -> None:
        """Generate CSV file from current sequence."""
        if not self._sequence.nodes:
            QMessageBox.warning(self, "Generate CSV", "Sequence is empty! Add nodes first.")
            return
        
        path, _ = QFileDialog.getSaveFileName(
            self, "Generate CSV",
            os.path.join(PATHS.DEFAULT_OUTPUT_DIR, f"{self._sequence.name}.csv"),
            "CSV Files (*.csv);;All Files (*)",
        )
        if not path:
            return
        
        try:
            with open(path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                for node in self._sequence.nodes:
                    row = [
                        # A: theta, beta
                        node.targets['A'].theta, node.targets['A'].beta,
                        # B: theta, beta
                        node.targets['B'].theta, node.targets['B'].beta,
                        # C: theta, beta
                        node.targets['C'].theta, node.targets['C'].beta,
                        # D: theta, beta
                        node.targets['D'].theta, node.targets['D'].beta,
                        # Gamma: A, B, C, D
                        node.targets['A'].gamma,
                        node.targets['B'].gamma,
                        node.targets['C'].gamma,
                        node.targets['D'].gamma,
                    ]
                    writer.writerow(row)
            self._add_log(f"CSV generated → {path}  ({len(self._sequence.nodes)} rows)", LOGLEVEL.INFO)
        except Exception as e:
            self._add_log(f"CSV generation failed: {e}", LOGLEVEL.ERROR)
            QMessageBox.critical(self, "Generate CSV Failed", str(e))

    # =========================================================================
    # Limit profile
    # =========================================================================

    def _on_load_limit(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "Load Limit Profile",
            PATHS.DEFAULT_OUTPUT_DIR,
            "JSON Files (*.json);;All Files (*)",
        )
        if not path:
            return
        try:
            self._limit_profile = load_limit_profile(path)
            self._sequence.limit_profile_path = path
            self._sync_limit_editor_from_profile()
            self.lbl_limit_status.setText(
                f"✔ {self._limit_profile.profile_name}\n({os.path.basename(path)})"
            )
            self.lbl_limit_status.setStyleSheet("color: #00e676; font-size: 11px;")
            self._add_log(f"Limit profile loaded: {self._limit_profile.profile_name}", LOGLEVEL.INFO)
        except Exception as e:
            self._add_log(f"Load limit profile failed: {e}", LOGLEVEL.ERROR)
            QMessageBox.warning(self, "Load Limit Profile Failed", str(e))

    def _on_save_limit(self) -> None:
        if self._limit_profile is None:
            self._add_log("No limit profile loaded to save", LOGLEVEL.WARN)
            return
        self._on_apply_limit_editor()
        path, _ = QFileDialog.getSaveFileName(
            self, "Save Limit Profile",
            os.path.join(PATHS.DEFAULT_OUTPUT_DIR,
                         f"{self._limit_profile.profile_name}_limits.json"),
            "JSON Files (*.json);;All Files (*)",
        )
        if not path:
            return
        try:
            save_limit_profile(path, self._limit_profile)
            self._add_log(f"Limit profile saved → {path}", LOGLEVEL.INFO)
        except Exception as e:
            self._add_log(f"Save limit profile failed: {e}", LOGLEVEL.ERROR)

    def _on_clear_limit(self) -> None:
        self._limit_profile = _default_limit_profile()
        self._sync_limit_editor_from_profile()
        self._sequence.limit_profile_path = None
        self.lbl_limit_status.setText("Using default template")
        self.lbl_limit_status.setStyleSheet("color: #00e676; font-size: 11px;")
        self._add_log("Limit profile reset to default template", LOGLEVEL.INFO)

    def _on_new_limit_template(self) -> None:
        self._limit_profile = _default_limit_profile()
        self._sync_limit_editor_from_profile()
        self.lbl_limit_status.setText('Using new editable template')
        self.lbl_limit_status.setStyleSheet('color: #00e676; font-size: 11px;')
        self._add_log('Created new editable limit template', LOGLEVEL.INFO)

    def _sync_limit_editor_from_profile(self) -> None:
        if self._limit_profile is None:
            return
        self.limit_profile_name.setText(self._limit_profile.profile_name)
        for leg in LEGS:
            for joint in JOINTS:
                entry = self._limit_profile.limits[(leg, joint)]
                self.limit_spins[(leg, joint, 'min')].setValue(entry.min_deg)
                self.limit_spins[(leg, joint, 'max')].setValue(entry.max_deg)
                self.limit_spins[(leg, joint, 'max_speed')].setValue(entry.max_speed_deg_per_sec)

    def _on_apply_limit_editor(self) -> None:
        if self._limit_profile is None:
            self._limit_profile = _default_limit_profile()
        self._limit_profile.profile_name = self.limit_profile_name.text().strip() or 'custom_profile'
        for leg in LEGS:
            for joint in JOINTS:
                min_v = self.limit_spins[(leg, joint, 'min')].value()
                max_v = self.limit_spins[(leg, joint, 'max')].value()
                max_speed = self.limit_spins[(leg, joint, 'max_speed')].value()
                if max_v < min_v:
                    min_v, max_v = max_v, min_v
                entry = self._limit_profile.limits[(leg, joint)]
                entry.min_deg = min_v
                entry.max_deg = max_v
                entry.max_speed_deg_per_sec = max_speed
        self.lbl_limit_status.setText(f"✔ {self._limit_profile.profile_name} (edited)")
        self.lbl_limit_status.setStyleSheet("color: #00e676; font-size: 11px;")
        self._add_log('Applied limit editor values to profile', LOGLEVEL.INFO)

    # =========================================================================
    # Validate / Dry-run
    # =========================================================================

    def _on_validate(self) -> None:
        """Run reachability check and display results in the window log."""
        from corgi_ui.core.sequence_model import check_sequence_reachability
        if not self._sequence.nodes:
            self._add_log("Sequence is empty – nothing to validate", LOGLEVEL.WARN)
            return
        pose = self._get_current_pose()
        errors = check_sequence_reachability(self._sequence, self._limit_profile, pose)
        if not errors:
            total_time = sum(n.duration_sec for n in self._sequence.nodes)
            self._add_log(
                f"Validation OK — {len(self._sequence.nodes)} nodes, "
                f"total {total_time:.2f}s",
                LOGLEVEL.INFO,
            )
        else:
            self._add_log(
                f"Validation found {len(errors)} issue(s):", LOGLEVEL.WARN
            )
            for e in errors:
                self._add_log(
                    f"  Node[{e.node_idx}] '{e.node_name}' "
                    f"leg={e.leg} joint={e.joint}: {e.message}",
                    LOGLEVEL.ERROR,
                )

    def _on_dry_run(self) -> None:
        self.run_requested.emit(self._sequence, self._limit_profile, True)

    # =========================================================================
    # Run / Stop
    # =========================================================================

    def _on_run(self) -> None:
        if not self._sequence.nodes:
            self._add_log("Cannot run – sequence is empty", LOGLEVEL.WARN)
            return
        self.run_requested.emit(self._sequence, self._limit_profile, False)

    def _on_stop(self) -> None:
        self.stop_requested.emit()

    # =========================================================================
    # External state update (called by CorgiControlPanel executor)
    # =========================================================================

    def update_execution_state(
        self,
        state: ExecutionState,
        node_idx: int,
        total_nodes: int,
        progress: float,
        eta_sec: float,
        message: str = "",
    ) -> None:
        """
        Called by the panel executor on every timer tick to update the
        progress bar and status label.

        Parameters
        ----------
        state:       Current ExecutionState.
        node_idx:    Zero-based index of the currently running node.
        total_nodes: Total number of nodes in the sequence.
        progress:    Fraction [0.0, 1.0] of the current node's duration elapsed.
        eta_sec:     Estimated remaining seconds (current node + remaining nodes).
        message:     Optional status message.
        """
        locked = (state == ExecutionState.RUNNING_NODE)
        if locked != self._exec_locked:
            self._set_exec_locked(locked)

        if state == ExecutionState.RUNNING_NODE:
            pct = int(progress * 100)
            self.progress_bar.setMaximum(total_nodes)
            self.progress_bar.setValue(node_idx)
            self.progress_bar.setFormat(f"Node {node_idx + 1}/{total_nodes}  {pct}%")
            self.lbl_progress.setText(
                f"Running node {node_idx + 1}/{total_nodes}"
            )
            self.lbl_eta.setText(f"ETA: {eta_sec:.1f}s")
            self.lbl_progress.setStyleSheet("color: #00e676;")

        elif state == ExecutionState.COMPLETED:
            self.progress_bar.setValue(total_nodes)
            self.progress_bar.setFormat("Completed")
            self.lbl_progress.setText("Sequence completed ✔")
            self.lbl_progress.setStyleSheet("color: #00e676;")
            self.lbl_eta.setText("")
            self._add_log("Sequence completed successfully", LOGLEVEL.INFO)

        elif state == ExecutionState.FAILED:
            self.progress_bar.setFormat("Failed")
            self.lbl_progress.setText(f"FAILED: {message}")
            self.lbl_progress.setStyleSheet("color: #ff5252;")
            self.lbl_eta.setText("")
            self._add_log(f"Sequence FAILED: {message}", LOGLEVEL.ERROR)

        elif state == ExecutionState.STOPPED:
            self.progress_bar.setFormat("Stopped")
            self.lbl_progress.setText(f"Stopped at node {node_idx + 1}")
            self.lbl_progress.setStyleSheet("color: #ffea00;")
            self.lbl_eta.setText("")
            self._add_log(f"Sequence stopped at node {node_idx + 1}", LOGLEVEL.WARN)

        elif state == ExecutionState.IDLE:
            self.progress_bar.setValue(0)
            self.progress_bar.setFormat("Ready")
            self.lbl_progress.setText("Ready")
            self.lbl_progress.setStyleSheet("color: #aaa;")
            self.lbl_eta.setText("ETA: --")

    def log_node_event(self, message: str, level: LOGLEVEL = LOGLEVEL.INFO) -> None:
        """Append a message to the embedded sequence log."""
        self._add_log(message, level)

    # =========================================================================
    # Internal log
    # =========================================================================

    def _add_log(self, message: str, level: LOGLEVEL = LOGLEVEL.INFO) -> None:
        timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        html = format_log_html(timestamp, level, "seq", message)
        self.log_text.append(html)
        sb = self.log_text.verticalScrollBar()
        sb.setValue(sb.maximum())

    def _load_custom_templates(self) -> None:
        self._custom_templates = {}
        if not os.path.exists(self._template_file):
            return
        try:
            import json
            with open(self._template_file, 'r', encoding='utf-8') as f:
                d = json.load(f)
            if isinstance(d, dict):
                self._custom_templates = d
        except Exception:
            self._custom_templates = {}

    def _save_custom_templates(self) -> None:
        try:
            import json
            os.makedirs(os.path.dirname(self._template_file), exist_ok=True)
            with open(self._template_file, 'w', encoding='utf-8') as f:
                json.dump(self._custom_templates, f, indent=2, ensure_ascii=False)
        except Exception as e:
            self._add_log(f'Could not save template file: {e}', LOGLEVEL.WARN)

    def _refresh_template_combo(self) -> None:
        current = self.combo_template.currentText() if hasattr(self, 'combo_template') else '-- select --'
        self.combo_template.blockSignals(True)
        self.combo_template.clear()
        self.combo_template.addItem('-- select --')
        for name in BUILTIN_TEMPLATES:
            self.combo_template.addItem(name)
        for name in sorted(self._custom_templates.keys()):
            self.combo_template.addItem(name)
        self.combo_template.blockSignals(False)
        idx = self.combo_template.findText(current)
        if idx >= 0:
            self.combo_template.setCurrentIndex(idx)

    # =========================================================================
    # Window close
    # =========================================================================

    def closeEvent(self, event) -> None:
        if self._exec_locked:
            self.stop_requested.emit()
        super().closeEvent(event)
