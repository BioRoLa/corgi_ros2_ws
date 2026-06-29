#!/usr/bin/env python3
"""
Sequence Model for Custom Command Sequence.

Data structures, reachability validation, and JSON serialisation for
multi-node position-based sequences.
"""
from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from enum import Enum

# ---------------------------------------------------------------------------
# Schema versions
# ---------------------------------------------------------------------------
SEQUENCE_VERSION = "1.0"
LIMIT_PROFILE_VERSION = "1.0"

LEGS = ('A', 'B', 'C', 'D')
JOINTS = ('theta', 'beta', 'gamma')


# ===========================================================================
# Domain model
# ===========================================================================

@dataclass
class JointTarget:
    """Joint position target – stored in **degrees**."""
    theta: float = 0.0
    beta: float = 0.0
    gamma: float = 0.0

    def copy(self) -> JointTarget:
        return JointTarget(theta=self.theta, beta=self.beta, gamma=self.gamma)


@dataclass
class SequenceNode:
    """One position-command node in a sequence."""
    name: str = "node"
    duration_sec: float = 1.0
    targets: dict = field(default_factory=lambda: {
        leg: JointTarget() for leg in LEGS
    })
    gains: dict = field(default_factory=lambda: {
        'leg_kp': 120.0,
        'leg_kd': 0.25,
        'gamma_kp': 150.0,
        'gamma_kd': 1.75,
    })
    notes: str = ""

    def copy(self) -> SequenceNode:
        return SequenceNode(
            name=self.name + "_copy",
            duration_sec=self.duration_sec,
            targets={leg: self.targets[leg].copy() for leg in LEGS},
            gains=dict(self.gains),
            notes=self.notes,
        )


@dataclass
class LimitEntry:
    """Per-joint angle and speed limits."""
    min_deg: float = -180.0
    max_deg: float = 180.0
    max_speed_deg_per_sec: float = 360.0


@dataclass
class LimitProfile:
    """User-defined joint limits for all 4 legs × 3 joints."""
    profile_name: str = "default"
    # key: (leg, joint) -> LimitEntry
    limits: dict = field(default_factory=lambda: {
        (leg, joint): LimitEntry() for leg in LEGS for joint in JOINTS
    })


@dataclass
class Sequence:
    """A named sequence of SequenceNode objects."""
    version: str = SEQUENCE_VERSION
    name: str = "my_sequence"
    notes: str = ""
    limit_profile_path: str | None = None
    nodes: list = field(default_factory=list)


class ExecutionState(Enum):
    IDLE = "idle"
    VALIDATING = "validating"
    RUNNING_NODE = "running_node"
    COMPLETED = "completed"
    FAILED = "failed"
    STOPPED = "stopped"


@dataclass
class ReachabilityError:
    node_idx: int
    node_name: str
    leg: str
    joint: str
    message: str


# ===========================================================================
# Built-in node templates (angles in degrees)
# ===========================================================================

def _make_template_node(name: str, duration: float,
                        theta: float, beta: float, gamma: float) -> SequenceNode:
    return SequenceNode(
        name=name,
        duration_sec=duration,
        targets={leg: JointTarget(theta=theta, beta=beta, gamma=gamma) for leg in LEGS},
    )


BUILTIN_TEMPLATES: dict[str, SequenceNode] = {
    "Stand":   _make_template_node("stand",  2.0, theta=90.0,  beta=0.0, gamma=0.0),
    "Squat":   _make_template_node("squat",  2.0, theta=45.0,  beta=0.0, gamma=0.0),
    "Low":     _make_template_node("low",    2.0, theta=17.5,  beta=0.0, gamma=0.0),
    "Neutral": _make_template_node("neutral", 2.0, theta=60.0, beta=0.0, gamma=0.0),
}


# ===========================================================================
# Reachability check
# ===========================================================================

def check_sequence_reachability(
    sequence: Sequence,
    limit_profile: LimitProfile | None,
    start_positions: dict | None,
) -> list[ReachabilityError]:
    """
    Validate every node against joint limits and speed reachability.

    Args:
        sequence:        The Sequence to validate.
        limit_profile:   Optional per-joint limits.
        start_positions: dict[leg][joint] = float (degrees) for the
                         robot's current pose, or None.

    Returns:
        List of ReachabilityError objects (empty list means OK).
    """
    errors: list[ReachabilityError] = []

    # Build virtual current positions (for chained reachability check)
    if start_positions is not None:
        cur = {
            leg: {j: float(start_positions[leg][j]) for j in JOINTS}
            for leg in LEGS
        }
    else:
        cur = {leg: {j: 0.0 for j in JOINTS} for leg in LEGS}

    for node_idx, node in enumerate(sequence.nodes):
        if node.duration_sec <= 0:
            errors.append(ReachabilityError(
                node_idx=node_idx, node_name=node.name,
                leg='*', joint='*',
                message=f"duration_sec must be > 0, got {node.duration_sec}",
            ))
            continue

        for leg in LEGS:
            for joint in JOINTS:
                target = getattr(node.targets[leg], joint)

                if limit_profile is not None:
                    entry = limit_profile.limits.get((leg, joint))
                    if entry is not None:
                        # Range check
                        if not (entry.min_deg <= target <= entry.max_deg):
                            errors.append(ReachabilityError(
                                node_idx=node_idx, node_name=node.name,
                                leg=leg, joint=joint,
                                message=(
                                    f"Target {target:.2f}° out of limits "
                                    f"[{entry.min_deg:.2f}, {entry.max_deg:.2f}]"
                                ),
                            ))
                            cur[leg][joint] = target  # still advance virtual pos
                            continue

                        # Speed reachability check
                        delta = abs(target - cur[leg][joint])
                        max_achievable = entry.max_speed_deg_per_sec * node.duration_sec
                        if delta > max_achievable:
                            errors.append(ReachabilityError(
                                node_idx=node_idx, node_name=node.name,
                                leg=leg, joint=joint,
                                message=(
                                    f"Requires {delta:.2f}° travel but max achievable is "
                                    f"{max_achievable:.2f}° "
                                    f"({entry.max_speed_deg_per_sec:.1f}°/s × {node.duration_sec:.2f}s)"
                                ),
                            ))

                # Advance virtual position regardless of errors
                cur[leg][joint] = target

    return errors


# ===========================================================================
# Serialisation – Sequence
# ===========================================================================

def _sequence_to_dict(seq: Sequence) -> dict:
    return {
        "version": seq.version,
        "name": seq.name,
        "notes": seq.notes,
        "limit_profile_path": seq.limit_profile_path,
        "nodes": [
            {
                "name": node.name,
                "duration_sec": node.duration_sec,
                "notes": node.notes,
                "gains": {
                    'leg_kp': float(node.gains.get('leg_kp', 120.0)),
                    'leg_kd': float(node.gains.get('leg_kd', 0.25)),
                    'gamma_kp': float(node.gains.get('gamma_kp', 150.0)),
                    'gamma_kd': float(node.gains.get('gamma_kd', 1.75)),
                },
                "targets": {
                    leg: {
                        "theta": node.targets[leg].theta,
                        "beta":  node.targets[leg].beta,
                        "gamma": node.targets[leg].gamma,
                    }
                    for leg in LEGS
                },
            }
            for node in seq.nodes
        ],
    }


def _extract_nodes_payload(d) -> tuple[dict, list]:
    """Accept legacy / auto-generated formats and return (meta_dict, nodes_list)."""
    if isinstance(d, list):
        # Legacy: top-level list of nodes
        return {
            'version': SEQUENCE_VERSION,
            'name': 'auto_generated_sequence',
            'notes': '',
            'limit_profile_path': None,
        }, d

    if not isinstance(d, dict):
        raise ValueError('Sequence JSON root must be object or list')

    # Canonical format
    if 'nodes' in d and isinstance(d['nodes'], list):
        meta = dict(d)
        meta.setdefault('version', SEQUENCE_VERSION)
        meta.setdefault('name', 'auto_generated_sequence')
        meta.setdefault('notes', '')
        meta.setdefault('limit_profile_path', None)
        return meta, d['nodes']

    # Common alternates
    for key in ('sequence', 'commands', 'steps', 'trajectory', 'data', 'items', 'waypoints', 'poses', 'motions'):
        if key in d and isinstance(d[key], list):
            meta = {
                'version': d.get('version', SEQUENCE_VERSION),
                'name': d.get('name', d.get('sequence_name', 'auto_generated_sequence')),
                'notes': d.get('notes', ''),
                'limit_profile_path': d.get('limit_profile_path'),
            }
            return meta, d[key]

    # Single-node object without nodes wrapper
    node_like_keys = (
        'targets', 'positions', 'pos', 'q', 'module_a',
        'duration_sec', 'duration', 'time', 'dt'
    )
    if any(k in d for k in node_like_keys) or any(f'{leg}_{joint}' in d for leg in LEGS for joint in JOINTS):
        meta = {
            'version': d.get('version', SEQUENCE_VERSION),
            'name': d.get('name', d.get('sequence_name', 'auto_generated_sequence')),
            'notes': d.get('notes', ''),
            'limit_profile_path': d.get('limit_profile_path'),
        }
        return meta, [d]

    # Heuristic: first list field containing dict nodes
    for k, v in d.items():
        if isinstance(v, list) and v and all(isinstance(x, dict) for x in v):
            meta = {
                'version': d.get('version', SEQUENCE_VERSION),
                'name': d.get('name', d.get('sequence_name', 'auto_generated_sequence')),
                'notes': d.get('notes', ''),
                'limit_profile_path': d.get('limit_profile_path'),
            }
            return meta, v

    raise ValueError(
        "Sequence load failed:\nMissing required field: 'nodes' (or recognized legacy keys)"
    )


def _parse_node_targets(nd: dict) -> dict:
    """
    Parse multiple node target formats into canonical dict[leg] -> JointTarget.
    """
    targets: dict = {leg: JointTarget() for leg in LEGS}

    def _to_float(x, default=0.0) -> float:
        try:
            return float(x)
        except Exception:
            return float(default)

    def _pick_joint_value(dct: dict, aliases: tuple[str, ...], default=0.0) -> float:
        for k in aliases:
            if k in dct:
                return _to_float(dct.get(k), default)
        return float(default)

    # Node itself can be a vector list: [Aθ, Aβ, Bθ, Bβ, Cθ, Cβ, Dθ, Dβ, Aγ, Bγ, Cγ, Dγ]
    if isinstance(nd, list) and len(nd) >= 12:
        targets['A'] = JointTarget(theta=_to_float(nd[0]), beta=_to_float(nd[1]), gamma=_to_float(nd[8]))
        targets['B'] = JointTarget(theta=_to_float(nd[2]), beta=_to_float(nd[3]), gamma=_to_float(nd[9]))
        targets['C'] = JointTarget(theta=_to_float(nd[4]), beta=_to_float(nd[5]), gamma=_to_float(nd[10]))
        targets['D'] = JointTarget(theta=_to_float(nd[6]), beta=_to_float(nd[7]), gamma=_to_float(nd[11]))
        return targets

    if not isinstance(nd, dict):
        raise ValueError('node must be object or 12-value list')

    # Canonical: targets: {A:{theta,beta,gamma}, ...}
    if isinstance(nd.get('targets'), dict):
        t_all = nd['targets']
        for leg in LEGS:
            t = t_all.get(leg, {}) if isinstance(t_all.get(leg, {}), dict) else {}
            targets[leg] = JointTarget(
                theta=float(t.get('theta', 0.0)),
                beta=float(t.get('beta', 0.0)),
                gamma=float(t.get('gamma', 0.0)),
            )
        return targets

    # Flat: A_theta, A_beta, ...
    has_flat = any(f'{leg}_{joint}' in nd for leg in LEGS for joint in JOINTS)
    if has_flat:
        for leg in LEGS:
            targets[leg] = JointTarget(
                theta=float(nd.get(f'{leg}_theta', 0.0)),
                beta=float(nd.get(f'{leg}_beta', 0.0)),
                gamma=float(nd.get(f'{leg}_gamma', 0.0)),
            )
        return targets

    # Vector format: positions/pos/q length >= 12
    vector = nd.get('positions', nd.get('pos', nd.get('q', nd.get('angles', nd.get('joint_positions', nd.get('joint_targets', nd.get('target')))))))
    if isinstance(vector, list) and len(vector) >= 12:
        # [Aθ, Aβ, Bθ, Bβ, Cθ, Cβ, Dθ, Dβ, Aγ, Bγ, Cγ, Dγ]
        targets['A'] = JointTarget(theta=float(vector[0]), beta=float(vector[1]), gamma=float(vector[8]))
        targets['B'] = JointTarget(theta=float(vector[2]), beta=float(vector[3]), gamma=float(vector[9]))
        targets['C'] = JointTarget(theta=float(vector[4]), beta=float(vector[5]), gamma=float(vector[10]))
        targets['D'] = JointTarget(theta=float(vector[6]), beta=float(vector[7]), gamma=float(vector[11]))
        return targets

    # Dict vector format: {'0': ..., '1': ... '11': ...}
    if isinstance(vector, dict):
        keys_012 = [str(i) for i in range(12)]
        if all(k in vector for k in keys_012):
            vals = [_to_float(vector[str(i)]) for i in range(12)]
            targets['A'] = JointTarget(theta=vals[0], beta=vals[1], gamma=vals[8])
            targets['B'] = JointTarget(theta=vals[2], beta=vals[3], gamma=vals[9])
            targets['C'] = JointTarget(theta=vals[4], beta=vals[5], gamma=vals[10])
            targets['D'] = JointTarget(theta=vals[6], beta=vals[7], gamma=vals[11])
            return targets

    # Module format: module_a/module_b...
    mod_map = {'A': 'module_a', 'B': 'module_b', 'C': 'module_c', 'D': 'module_d'}
    if any(k in nd for k in mod_map.values()):
        for leg, key in mod_map.items():
            m = nd.get(key, {})
            if isinstance(m, dict):
                targets[leg] = JointTarget(
                    theta=_pick_joint_value(m, ('theta', 'r', 'joint_r', 'hip')),
                    beta=_pick_joint_value(m, ('beta', 'l', 'joint_l', 'knee')),
                    gamma=_pick_joint_value(m, ('gamma', 'h', 'joint_h', 'abad')),
                )
        return targets

    # Leg object format: {'A': {...}, 'B': {...}, ...} or lowercase keys
    leg_keys_present = any(k in nd for k in ('A', 'B', 'C', 'D', 'a', 'b', 'c', 'd', 'leg_a', 'leg_b', 'leg_c', 'leg_d'))
    if leg_keys_present:
        leg_aliases = {
            'A': ('A', 'a', 'leg_a'),
            'B': ('B', 'b', 'leg_b'),
            'C': ('C', 'c', 'leg_c'),
            'D': ('D', 'd', 'leg_d'),
        }
        for leg in LEGS:
            leg_obj = None
            for alias in leg_aliases[leg]:
                if alias in nd:
                    leg_obj = nd[alias]
                    break
            if isinstance(leg_obj, dict):
                targets[leg] = JointTarget(
                    theta=_pick_joint_value(leg_obj, ('theta', 'r', 'joint_r', 'hip', 'q0', 'q_r')),
                    beta=_pick_joint_value(leg_obj, ('beta', 'l', 'joint_l', 'knee', 'q1', 'q_l')),
                    gamma=_pick_joint_value(leg_obj, ('gamma', 'h', 'joint_h', 'abad', 'q2', 'q_h')),
                )
            elif isinstance(leg_obj, list) and len(leg_obj) >= 3:
                targets[leg] = JointTarget(
                    theta=_to_float(leg_obj[0]),
                    beta=_to_float(leg_obj[1]),
                    gamma=_to_float(leg_obj[2]),
                )
        return targets

    # Flat aliases: A_r / A_l / A_h, etc.
    has_flat_alt = any(
        (f'{leg}_r' in nd or f'{leg}_l' in nd or f'{leg}_h' in nd or
         f'{leg.lower()}_r' in nd or f'{leg.lower()}_l' in nd or f'{leg.lower()}_h' in nd)
        for leg in LEGS
    )
    if has_flat_alt:
        for leg in LEGS:
            u = leg
            l = leg.lower()
            targets[leg] = JointTarget(
                theta=_to_float(nd.get(f'{u}_r', nd.get(f'{l}_r', 0.0))),
                beta=_to_float(nd.get(f'{u}_l', nd.get(f'{l}_l', 0.0))),
                gamma=_to_float(nd.get(f'{u}_h', nd.get(f'{l}_h', 0.0))),
            )
        return targets

    raise ValueError(
        "Unsupported node target format; supported: targets/A_theta/positions(or angles)/module_a/leg objects/list node"
    )


def _sequence_from_dict(d: dict) -> Sequence:
    errors: list[str] = []
    meta, nodes_payload = _extract_nodes_payload(d)

    if str(meta.get('version', SEQUENCE_VERSION)) != SEQUENCE_VERSION:
        raise ValueError(
            f"Unsupported sequence version '{meta.get('version')}' (expected '{SEQUENCE_VERSION}')"
        )

    nodes: list[SequenceNode] = []
    for i, nd in enumerate(nodes_payload):
        targets = None
        try:
            targets = _parse_node_targets(nd)
        except Exception as e:
            # Allow loading execution-record style items as editable placeholders.
            if isinstance(nd, dict) and (
                'node_name' in nd or 'duration_planned_sec' in nd
            ):
                targets = {leg: JointTarget() for leg in LEGS}
            else:
                errors.append(f"Node[{i}] target parse failed: {e}")
                continue

        if isinstance(nd, dict):
            duration = nd.get('duration_sec', nd.get('duration', nd.get('time', nd.get('dt', 1.0))))
            duration = nd.get('duration_planned_sec', duration)
            gains = nd.get('gains', {}) if isinstance(nd.get('gains', {}), dict) else {}
            node_name = str(nd.get("name", nd.get('node_name', f"node_{i}")))
            node_notes = str(nd.get("notes", ""))
            if 'target parse failed' not in node_notes and (
                'node_name' in nd or 'duration_planned_sec' in nd
            ):
                if not node_notes:
                    node_notes = 'Imported from execution record; targets need manual assignment.'
            leg_kp_default = nd.get('leg_kp', 120.0)
            leg_kd_default = nd.get('leg_kd', 0.25)
            gamma_kp_default = nd.get('gamma_kp', 150.0)
            gamma_kd_default = nd.get('gamma_kd', 1.75)
        else:
            duration = 1.0
            gains = {}
            node_name = f"node_{i}"
            node_notes = ""
            leg_kp_default = 120.0
            leg_kd_default = 0.25
            gamma_kp_default = 150.0
            gamma_kd_default = 1.75

        nodes.append(SequenceNode(
            name=node_name,
            duration_sec=float(duration),
            targets=targets,
            gains={
                'leg_kp': float(gains.get('leg_kp', leg_kp_default)),
                'leg_kd': float(gains.get('leg_kd', leg_kd_default)),
                'gamma_kp': float(gains.get('gamma_kp', gamma_kp_default)),
                'gamma_kd': float(gains.get('gamma_kd', gamma_kd_default)),
            },
            notes=node_notes,
        ))

    if errors:
        raise ValueError("Sequence load failed:\n" + "\n".join(errors))

    return Sequence(
        version=str(meta.get("version", SEQUENCE_VERSION)),
        name=str(meta.get("name", "unnamed")),
        notes=str(meta.get("notes", "")),
        limit_profile_path=meta.get("limit_profile_path"),
        nodes=nodes,
    )


def save_sequence(path: str, seq: Sequence) -> None:
    """Save *seq* to a JSON file at *path*."""
    dirpath = os.path.dirname(os.path.abspath(path))
    if dirpath:
        os.makedirs(dirpath, exist_ok=True)
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(_sequence_to_dict(seq), f, indent=2, ensure_ascii=False)


def load_sequence(path: str, auto_load_profile: bool = True) -> Sequence:
    """Load a Sequence from a JSON file.  Raises ValueError or OSError.
    
    If auto_load_profile=True, automatically looks for a corresponding profile file:
      my_seq.json → my_seq_profile.json
    """
    with open(path, 'r', encoding='utf-8') as f:
        d = json.load(f)
    seq = _sequence_from_dict(d)
    
    # Auto-load profile if it exists
    if auto_load_profile:
        base_path = path.rsplit('.json', 1)[0] if path.endswith('.json') else path
        profile_path = base_path + '_profile.json'
        if os.path.exists(profile_path):
            try:
                # Don't fail if profile loading fails
                load_limit_profile(profile_path)
                seq.limit_profile_path = profile_path
            except Exception:
                pass  # Silently ignore profile load errors
    
    return seq


# ===========================================================================
# Serialisation – LimitProfile
# ===========================================================================

def _limit_profile_to_dict(p: LimitProfile) -> dict:
    joints_dict: dict = {}
    for leg in LEGS:
        joints_dict[leg] = {}
        for joint in JOINTS:
            entry = p.limits.get((leg, joint), LimitEntry())
            joints_dict[leg][joint] = {
                "min":       entry.min_deg,
                "max":       entry.max_deg,
                "max_speed": entry.max_speed_deg_per_sec,
            }
    return {
        "version":      LIMIT_PROFILE_VERSION,
        "profile_name": p.profile_name,
        "joints":       joints_dict,
    }


def _limit_profile_from_dict(d: dict) -> LimitProfile:
    errors: list[str] = []
    for req in ("version", "profile_name", "joints"):
        if req not in d:
            errors.append(f"Missing required field: '{req}'")
    if errors:
        raise ValueError("Limit profile load failed:\n" + "\n".join(errors))

    if d["version"] != LIMIT_PROFILE_VERSION:
        raise ValueError(
            f"Unsupported limit profile version '{d['version']}'"
        )

    limits: dict = {}
    for leg in LEGS:
        if leg not in d["joints"]:
            errors.append(f"Missing leg '{leg}' in joints")
            continue
        for joint in JOINTS:
            if joint not in d["joints"][leg]:
                errors.append(f"Missing joint '{joint}' for leg '{leg}'")
                limits[(leg, joint)] = LimitEntry()
                continue
            j = d["joints"][leg][joint]
            for k in ("min", "max", "max_speed"):
                if k not in j:
                    errors.append(f"Missing '{k}' for leg '{leg}' joint '{joint}'")
            limits[(leg, joint)] = LimitEntry(
                min_deg=float(j.get("min", -180.0)),
                max_deg=float(j.get("max", 180.0)),
                max_speed_deg_per_sec=float(j.get("max_speed", 360.0)),
            )

    if errors:
        raise ValueError("Limit profile load failed:\n" + "\n".join(errors))

    return LimitProfile(profile_name=str(d["profile_name"]), limits=limits)


def save_limit_profile(path: str, profile: LimitProfile) -> None:
    dirpath = os.path.dirname(os.path.abspath(path))
    if dirpath:
        os.makedirs(dirpath, exist_ok=True)
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(_limit_profile_to_dict(profile), f, indent=2, ensure_ascii=False)


def load_limit_profile(path: str) -> LimitProfile:
    """Load a LimitProfile from a JSON file.  Raises ValueError or OSError."""
    with open(path, 'r', encoding='utf-8') as f:
        d = json.load(f)
    return _limit_profile_from_dict(d)


# ===========================================================================
# Execution record
# ===========================================================================

@dataclass
class NodeExecutionRecord:
    node_idx: int
    node_name: str
    started_at_wall: str
    finished_at_wall: str | None = None
    duration_planned_sec: float = 0.0
    duration_actual_sec: float = 0.0
    result: str = "running"   # "success" | "stopped" | "failed"
    message: str = ""


def save_execution_record(
    path: str,
    sequence_name: str,
    records: list[NodeExecutionRecord],
) -> None:
    """Write an execution record JSON file to *path*."""
    data = {
        "sequence_name": sequence_name,
        "records": [
            {
                "node_idx":             r.node_idx,
                "node_name":            r.node_name,
                "started_at":           r.started_at_wall,
                "finished_at":          r.finished_at_wall,
                "duration_planned_sec": r.duration_planned_sec,
                "duration_actual_sec":  r.duration_actual_sec,
                "result":               r.result,
                "message":              r.message,
            }
            for r in records
        ],
    }
    dirpath = os.path.dirname(os.path.abspath(path))
    if dirpath:
        os.makedirs(dirpath, exist_ok=True)
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
