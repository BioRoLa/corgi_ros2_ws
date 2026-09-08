#!/usr/bin/env python3
"""reg_dump.py -- READ-ONLY flash-register dump of the Corgi motors over the
shipped config path (ROS config/command -> corgi_ros_bridge -> gRPC
motor/config/request -> fpga_driver handleConfigMode -> CAN READ frame).

This script can never send a register WRITE: the only ConfigMode it knows is
READ (0); it is asserted on every message before publish, and ConfigMode WRITE
is not referenced anywhere in this file. It never publishes a robot-mode
command either -- the FSM is driven from the control panel by the operator.

Gate (MEASURED): the driver dispatches handleConfigMode ONLY while its motor
FSM is FunctionMode::CONFIG (motor_fsm.cpp:45-66), so a request sent in any
other mode is silently dropped and shows here as a timeout -- the path is
self-gating. The published `motor_mode` field is NOT a check: the driver never
calls set_motor_mode, so it is always 0/REST on this branch (verified by grep
over fpga_driver/src). We therefore gate on robot/state robot_mode == 4
(MOTORCONFIG, really published by robot_fsm) plus a live probe read.

Sub-commands (run on the Orin with the ROS env sourced):
  status                     print robot_mode / power switches / stream counts
  probe                      one READ (A:R KP_MAX) to prove the path
  dump    [--targets ...]    read every register of each target

Sources: corgi_msgs/msg/ConfigStamped.msg, corgi_panel constants.py (enums),
Config.proto (MOTOR_H = 2), fpga_driver mode.hpp, motor_fsm.cpp:411-495,
motorcontrol Core/Inc/user_config.h (address names), and the Orin's
config_panel_20260719_2041*.log (known answers for module A, motors R and L).
"""
import argparse
import json
import os
import sys
import time

import rclpy
from corgi_msgs.msg import (ConfigStamped, MotorStateStamped,
                            PowerStateStamped, RobotStateStamped)

ROBOT_MODE = {0: 'SYSTEM_ON', 1: 'INIT', 2: 'IDLE', 3: 'STANDBY', 4: 'MOTORCONFIG'}
MODULE = {'A': 0, 'B': 1, 'C': 2, 'D': 3}
MOTOR = {'R': 0, 'L': 1, 'H': 2}          # Config.proto: MOTOR_R 0, MOTOR_L 1, MOTOR_H 2
CONFIG_MODE_READ = 0                       # the ONLY mode this script uses
TYPE_INT, TYPE_FLOAT = 0, 1
ERR_NAMES = {0: 'CODE_CONFIG_SUCCESS', 5: 'invalid module index',
             6: 'invalid motor index', 7: 'duplicate seq'}

FLOAT_NAMES = {2: 'I_BW', 3: 'I_MAX', 4: 'THETA_MIN', 5: 'THETA_MAX', 6: 'I_FW_MAX',
               7: 'R_NOMINAL', 8: 'TEMP_MAX', 9: 'I_MAX_CONT', 10: 'PPAIRS', 13: 'R_PHASE',
               14: 'KT', 15: 'R_TH', 16: 'C_TH', 17: 'GR', 18: 'I_CAL', 19: 'P_MIN',
               20: 'P_MAX', 21: 'V_MIN', 22: 'V_MAX', 23: 'T_MIN', 24: 'T_MAX',
               25: 'KP_MAX', 26: 'KI_MAX', 27: 'KD_MAX', 28: 'HALL_CAL_OFFSET',
               29: 'HALL_CAL_SPEED', 30: 'HALL_CAL_KP', 31: 'HALL_CAL_KI', 32: 'HALL_CAL_KD',
               33: 'MOTOR_MODE_KP', 34: 'MOTOR_MODE_KI', 35: 'MOTOR_MODE_KD',
               36: 'ABAD_CAL_OFFSET', 37: 'ABAD_CAL_SPEED', 38: 'ABAD_CAL_KP',
               39: 'ABAD_CAL_KI', 40: 'ABAD_CAL_KD'}
INT_NAMES = {1: 'CAN_ID'}
INT_ADDRS = list(range(0, 8))
FLOAT_ADDRS = list(range(0, 41))
HEADLINE = [3, 14, 17, 24, 25, 27]        # I_MAX KT GR T_MAX KP_MAX KD_MAX

# known answers: module A motors R and L, config panel 2026-07-19 20:41 (Orin log_file)
KNOWN_JULY_A_RL = {2: 1000.0, 3: 40.0, 8: 125.0, 9: 14.0, 10: 21.0, 14: 0.08, 17: 6.0,
                   18: 5.0, 20: 6.28, 21: -45.0, 22: 45.0, 23: -20.0, 24: 20.0,
                   25: 500.0, 26: 0.0, 27: 5.0}
# registered prediction P-REG-1 (log 325.46): every H board KP_MAX 500 / KD_MAX 5
P_REG_1 = {25: 500.0, 27: 5.0}
DEFAULT_TARGETS = 'A:R,A:H,B:H,C:H,D:H,A:L,B:R,B:L,C:R,C:L,D:R,D:L'


class Client:
    def __init__(self):
        self.node = rclpy.create_node('reg_dump_readonly')
        self.cfg_pub = self.node.create_publisher(ConfigStamped, 'config/command', 10)
        self.replies = {}
        self.robot = self.motor = self.power = None
        self.n_cfg = self.n_robot = self.n_motor = self.n_power = 0
        self.node.create_subscription(ConfigStamped, 'config/state', self._cfg_cb, 50)
        self.node.create_subscription(RobotStateStamped, 'robot/state', self._robot_cb, 10)
        self.node.create_subscription(MotorStateStamped, 'motor/state', self._motor_cb, 10)
        self.node.create_subscription(PowerStateStamped, 'power/state', self._power_cb, 10)
        self.seq = 40000 + int(time.time()) % 20000   # far from the panel's own 1..N counter

    def _cfg_cb(self, m):
        self.n_cfg += 1
        self.replies[m.header.seq] = m

    def _robot_cb(self, m):
        self.n_robot += 1
        self.robot = m

    def _motor_cb(self, m):
        self.n_motor += 1
        self.motor = m

    def _power_cb(self, m):
        self.n_power += 1
        self.power = m

    def spin(self, seconds):
        t_end = time.time() + seconds
        while time.time() < t_end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def status_line(self):
        rm = self.robot.robot_mode if self.robot else None
        pw = self.power
        sw = ('PB1 d/s/p=%d%d%d PB2 d/s/p=%d%d%d'
              % (pw.pb1_digital, pw.pb1_signal, pw.pb1_power,
                 pw.pb2_digital, pw.pb2_signal, pw.pb2_power)) if pw else 'no power/state'
        return ('robot_mode=%s(%s)  %s  [msgs robot %d motor %d power %d config %d]'
                % (rm, ROBOT_MODE.get(rm, '?'), sw,
                   self.n_robot, self.n_motor, self.n_power, self.n_cfg))

    def in_motorconfig(self):
        return self.robot is not None and self.robot.robot_mode == 4

    def read(self, module, motor, c_type, addr, timeout=1.5):
        """One READ request; returns (value, error_code) or (None, 'timeout')."""
        self.seq = (self.seq + 1) % 65535
        msg = ConfigStamped()
        msg.header.seq = self.seq
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = ''
        msg.transmit = True
        msg.module = int(module)
        msg.motor = int(motor)
        msg.mode = CONFIG_MODE_READ
        msg.type = int(c_type)
        msg.address = int(addr)
        msg.value_f = 0.0
        msg.value_i = 0
        msg.error_code = 0
        assert msg.mode == CONFIG_MODE_READ == 0, 'refusing to publish anything but READ'
        self.cfg_pub.publish(msg)
        t_end = time.time() + timeout
        while time.time() < t_end:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            r = self.replies.pop(self.seq, None)
            if r is not None:
                val = r.value_i if r.type == TYPE_INT else r.value_f
                return val, int(r.error_code)
        return None, 'timeout'

    def read_retry(self, module, motor, c_type, addr, tries=2):
        for _ in range(tries):
            val, err = self.read(module, motor, c_type, addr)
            if val is not None:
                return val, err
        return None, 'timeout'


def cmd_status(args):
    c = Client()
    c.spin(args.seconds)
    print(c.status_line())
    return 0


def cmd_probe(args):
    c = Client()
    c.spin(3.0)
    print('precheck:', c.status_line())
    if not c.in_motorconfig():
        print('REFUSED: robot/state robot_mode must be 4 (MOTORCONFIG). Nothing was sent.')
        return 2
    val, err = c.read_retry(MODULE['A'], MOTOR['R'], TYPE_FLOAT, 25)
    print('probe A:R FLOAT 25 (KP_MAX) -> value=%s error=%s' % (val, err))
    if val is None:
        print('No reply. The driver dispatches config requests only while its motor FSM is '
              'CONFIG; nothing was written. Check the driver terminal on the sbRIO.')
        return 3
    return 0


def cmd_dump(args):
    c = Client()
    c.spin(3.0)
    print('precheck:', c.status_line())
    if not c.in_motorconfig():
        print('REFUSED: robot/state robot_mode must be 4 (MOTORCONFIG). Nothing was sent.')
        return 2
    targets = [t.strip().upper() for t in args.targets.split(',') if t.strip()]
    stamp = time.strftime('%Y%m%d_%H%M%S')
    out_dir = os.path.expanduser('~/corgi_ws/corgi_ros2_ws/log_file')
    os.makedirs(out_dir, exist_ok=True)
    result = {'stamp': stamp, 'host': os.uname().nodename, 'precheck': c.status_line(),
              'mode_used': 'READ only (ConfigMode 0)', 'targets': {}, 'checks': {}}
    t0 = time.time()
    for tgt in targets:
        mod_s, mot_s = tgt.split(':')
        mod, mot = MODULE[mod_s], MOTOR[mot_s]
        # probe first: a motor that does not answer costs 3 s, not 200
        pv, pe = c.read_retry(mod, mot, TYPE_FLOAT, 25)
        if pv is None:
            print('%s  NO REPLY (probe timed out twice) -- skipped' % tgt)
            result['targets'][tgt] = {'INT': {}, 'FLOAT': {}, 'errors': {'probe': 'timeout'}}
            sys.stdout.flush()
            continue
        regs = {'INT': {}, 'FLOAT': {}, 'errors': {}}
        n_timeout = 0
        for c_type, addrs, key in ((TYPE_INT, INT_ADDRS, 'INT'),
                                   (TYPE_FLOAT, FLOAT_ADDRS, 'FLOAT')):
            for a in addrs:
                val, err = c.read_retry(mod, mot, c_type, a)
                if val is None:
                    n_timeout += 1
                    regs['errors']['%s%d' % (key, a)] = 'timeout'
                    continue
                regs[key][str(a)] = val
                if err != 0:
                    regs['errors']['%s%d' % (key, a)] = ERR_NAMES.get(err, 'error %s' % err)
                time.sleep(0.02)
        result['targets'][tgt] = regs
        fl = regs['FLOAT']
        head = '  '.join('%s=%s' % (FLOAT_NAMES[a], fl.get(str(a), 'NA')) for a in HEADLINE)
        print('%s  CAN_ID=%s  %s  (timeouts %d, errors %d)'
              % (tgt, regs['INT'].get('1', 'NA'), head, n_timeout, len(regs['errors'])))
        sys.stdout.flush()
    result['elapsed_s'] = round(time.time() - t0, 1)

    checks = {}
    for tgt in ('A:R', 'A:L'):
        fl = result['targets'].get(tgt, {}).get('FLOAT', {})
        if fl:
            bad = {FLOAT_NAMES.get(a, a): (fl.get(str(a)), v) for a, v in KNOWN_JULY_A_RL.items()
                   if str(a) in fl and abs(float(fl[str(a)]) - v) > 1e-2 * max(1.0, abs(v))}
            checks['july_known_answers_' + tgt] = 'PASS' if not bad else 'FAIL %s' % bad
    for tgt, regs in result['targets'].items():
        if tgt.endswith(':H') and regs['FLOAT']:
            fl = regs['FLOAT']
            got = {FLOAT_NAMES[a]: fl.get(str(a)) for a in P_REG_1}
            ok = all(str(a) in fl and abs(float(fl[str(a)]) - v) < 1e-3 for a, v in P_REG_1.items())
            checks['P-REG-1_' + tgt] = ('PASS' if ok else 'FAIL') + ' %s' % got
    result['checks'] = checks
    for k, v in sorted(checks.items()):
        print('CHECK', k, v)

    jp = os.path.join(out_dir, 'reg_dump_%s.json' % stamp)
    tp = os.path.join(out_dir, 'reg_dump_%s.txt' % stamp)
    with open(jp, 'w') as f:
        json.dump(result, f, indent=1, sort_keys=True)
    with open(tp, 'w') as f:
        f.write('reg_dump %s on %s -- READ only\n%s\n\n'
                % (stamp, result['host'], result['precheck']))
        for tgt, regs in result['targets'].items():
            f.write('== %s ==\n' % tgt)
            for a in INT_ADDRS:
                if str(a) in regs['INT']:
                    f.write('  INT   %2d %-16s %s\n' % (a, INT_NAMES.get(a, ''), regs['INT'][str(a)]))
            for a in FLOAT_ADDRS:
                if str(a) in regs['FLOAT']:
                    f.write('  FLOAT %2d %-16s %s\n' % (a, FLOAT_NAMES.get(a, ''), regs['FLOAT'][str(a)]))
            if regs['errors']:
                f.write('  errors: %s\n' % regs['errors'])
        f.write('\nchecks:\n')
        for k, v in sorted(checks.items()):
            f.write('  %s: %s\n' % (k, v))
    print('wrote', jp)
    print('wrote', tp)
    print('post:', c.status_line())
    return 0


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest='cmd', required=True)
    s = sub.add_parser('status')
    s.add_argument('--seconds', type=float, default=3.0)
    sub.add_parser('probe')
    d = sub.add_parser('dump')
    d.add_argument('--targets', default=DEFAULT_TARGETS)
    args = ap.parse_args()
    rclpy.init()
    try:
        return {'status': cmd_status, 'probe': cmd_probe, 'dump': cmd_dump}[args.cmd](args)
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
