#!/usr/bin/env python3
"""Delivered ABAD gain from the bagged command -- the #36 wrap made a number (log 325.46 §1).

The driver packs kp_h 12-bit over [0, 500] with no saturation (fpga_driver
can_motor.cpp:41, float_to_uint :147-152, packing :48-49), the firmware decodes
the same 12 bits against its flash register KP_MAX (motorcontrol can.c:263,
user_config.h:277; boot default 500). So the gain the motor actually runs is a
deterministic map of the logged /motor/command kp_h:

    code      = int(kp * 4095 / 500) & 0xFFF
    delivered = code * KP_MAX_fw / 4095            (= kp - 500.122 for 500.122 < kp <= 1000.244 at KP_MAX_fw 500)

The driver also multiplies every REPORTED torque by the per-motor kt_ (motor_fsm.cpp:758-760,
config.yaml Motor_H KT 2.148) while passing kp raw (:400-402), so a static hold's
torque-per-deflection readback in ROS units is kt_ * delivered / kp_cmd.

Per bag this prints and stores:
  * hold window [T0-3.0, T0-0.2] s: per leg kp/kd command, delivered firmware gain,
    the readback ratio |mean(tau_h - tau_h_cmd)| / |mean(gamma_state - gamma_cmd)| / kp_cmd
    and its prediction kt_ * (delivered/kp_cmd)  (P-WRAP-1);
  * gait window [T0+2.6, TOFF]: time-weighted fraction of rows over the 500.122 wrap,
    delivered kp_h min/median/max split by gain set (stance = module_a.kp_r < 100),
    and the command band itself (P-WRAP-2);
  * --selftest: the arithmetic known answers and the probe-3 hold of 2026-09-03
    from the cached arrays ~/.cache/wf_verify/*.npz (P-WRAP-3).

Usage:
    wrap_delivered.py [--kp-max-fw 500] [--kt 2.148] [--out wrap_delivered.json] bag.db3 [bag.db3 ...]
    wrap_delivered.py --selftest
Needs ROS sourced for bags (rclpy + corgi_msgs). Read-only on every input.
"""
import sys, os, json, sqlite3, argparse
import numpy as np

KP_MAX_DRV, KD_MAX_DRV = 500.0, 5.0
WRAP_KP = 4096.0 * KP_MAX_DRV / 4095.0          # 500.122
WRAP_KD = 4096.0 * KD_MAX_DRV / 4095.0          # 5.0012
L4 = "abcd"
NM = {"a": "A FL", "b": "B FR", "c": "C RR", "d": "D RL"}


def code12(x, xmax):
    # can_motor.cpp:147-152 (truncating cast, no clamp) + :48-49 (keeps bits 11..0)
    return int(x * 4095.0 / xmax) & 0xFFF


def delivered_kp(kp, kp_max_fw=500.0):
    return code12(kp, KP_MAX_DRV) * kp_max_fw / 4095.0


def delivered_kd(kd, kd_max_fw=5.0):
    return code12(kd, KD_MAX_DRV) * kd_max_fw / 4095.0


vkp = np.vectorize(delivered_kp, otypes=[float])
vkd = np.vectorize(delivered_kd, otypes=[float])


# ----------------------------------------------------------------------------- self-test
def selftest(kt):
    ok = True
    known = [(667.0, 166.79), (500.12, 500.0), (500.13, 0.0), (745.0, 244.81), (646.0, 145.79), (700.0, 199.88)]
    print("arithmetic (KP_MAX_fw 500):")
    for kp, want in known:
        got = delivered_kp(kp)
        flag = "ok" if abs(got - want) < 0.02 else "MISMATCH"
        ok &= flag == "ok"
        print("   kp %8.2f -> %8.2f  (expected %8.2f) %s" % (kp, got, want, flag))
    for kd, want in [(10.3, 0.297), (8.9, 3.90), (4.9, 4.90), (0.8, 0.80)]:
        got = delivered_kd(kd)
        flag = "ok" if abs(got - want) < 0.01 else "MISMATCH"
        ok &= flag == "ok"
        print("   kd %8.2f -> %8.3f  (expected %8.3f) %s" % (kd, got, want, flag))
    # the probe-3 hold of 2026-09-03 (bag ladder_ramp1_rec), F6 estimator of the verification pass
    D = os.path.expanduser("~/.cache/wf_verify/")
    if not os.path.exists(D + "motor_command.npz"):
        print("probe-3 cache not found at %s -- skipping that half of the self-test" % D)
        return ok
    cmd = np.load(D + "motor_command.npz"); ms = np.load(D + "motor_state.npz")
    tc = cmd["t_rel"]; tm = ms["t_rel"]
    R2D = 180.0 / np.pi
    want = {"a": 0.391, "b": 0.529, "c": 0.446, "d": 0.466}
    print("probe-3 pre-trigger hold 3-6 s (known answer of the 2026-09-03 verification pass):")
    m = (tm > 3) & (tm < 6); mc = (tc > 3) & (tc < 6)
    for l in L4:
        ga_i = np.interp(tm, tc, cmd["module_%s_gamma" % l])
        e = np.mean(ms["module_%s_gamma" % l][m] - ga_i[m])                  # rad
        tq = np.mean(ms["module_%s_torque_h" % l][m])
        kpc = float(np.median(cmd["module_%s_kp_h" % l][mc]))
        ratio = abs(tq / e) / kpc
        pred = kt * delivered_kp(kpc) / kpc
        flag = "ok" if abs(ratio - want[l]) < 0.005 else "MISMATCH"
        ok &= flag == "ok"
        print("   %s err %+.3f deg  tau_h %+.3f N.m  kp_cmd %.1f  ratio %.3f (expected %.3f) %s | predicted kt*(1-500.122/kp) = %.3f"
              % (NM[l], e * R2D, tq, kpc, ratio, want[l], flag, pred))
    print("SELFTEST", "PASS" if ok else "FAIL")
    return ok


# ----------------------------------------------------------------------------- bag
def read_bag(DB):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    con = sqlite3.connect("file:%s?mode=ro" % DB, uri=True)
    T = {n: (i, get_message(t)) for i, n, t in con.execute("SELECT id,name,type FROM topics")}

    def rd(n):
        i, M = T[n]
        return [(ts * 1e-9, deserialize_message(d, M))
                for ts, d in con.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp", (i,))]

    trig = rd("/trigger"); pairs = []; on = None
    for ts, m in trig:
        if m.enable and on is None:
            on = ts
        elif not m.enable and on is not None:
            pairs.append((on, ts)); on = None
    if not pairs:
        return None
    T0, TOFF = max(pairs, key=lambda p: p[1] - p[0])
    ms = rd("/motor/state"); mc = rd("/motor/command")
    out = dict(T0=T0, TOFF=TOFF, gait=TOFF - T0,
               ts=np.array([t - T0 for t, _ in ms]), tc=np.array([t - T0 for t, _ in mc]))
    for l in L4:
        out["g_s_" + l] = np.array([getattr(m, "module_" + l).gamma for _, m in ms])
        out["tq_s_" + l] = np.array([getattr(m, "module_" + l).torque_h for _, m in ms])
        out["g_c_" + l] = np.array([getattr(m, "module_" + l).gamma for _, m in mc])
        out["tq_c_" + l] = np.array([getattr(m, "module_" + l).torque_h for _, m in mc])
        out["kp_c_" + l] = np.array([getattr(m, "module_" + l).kp_h for _, m in mc])
        out["kd_c_" + l] = np.array([getattr(m, "module_" + l).kd_h for _, m in mc])
    out["kpr_a"] = np.array([m.module_a.kp_r for _, m in mc])
    return out


def analyse(DB, kp_max_fw, kt):
    b = read_bag(DB)
    name = os.path.basename(DB).replace("_0.db3", "")
    if b is None:
        print("%s: no trigger cycle" % name); return None
    R2D = 180.0 / np.pi
    res = dict(bag=name, gait_s=float(b["gait"]), kp_max_fw=kp_max_fw, kt=kt, hold={}, gait={})
    # ---- hold [T0-3.0, T0-0.2]
    hs = (b["ts"] > -3.0) & (b["ts"] < -0.2); hc = (b["tc"] > -3.0) & (b["tc"] < -0.2)
    print("\n================ %s  (gait %.1f s)" % (name, b["gait"]))
    print("HOLD [T0-3.0, T0-0.2] s: readback ratio |tau|/|e| / kp_cmd vs prediction kt*(delivered/kp_cmd)")
    for l in L4:
        if hs.sum() < 50 or hc.sum() < 50:
            res["hold"][l] = dict(unresolvable="no hold samples"); print("   %s no hold samples" % NM[l]); continue
        ga_i = np.interp(b["ts"], b["tc"], b["g_c_" + l])
        e = float(np.mean(b["g_s_" + l][hs] - ga_i[hs]))
        tq = float(np.mean(b["tq_s_" + l][hs]) - np.mean(b["tq_c_" + l][hc]))
        kpc = float(np.median(b["kp_c_" + l][hc])); kdc = float(np.median(b["kd_c_" + l][hc]))
        dkp = delivered_kp(kpc, kp_max_fw); dkd = delivered_kd(kdc)
        pred = kt * dkp / kpc if kpc > 0 else float("nan")
        row = dict(err_deg=e * R2D, tau_nm=tq, kp_cmd=kpc, kd_cmd=kdc, delivered_kp_fw=dkp, delivered_kd_fw=dkd,
                   predicted_ratio=pred)
        if abs(e * R2D) < 0.25:
            row["unresolvable"] = "|e| < 0.25 deg (1.5 LSB)"; row["ratio"] = None
            print("   %s err %+.3f deg (UNRESOLVABLE, < 1.5 LSB)  kp_cmd %.1f -> fw %.1f  pred %.3f"
                  % (NM[l], e * R2D, kpc, dkp, pred))
        else:
            ratio = abs(tq / e) / kpc; row["ratio"] = ratio
            print("   %s err %+.3f deg  tau %+.3f N.m  kp_cmd %.1f -> fw %.1f (kd %.2f -> %.2f)  ratio %.3f  pred %.3f  (%+.0f %%)"
                  % (NM[l], e * R2D, tq, kpc, dkp, kdc, dkd, ratio, pred, 100 * (ratio / pred - 1) if pred else float("nan")))
        res["hold"][l] = row
    # ---- gait [T0+2.6, TOFF]
    wc = (b["tc"] > 2.6) & (b["tc"] < b["gait"])
    tcw = b["tc"][wc]
    dt = np.diff(np.append(tcw, tcw[-1] + 0.001)); dt = np.clip(dt, 0, 0.05)     # time weights, capped across stalls
    stance = b["kpr_a"][wc] < 100
    print("GAIT [T0+2.6, TOFF]: kp_h command band / wrapped fraction (time-weighted) / delivered kp_h by gain set")
    for l in L4:
        kp = b["kp_c_" + l][wc]; dkp = vkp(kp, kp_max_fw)
        over = kp > WRAP_KP
        fr = float(np.sum(dt * over) / np.sum(dt))
        fr_fl = float(np.sum(dt * (over & ~stance)) / max(np.sum(dt * ~stance), 1e-9))
        fr_st = float(np.sum(dt * (over & stance)) / max(np.sum(dt * stance), 1e-9))
        row = dict(cmd_min=float(kp.min()), cmd_med=float(np.median(kp)), cmd_max=float(kp.max()),
                   wrapped_frac=fr, wrapped_frac_flight=fr_fl, wrapped_frac_stance=fr_st,
                   delivered_flight=[float(np.min(dkp[~stance])), float(np.median(dkp[~stance])), float(np.max(dkp[~stance]))] if (~stance).any() else None,
                   delivered_stance=[float(np.min(dkp[stance])), float(np.median(dkp[stance])), float(np.max(dkp[stance]))] if stance.any() else None,
                   delivered_timeweighted_med=float(np.median(np.repeat(dkp, np.maximum(1, (dt * 1000).astype(int))))),
                   stance_time_frac=float(np.sum(dt * stance) / np.sum(dt)))
        res["gait"][l] = row
        print("   %s cmd min/med/max %5.1f / %5.1f / %5.1f | wrapped %4.0f %% (flight %4.0f %%, stance %4.0f %%) | delivered flight %s  stance %s"
              % (NM[l], row["cmd_min"], row["cmd_med"], row["cmd_max"], 100 * fr, 100 * fr_fl, 100 * fr_st,
                 "%.0f/%.0f/%.0f" % tuple(row["delivered_flight"]) if row["delivered_flight"] else "-",
                 "%.0f/%.0f/%.0f" % tuple(row["delivered_stance"]) if row["delivered_stance"] else "-"))
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bags", nargs="*")
    ap.add_argument("--kp-max-fw", type=float, default=500.0, help="firmware KP_MAX register (addr 25); 500 = boot default")
    ap.add_argument("--kt", type=float, default=2.148, help="driver KT_yaml for Motor_H (config.yaml)")
    ap.add_argument("--out", default=None)
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        ok = selftest(a.kt)
        if not a.bags:
            sys.exit(0 if ok else 1)
    results = [r for r in (analyse(db, a.kp_max_fw, a.kt) for db in a.bags) if r]
    if results:
        rat = [v["ratio"] for r in results for v in r["hold"].values() if v.get("ratio") is not None]
        prd = [v["predicted_ratio"] for r in results for v in r["hold"].values() if v.get("ratio") is not None]
        print("\nSUMMARY over %d bags: %d resolvable leg-holds; readback ratio median %.3f (prediction median %.3f, ratio %.2f); "
              "wrapped fraction per arc %.2f-%.2f"
              % (len(results), len(rat), np.median(rat), np.median(prd), np.median(rat) / np.median(prd),
                 min(r["gait"]["a"]["wrapped_frac"] for r in results), max(r["gait"]["a"]["wrapped_frac"] for r in results)))
    if a.out and results:
        with open(a.out, "w") as fh:
            json.dump(results, fh, indent=1)
        print("wrote", a.out)


if __name__ == "__main__":
    main()
