#!/usr/bin/env python3
"""Convert a stage15 IMU dump (.npz, record_camber_imu.py) to the offline
replay CSV that corgi_odometry's offline_test consumes.

The offline pipeline reads rows at a uniform Config::DT = 1 ms (the "Time"
column is informational); GT velocity is derived internally from sim_pos by
central difference. This converter therefore resamples every stream onto a
uniform 1 kHz clock spanning the post-trigger record.

Conventions handled here (all verified against build_leg_observation):
- CSV angles are RADIANS in the robot's RAW sign convention (the node flips
  beta and beta_dot for right-side legs itself). motor_deg is deg -> rad.
- beta is wrapped to [0, 2pi): ContactMap::lookup's rim polynomials have
  never seen wheel-mode continuous beta (6+ revolutions). KNOWN RISK: if
  lookup still misclassifies the closed wheel, every leg goes NO_CONTACT and
  the replay drifts on pure IMU -- visible in the per-leg diagnostics, not
  silent.
- vel_r/vel_l are reconstructed so the node's decode recovers the smoothed
  derivatives exactly (same algebra both sides):
      vl = theta_dot + beta_dot_raw,  vr = beta_dot_raw - theta_dot
- state_gamma_* / state_vel_h_* are raw radians (gamma_signs live in the
  estimator config, all +1 per the stage15 adjudication).

Self-checks before writing (refuse-unfit-input rule): schema, timestamp
health, node-decode round-trip on the velocity columns, |imu accel| ~ g,
quat norms ~ 1.

Usage:
    python3 npz_to_offline_csv.py <dump.npz> <out.csv>
"""
import sys

import numpy as np

DT = 0.001
G_BAND = (8.0, 12.0)
SMOOTH_S = 0.021


def _movavg(v, n):
    if n <= 1:
        return v
    k = np.ones(n) / n
    if v.ndim == 1:
        return np.convolve(v, k, mode="same")
    return np.apply_along_axis(lambda c: np.convolve(c, k, mode="same"), 0, v)


def _rate(t, v):
    dt = float(np.median(np.diff(t)))
    return _movavg(np.gradient(v, axis=0) / dt, max(1, int(round(SMOOTH_S / dt)) | 1))


def _interp(tq, ts, v):
    if v.ndim == 1:
        return np.interp(tq, ts, v)
    return np.stack([np.interp(tq, ts, v[:, k]) for k in range(v.shape[1])], -1)


def _health(t, name):
    dup = 1.0 - len(np.unique(t)) / len(t)
    back = int(np.sum(np.diff(t) < 0.0))
    if dup > 0.01 or back > 0:
        raise SystemExit(f"REFUSED: {name} dup {dup:.2%} back {back}")


def main(npz_path, csv_path):
    d = np.load(npz_path)
    for k in ("motor_t", "motor_deg", "torque_t", "torque_nm",
              "imu_t", "imu_quat", "imu_gyro", "imu_accel",
              "odom_t", "odom"):
        if k not in d:
            raise SystemExit(f"REFUSED: missing key {k} (IMU dump required)")
    mt, motor = d["motor_t"], np.deg2rad(d["motor_deg"])
    qt, torque = d["torque_t"], d["torque_nm"]
    it, iq, ig, ia = d["imu_t"], d["imu_quat"], d["imu_gyro"], d["imu_accel"]
    ot, odom = d["odom_t"], d["odom"]
    _health(mt, "motor_t"), _health(it, "imu_t"), _health(ot, "odom_t")

    a_norm = float(np.linalg.norm(ia, axis=1).mean())
    if not (G_BAND[0] < a_norm < G_BAND[1]):
        raise SystemExit(f"REFUSED: |accel| mean {a_norm:.2f} not ~g "
                         f"(driver device cross-wiring suspect)")
    qn = float(np.linalg.norm(iq, axis=1).mean())
    if abs(qn - 1.0) > 0.05:
        raise SystemExit(f"REFUSED: imu quat norm {qn:.3f}")

    t0 = max(0.0, mt.min(), it.min(), ot.min())
    t1 = min(mt.max(), it.max(), ot.max())
    t = np.arange(t0, t1, DT)

    theta = _interp(t, mt, motor[:, :, 0])
    beta = _interp(t, mt, motor[:, :, 1])          # continuous, raw signs
    gamma = _interp(t, mt, motor[:, :, 2])
    theta_d = _interp(t, mt, _rate(mt, motor[:, :, 0]))
    beta_d = _interp(t, mt, _rate(mt, motor[:, :, 1]))
    gamma_d = _interp(t, mt, _rate(mt, motor[:, :, 2]))
    trq_r = _interp(t, qt, torque[:, :, 0])
    trq_l = _interp(t, qt, torque[:, :, 1])
    vl = theta_d + beta_d
    vr = beta_d - theta_d
    # node-decode round-trip: theta_d = (-vr+vl)/2, beta_d = (vr+vl)/2
    assert np.allclose((-vr + vl) / 2, theta_d, atol=1e-9)
    assert np.allclose((vr + vl) / 2, beta_d, atol=1e-9)
    beta_wrapped = np.mod(beta, 2 * np.pi)

    imu_q = _interp(t, it, iq)
    imu_q /= np.linalg.norm(imu_q, axis=1, keepdims=True)
    imu_w = _interp(t, it, ig)
    imu_a = _interp(t, it, ia)
    pos = _interp(t, ot, odom[:, 0:3])
    quat = _interp(t, ot, odom[:, 6:10])
    quat /= np.linalg.norm(quat, axis=1, keepdims=True)

    legs = "abcd"
    cols = ["Time", "imu_seq", "imu_sec", "imu_nsec",
            "imu_orien_x", "imu_orien_y", "imu_orien_z", "imu_orien_w",
            "imu_ang_vel_x", "imu_ang_vel_y", "imu_ang_vel_z",
            "imu_lin_acc_x", "imu_lin_acc_y", "imu_lin_acc_z",
            "sim_pos_x", "sim_pos_y", "sim_pos_z",
            "sim_orien_x", "sim_orien_y", "sim_orien_z", "sim_orien_w"]
    for leg in legs:
        cols += [f"state_theta_{leg}", f"state_beta_{leg}",
                 f"state_vel_r_{leg}", f"state_vel_l_{leg}",
                 f"state_trq_r_{leg}", f"state_trq_l_{leg}",
                 f"state_gamma_{leg}", f"state_vel_h_{leg}"]

    n = len(t)
    out = np.empty((n, len(cols)))
    out[:, 0] = t
    out[:, 1] = np.arange(n)
    out[:, 2] = np.floor(t).astype(int)
    out[:, 3] = np.round((t - np.floor(t)) * 1e9)
    out[:, 4:8] = imu_q
    out[:, 8:11] = imu_w
    out[:, 11:14] = imu_a
    out[:, 14:17] = pos
    out[:, 17:21] = quat
    c = 21
    for j in range(4):
        out[:, c + 0] = theta[:, j]
        out[:, c + 1] = beta_wrapped[:, j]
        out[:, c + 2] = vr[:, j]
        out[:, c + 3] = vl[:, j]
        out[:, c + 4] = trq_r[:, j]
        out[:, c + 5] = trq_l[:, j]
        out[:, c + 6] = gamma[:, j]
        out[:, c + 7] = gamma_d[:, j]
        c += 8

    header = ",".join(cols)
    np.savetxt(csv_path, out, delimiter=",", header=header, comments="",
               fmt="%.9g")
    print(f"{csv_path}: {n} rows x {len(cols)} cols, "
          f"t [{t0:.2f}, {t1:.2f}] s, |accel| {a_norm:.2f}")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise SystemExit(__doc__)
    main(sys.argv[1], sys.argv[2])
