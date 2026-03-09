#!/usr/bin/env python3
"""
ESEKF Fusion Diagnosis: Fair Comparison with contact_state_analysis.py
=======================================================================
Uses the SAME Schmitt trigger logic (rm_high=25, rm_low=15, beta_high=10, beta_low=1)
and the same link-leg kinematics as contact_state_analysis.py.
This ensures both Leg FK and ESEKF are evaluated under the same contact detection.

GT velocity: sim_pos finite difference + 10Hz IIR LPF (same as contact_state_analysis.py)
NO filtering on integration data (IMU dead-reckoning).
Primary metric: Vx RMSE
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import matplotlib
matplotlib.use("Agg")

# =====================================================================
# Constants (match contact_state_analysis.py)
# =====================================================================
DT = 0.001
START_INDEX = 5000
N_ANALYSIS = 12000
WARMUP = 2000

G_MAG = 9.81
g_world = np.array([0.0, 0.0, -G_MAG])

R_wheel = 0.10
r_tire = 0.019
LEG_NAMES = ['a', 'b', 'c', 'd']
LEG_SIGNS = [(1, 1), (1, -1), (-1, -1), (-1, 1)]   # (x_sign, y_sign)
OFFSET_X = 0.222
OFFSET_Y = 0.193

# =====================================================================
# Link-leg kinematics (identical to contact_state_analysis.py)
# =====================================================================
l1 = 0.8 * R_wheel
l2 = 0.9 * R_wheel
l3 = 1.3 * R_wheel
l4 = 0.4 * R_wheel
l5 = 0.88296634 * R_wheel
l6 = 0.2 * R_wheel
l7 = 2.0 * R_wheel * np.sin(np.radians(101.0 / 2.0))
l8 = 2.0 * R_wheel * np.sin(np.radians(113.0 / 2.0))
l9 = 2.0 * R_wheel * np.sin(np.radians(17.0 / 2.0))
l10 = 2.0 * R_wheel * np.sin(np.radians(50.0 / 2.0))

to1 = np.radians(39.5)
to2 = -np.radians(65.0)
tf = np.radians(6.0)
th = np.radians(121.0)


def D_phi(theta, theta_d, _l1, _l3):
    k = _l1 / _l3
    phi = np.arcsin(k * np.sin(theta))
    ph = k * np.cos(theta) / np.cos(np.arcsin(k * np.sin(theta)))
    phi_d = ph * theta_d
    return phi, phi_d


def linkleg_calculate(theta, theta_d):
    phi, phi_d = D_phi(theta, theta_d, l1, l3)
    A = l1 * np.exp(1j * theta)
    B = R_wheel * np.exp(1j * theta)
    oe = l1 * np.cos(theta) - l3 * np.cos(phi)
    E = oe + 0j
    D = E + l4 * np.exp(1j * phi)

    db2 = l2**2 + l6**2 - 2*l2*l6*np.cos(np.pi - theta + phi)
    db = np.sqrt(db2)
    db2_term = l2*l6*np.sin(np.pi - theta + phi)
    db_d = db2_term * (-theta_d + phi_d) / db

    epsilon = np.arctan2(l2*np.sin(phi) + l6*np.sin(theta),
                         l2*np.cos(phi) + l6*np.cos(theta))
    epsilon_d = (l2**2*phi_d + l6**2*theta_d +
                 l2*l6*np.cos(phi - theta)*(phi_d + theta_d)) / db2

    theta2 = np.arccos(np.clip((db2 + l7**2 - l5**2) / (2*db*l7), -1, 1))
    num = db_d * l7 * (4*db**2 - 2*(db**2 + l7**2 - l5**2))
    num_part = (2*db*l7)**2
    den_sq = num_part**2 + (db**2 + l7**2 - l5**2)**2 * num_part
    theta2_d = num / np.sqrt(den_sq) if den_sq > 0 else 0.0

    to_ = np.pi - theta2 + epsilon
    to_d = -theta2_d + epsilon_d

    C = B + l7 * np.exp(1j * to_)
    O1 = B + R_wheel * np.exp(1j * (to_ + to1))
    F = B + l8 * np.exp(1j * (to_ + tf))
    H = B + l9 * np.exp(1j * (to_ + th))

    rho = np.arcsin(
        np.clip((R_wheel*np.sin(theta) + l8*np.sin(to_ + tf)) / l10, -1, 1))
    num_rho = R_wheel*np.cos(theta)*theta_d + l8*to_d*np.cos(to_ + tf)
    den_sq_rho = l10**2 - (R_wheel*np.sin(theta) + l8*np.sin(to_ + tf))**2
    rho_d = num_rho / np.sqrt(den_sq_rho) if den_sq_rho > 0 else 0.0

    G = F - l10 * np.exp(1j * rho)
    O2 = G + R_wheel * np.exp(1j * (rho + to2))

    O1_d_c = (R_wheel * theta_d * np.exp(1j*(theta + np.pi/2)) +
              R_wheel * to_d * np.exp(1j*(to_ + to1 + np.pi/2)))
    G_d_c = (R_wheel * theta_d * np.exp(1j*(theta + np.pi/2)) +
             l8 * to_d * np.exp(1j*(to_ + tf + np.pi/2)) -
             l10 * rho_d * np.exp(1j*(rho + np.pi/2)))
    O2_d_c = G_d_c + R_wheel * rho_d * np.exp(1j*(rho + to2 + np.pi/2))

    O1_ = np.conj(O1)
    O2_ = np.conj(O2)
    O1_d_c_ = np.conj(O1_d_c)
    O2_d_c_ = np.conj(O2_d_c)
    O1_w = to_d
    O1_w_ = -O1_w
    O2_w = rho_d
    O2_w_ = -O2_w

    return {
        'O1': O1, 'O2': O2, 'G': G, 'O1_': O1_, 'O2_': O2_,
        'O1_d': O1_d_c, 'O2_d': O2_d_c, 'G_d': G_d_c,
        'O1_d_': O1_d_c_, 'O2_d_': O2_d_c_,
        'O1_w': O1_w, 'O2_w': O2_w, 'O1_w_': O1_w_, 'O2_w_': O2_w_,
        'F': F, 'rho': rho, 'rho_d': rho_d,
        'to': to_, 'to_d': to_d,
    }


def leg_calculate(theta, theta_d, beta, beta_d):
    fk0 = linkleg_calculate(theta, theta_d)
    rot = np.exp(1j * beta)
    rot_v = beta_d * np.exp(1j * (beta + np.pi/2))
    fk = dict(fk0)
    for key in ['O1', 'O2', 'G', 'O1_', 'O2_']:
        fk[key] = rot * fk0[key]
    fk['F'] = rot * fk0['F']
    fk['O1_d'] = rot * fk0['O1_d'] + rot_v * fk0['O1']
    fk['O1_d_'] = rot * fk0['O1_d_'] + rot_v * fk0['O1_']
    fk['O2_d'] = rot * fk0['O2_d'] + rot_v * fk0['O2']
    fk['O2_d_'] = rot * fk0['O2_d_'] + rot_v * fk0['O2_']
    fk['G_d'] = rot * fk0['G_d'] + rot_v * fk0['G']
    fk['O1_w'] = fk0['O1_w'] + beta_d
    fk['O1_w_'] = fk0['O1_w_'] + beta_d
    fk['O2_w'] = fk0['O2_w'] + beta_d
    fk['O2_w_'] = fk0['O2_w_'] + beta_d
    fk['beta'] = beta
    fk['beta_d'] = beta_d
    return fk


# =====================================================================
# ContactMap lookup (identical to contact_state_analysis.py)
# =====================================================================
def rad_mod2(rad):
    while rad > 2*np.pi:
        rad -= 2*np.pi
    while rad <= 0:
        rad += 2*np.pi
    return rad


def b1(t): return (-2.61019580e-09*t**5 + 1.24181267e-06*t**4 -
                   2.24183011e-04*t**3 + 1.78431692e-02*t**2 - 1.33151836e-01*t - 1.78362899e+00)


def b2(t): return (-1.22581785e-09*t**5 + 5.02932993e-07*t**4 -
                   7.37114643e-05*t**3 + 6.47617996e-03*t**2 - 3.31750539e-01*t + 5.40846840e+01)


def b3(t): return (-4.87190741e-07*t**5 + 3.21347467e-04*t**4 -
                   8.40604260e-02*t**3 + 1.09041600e+01*t**2 - 7.02946587e+02*t + 1.82438639e+04)


def contact_lookup(theta_rad, contact_beta_rad):
    beta = rad_mod2(contact_beta_rad)
    theta_deg = np.degrees(theta_rad)
    beta_deg = np.degrees(beta)
    if theta_deg > 108.3:
        if b1(theta_deg) > beta_deg:
            return 'G_POINT'
        elif b2(theta_deg) >= beta_deg:
            return 'LOWER_RIM_R'
        elif b3(theta_deg) > beta_deg:
            return 'UPPER_RIM_R'
        elif (360 - b3(theta_deg)) > beta_deg:
            return 'NO_CONTACT'
        elif (360 - b2(theta_deg)) > beta_deg:
            return 'UPPER_RIM_L'
        elif (360 - b1(theta_deg)) > beta_deg:
            return 'LOWER_RIM_L'
        else:
            return 'G_POINT'
    else:
        if b1(theta_deg) > beta_deg:
            return 'G_POINT'
        elif b2(theta_deg) >= beta_deg:
            return 'LOWER_RIM_R'
        elif 180.0 > beta_deg:
            return 'UPPER_RIM_R'
        elif (360 - b2(theta_deg)) > beta_deg:
            return 'UPPER_RIM_L'
        elif (360 - b1(theta_deg)) > beta_deg:
            return 'LOWER_RIM_L'
        else:
            return 'G_POINT'


# =====================================================================
# Contact velocity (identical to contact_state_analysis.py)
# =====================================================================
def rim_center_pos(fk, rim, leg_offset):
    mapper = {'G_POINT': 'G', 'UPPER_RIM_R': 'O1', 'LOWER_RIM_R': 'O2',
              'LOWER_RIM_L': 'O2_', 'UPPER_RIM_L': 'O1_'}
    key = mapper.get(rim)
    if key is None:
        return leg_offset.copy()
    c = fk[key]
    return leg_offset + np.array([c.imag, 0.0, c.real])


def rim_center_vel(fk, rim):
    mapper = {'G_POINT': 'G_d', 'UPPER_RIM_R': 'O1_d', 'LOWER_RIM_R': 'O2_d',
              'LOWER_RIM_L': 'O2_d_', 'UPPER_RIM_L': 'O1_d_'}
    key = mapper.get(rim)
    if key is None:
        return np.zeros(3)
    c = fk[key]
    return np.array([c.imag, 0.0, c.real])


def rim_omega(fk, rim, y_offset):
    if rim == 'G_POINT':
        return fk['O2_w_'] if y_offset < 0 else fk['O2_w']
    elif rim == 'UPPER_RIM_R':
        return fk['O1_w']
    elif rim == 'LOWER_RIM_R':
        return fk['O2_w']
    elif rim == 'LOWER_RIM_L':
        return fk['O2_w_']
    elif rim == 'UPPER_RIM_L':
        return fk['O1_w_']
    return 0.0


def rim_radius(rim):
    if rim == 'G_POINT':
        return r_tire
    elif rim == 'NO_CONTACT':
        return 0.0
    else:
        return r_tire + R_wheel


def contact_velocity_noslip(fk, rim, alpha, w_body, leg_offset, y_offset):
    r = rim_radius(rim)
    rv = rim_center_vel(fk, rim)
    link_w = rim_omega(fk, rim, y_offset)
    rim_p = r * np.exp(1j * (np.pi + alpha))
    rp_vec = np.array([rim_p.imag, 0.0, rim_p.real])
    cp = rim_center_pos(fk, rim, leg_offset) + rp_vec
    w_rim = np.array([0.0, link_w, 0.0])
    cv = np.cross(w_body, cp) + rv + np.cross(w_rim, rp_vec)
    return -cv


# =====================================================================
# Schmitt trigger (identical to contact_state_analysis.py)
# =====================================================================
def schmitt_trigger(rm_array, beta_array, rm_high, rm_low, beta_high, beta_low,
                    use_and=False, initial_state=False):
    n = len(rm_array)
    contact = np.zeros(n, dtype=bool)
    state = initial_state
    for i in range(n):
        rm_val = abs(rm_array[i])
        beta_val = abs(beta_array[i])
        if not state:
            if use_and:
                if rm_val > rm_high and beta_val > beta_high:
                    state = True
            else:
                if rm_val > rm_high or beta_val > beta_high:
                    state = True
        else:
            if rm_val < rm_low and beta_val < beta_low:
                state = False
        contact[i] = state
    return contact


# =====================================================================
# Helpers
# =====================================================================
def quat_to_rotmat(qw, qx, qy, qz):
    return np.array([
        [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
        [2*(qx*qy+qz*qw),   1-2*(qx*qx+qz*qz),  2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw),     1-2*(qx*qx+qy*qy)]])


def iir_lpf(data, cutoff_hz, dt):
    alpha = 1.0 - np.exp(-2*np.pi*cutoff_hz*dt)
    out = data.copy().astype(float)
    for i in range(1, len(out)):
        out[i] = (1-alpha)*out[i-1] + alpha*out[i]
    return out


def angle_axis_to_rotmat(dw):
    angle = np.linalg.norm(dw)
    if angle > 1e-8:
        axis = dw / angle
        K = np.array([[0, -axis[2], axis[1]],
                      [axis[2], 0, -axis[0]],
                      [-axis[1], axis[0], 0]])
        return np.eye(3) + np.sin(angle)*K + (1-np.cos(angle))*(K@K)
    return np.eye(3)


# =====================================================================
# Main
# =====================================================================
def main():
    ws = Path(__file__).resolve().parents[5]
    fig_dir = Path(__file__).resolve().parent

    raw_csv = ws / "output_data" / "walk_3m_01m.csv"
    dist_csv = ws / "output_data" / "walk_3m_01m_result.csv"
    esekf_csv = ws / "output_data" / "walk_3m_01m_esekf.csv"

    print("Loading data...")
    raw = pd.read_csv(raw_csv)
    dist = pd.read_csv(dist_csv)
    esekf = pd.read_csv(esekf_csv)

    # Align: disturbance starts at START_INDEX raw rows
    N = N_ANALYSIS
    d = raw.iloc[START_INDEX: START_INDEX + N].reset_index(drop=True)
    dst = dist.iloc[:N].reset_index(drop=True)
    t = np.arange(N) * DT
    print(
        f"  Raw: {len(raw)} rows  |  Analysis: {N} samples ({N*DT:.1f}s)  |  ESEKF: {len(esekf)} rows")

    # =====================================================================
    # 1. Ground Truth velocity (same as contact_state_analysis.py)
    # =====================================================================
    gt_vx_w = iir_lpf(np.gradient(d.sim_pos_x.values, DT), 10.0, DT)
    gt_vy_w = iir_lpf(np.gradient(d.sim_pos_y.values, DT), 10.0, DT)
    gt_vz_w = iir_lpf(np.gradient(d.sim_pos_z.values, DT), 10.0, DT)

    gt_vb = np.zeros((N, 3))
    for i in range(N):
        R = quat_to_rotmat(d.sim_orien_w.iloc[i], d.sim_orien_x.iloc[i],
                           d.sim_orien_y.iloc[i], d.sim_orien_z.iloc[i])
        gt_vb[i] = R.T @ [gt_vx_w[i], gt_vy_w[i], gt_vz_w[i]]
    gt_vx = gt_vb[:, 0]

    # =====================================================================
    # 2. Schmitt trigger contact (same parameters as contact_state_analysis.py)
    # =====================================================================
    RM_COLS = [f'estimated_disturbance_rm_{n}' for n in LEG_NAMES]
    BETA_COLS = [f'estimated_disturbance_beta_{n}' for n in LEG_NAMES]

    # Thresholds from contact_state_analysis.py "Current OR (rm=25,β=10)"
    rm_high, rm_low, beta_high, beta_low = 25, 15, 10, 1

    leg_contacts = {}
    for name in LEG_NAMES:
        leg_contacts[name] = schmitt_trigger(
            dst[f'estimated_disturbance_rm_{name}'].values,
            dst[f'estimated_disturbance_beta_{name}'].values,
            rm_high, rm_low, beta_high, beta_low,
            use_and=False)  # OR logic, same as contact_state_analysis.py

    n_contact_arr = np.array([
        sum(leg_contacts[n][i] for n in LEG_NAMES) for i in range(N)])
    any_contact = n_contact_arr > 0

    # =====================================================================
    # 3. Leg FK velocity (same as contact_state_analysis.py)
    # =====================================================================
    def get_leg_data(leg_name, x_sign, y_sign):
        theta = d[f'state_theta_{leg_name}'].values
        beta = d[f'state_beta_{leg_name}'].values
        vel_r = d[f'state_vel_r_{leg_name}'].values
        vel_l = d[f'state_vel_l_{leg_name}'].values
        is_right = leg_name in ['b', 'c']
        if is_right:
            beta = -beta
        theta_d = (-vel_r + vel_l) / 2.0
        beta_d = -(vel_r + vel_l) / 2.0
        if is_right:
            beta_d = -beta_d
        offset = np.array([x_sign * OFFSET_X, y_sign * OFFSET_Y, 0.0])
        return theta, beta, theta_d, beta_d, offset, y_sign * OFFSET_Y

    leg_data = {name: get_leg_data(name, xs, ys)
                for name, (xs, ys) in zip(LEG_NAMES, LEG_SIGNS)}

    # pitch rate (same as contact_state_analysis.py)
    w_body_y = d.imu_ang_vel_y.values

    vx_legfk = np.full(N, np.nan)
    for i in range(N):
        vx_sum, cnt = 0.0, 0
        w = np.array([0.0, w_body_y[i], 0.0])
        for name, (xs, ys) in zip(LEG_NAMES, LEG_SIGNS):
            if not leg_contacts[name][i]:
                continue
            theta_arr, beta_arr, theta_d_arr, beta_d_arr, offset, y_off = leg_data[name]
            try:
                fk = leg_calculate(
                    theta_arr[i], theta_d_arr[i], beta_arr[i], beta_d_arr[i])
                rim = contact_lookup(theta_arr[i], beta_arr[i])
                if rim == 'NO_CONTACT':
                    continue
                v_est = contact_velocity_noslip(fk, rim, 0.0, w, offset, y_off)
                vx_sum += v_est[0]
                cnt += 1
            except:
                continue
        if cnt > 0:
            vx_legfk[i] = vx_sum / cnt

    # =====================================================================
    # 4. IMU dead-reckoning (NO filtering) + LPF sweep
    # =====================================================================

    def imu_dead_reckoning(acc_x, acc_y, acc_z):
        """Run IMU dead-reckoning with given (possibly pre-filtered) acceleration arrays."""
        vb_arr = np.zeros((N, 3))
        vb = np.zeros(3)
        for i in range(N):
            a_m = np.array([acc_x[i], acc_y[i], acc_z[i]])
            w_m = np.array(
                [d.imu_ang_vel_x.iloc[i], d.imu_ang_vel_y.iloc[i], d.imu_ang_vel_z.iloc[i]])
            R_imu = quat_to_rotmat(d.imu_orien_w.iloc[i], d.imu_orien_x.iloc[i],
                                   d.imu_orien_y.iloc[i], d.imu_orien_z.iloc[i])
            g_b = R_imu.T @ g_world
            R_delta = angle_axis_to_rotmat(w_m * DT)
            vb = R_delta.T @ vb + (a_m + g_b) * DT
            vb_arr[i] = vb
        return vb_arr

    # Raw (no filter)
    raw_ax = d.imu_lin_acc_x.values
    raw_ay = d.imu_lin_acc_y.values
    raw_az = d.imu_lin_acc_z.values
    imu_vb = imu_dead_reckoning(raw_ax, raw_ay, raw_az)

    # Pre-compute eval slice (needed by LPF sweep below)
    sl = slice(WARMUP, N)
    gt_eval = gt_vx[sl]

    # LPF sweep: test multiple cutoff frequencies
    LPF_CUTOFFS = [5, 10, 20, 30]   # Hz
    lpf_results = {}  # cutoff -> (vb_array, rmse, bias)
    for cutoff in LPF_CUTOFFS:
        ax_f = iir_lpf(raw_ax, cutoff, DT)
        ay_f = iir_lpf(raw_ay, cutoff, DT)
        az_f = iir_lpf(raw_az, cutoff, DT)
        vb_f = imu_dead_reckoning(ax_f, ay_f, az_f)
        rmse_f = np.sqrt(np.mean((vb_f[sl, 0] - gt_eval)**2)) * 1000
        bias_f = np.mean(vb_f[sl, 0] - gt_eval) * 1000
        lpf_results[cutoff] = (vb_f, rmse_f, bias_f)

    # =====================================================================
    # 5. ESEKF result
    # =====================================================================
    est_vx = esekf['est_vel_x'].values
    ba_x = esekf['ba_x'].values

    # =====================================================================
    # 6. RMSE comparison (after WARMUP, consistent with contact_state_analysis.py)
    # =====================================================================
    sl = slice(WARMUP, N)
    contact_mask = any_contact[sl]
    gt_eval = gt_vx[sl]

    # Leg FK: only where there's contact (same as contact_state_analysis.py eval_start=2000)
    leg_mask = ~np.isnan(vx_legfk[sl]) & contact_mask
    leg_rmse = np.sqrt(np.nanmean(
        (vx_legfk[sl][leg_mask] - gt_eval[leg_mask])**2)) * 1000
    leg_bias = np.nanmean(vx_legfk[sl][leg_mask] - gt_eval[leg_mask]) * 1000

    # ESEKF fused: all timesteps
    est_rmse = np.sqrt(np.mean((est_vx[sl] - gt_eval)**2)) * 1000
    est_bias = np.mean(est_vx[sl] - gt_eval) * 1000

    # ESEKF fused: contact only (fair comparison)
    est_rmse_c = np.sqrt(
        np.mean((est_vx[sl][contact_mask] - gt_eval[contact_mask])**2)) * 1000
    est_bias_c = np.mean(est_vx[sl][contact_mask] -
                         gt_eval[contact_mask]) * 1000

    # IMU dead-reckoning (raw)
    imu_rmse = np.sqrt(np.mean((imu_vb[sl, 0] - gt_eval)**2)) * 1000
    imu_bias = np.mean(imu_vb[sl, 0] - gt_eval) * 1000

    contact_ratio = np.mean(contact_mask) * 100
    mean_n_legs = np.mean(n_contact_arr[sl])

    print(f"\n{'='*70}")
    print(f"SCHMITT TRIGGER: OR logic, rm_high={rm_high}, rm_low={rm_low}, "
          f"beta_high={beta_high}, beta_low={beta_low}")
    print(
        f"  Contact ratio: {contact_ratio:.1f}%   Avg legs: {mean_n_legs:.2f}")
    print(f"{'='*70}")
    print(f"  {'Method':<40s}  {'RMSE':>8s}  {'Bias':>8s}  (mm/s)")
    print(f"  {'-'*60}")
    print(
        f"  {'Leg FK (Python FK, contact mask)':<40s}  {leg_rmse:8.2f}  {leg_bias:+8.2f}")
    print(f"  {'ESEKF Fused (all steps)':<40s}  {est_rmse:8.2f}  {est_bias:+8.2f}")
    print(
        f"  {'ESEKF Fused (contact mask only)':<40s}  {est_rmse_c:8.2f}  {est_bias_c:+8.2f}")
    print(
        f"  {'IMU dead-reckoning (no filter)':<40s}  {imu_rmse:8.2f}  {imu_bias:+8.2f}")
    print()
    print(f"  {'--- IMU LPF Sweep (offline simulation of hardware LPF) ---':<60s}")
    for cutoff, (_, rmse_f, bias_f) in lpf_results.items():
        label = f'IMU DR + LPF {cutoff}Hz'
        print(f"  {label:<40s}  {rmse_f:8.2f}  {bias_f:+8.2f}")
    print()

    # =====================================================================
    # 7. Plot
    # =====================================================================
    fig, axes = plt.subplots(5, 1, figsize=(14, 18), sharex=True)
    fig.suptitle('ESEKF vs Leg FK — Same Schmitt Trigger Contact Detection\n'
                 f'(OR: rm_high={rm_high}/low={rm_low}, β_high={beta_high}/low={beta_low})',
                 fontsize=13, fontweight='bold')

    # Panel 1: Velocity
    ax = axes[0]
    ax.plot(t, gt_vx*1000, 'k-', lw=1.0, label='GT vx')
    leg_mask_full = ~np.isnan(vx_legfk)
    ax.plot(t[leg_mask_full], vx_legfk[leg_mask_full]*1000, 'b-', lw=0.5, alpha=0.7,
            label=f'Leg FK (RMSE={leg_rmse:.1f} mm/s, contact only)')
    ax.plot(t[:len(est_vx)], est_vx*1000, 'm-', lw=0.6, alpha=0.7,
            label=f'ESEKF Fused (RMSE={est_rmse:.1f} mm/s, all steps)')
    ax.axvline(WARMUP*DT, color='gray', ls='--', lw=0.5,
               label=f'Warmup end ({WARMUP*DT:.0f}s)')
    ax.set_ylabel('vx (mm/s)')
    ax.set_title(
        f'Forward Velocity (contact_ratio={contact_ratio:.0f}%, avg_legs={mean_n_legs:.2f})')
    ax.legend(fontsize=7, loc='upper right')
    ax.grid(True, alpha=0.3)

    # Panel 2: Error
    ax = axes[1]
    leg_err_full = np.full(N, np.nan)
    leg_err_full[leg_mask_full] = (
        vx_legfk[leg_mask_full] - gt_vx[leg_mask_full]) * 1000
    ax.plot(t, leg_err_full, 'b-', lw=0.5, alpha=0.6, label='Leg FK error')
    ax.plot(t[:len(est_vx)], (est_vx - gt_vx[:len(est_vx)]) *
            1000, 'm-', lw=0.5, alpha=0.6, label='ESEKF error')
    ax.axhline(0, color='k', lw=0.3)
    ax.set_ylabel('Vx error (mm/s)')
    ax.set_title('Vx Error')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # Panel 3: ba_x and per-leg contact
    ax = axes[2]
    ax.plot(t[:len(ba_x)], ba_x, 'r-', lw=0.8, label='ba_x (accel bias est.)')
    ax.axhline(0, color='k', lw=0.3)
    ax2 = ax.twinx()
    ax2.fill_between(t, 0, n_contact_arr, alpha=0.2,
                     color='green', label='# legs in contact')
    ax2.set_ylabel('# legs', color='green')
    ax2.set_ylim(0, 6)
    ax.set_ylabel('ba_x (m/s²)', color='r')
    ax.set_title('Accelerometer Bias ba_x + Contact Legs')
    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labels1 + labels2,
              fontsize=7, loc='upper right')
    ax.grid(True, alpha=0.3)

    # Panel 4: IMU raw dead-reckoning vs GT
    ax = axes[3]
    ax.plot(t, gt_vx*1000, 'k-', lw=0.8, label='GT vx')
    ax.plot(t, imu_vb[:, 0]*1000, 'orange', lw=0.5, alpha=0.7,
            label=f'IMU DR – no filter (RMSE={imu_rmse:.1f} mm/s)')
    colors_lpf = ['royalblue', 'seagreen', 'crimson', 'darkorchid']
    for (cutoff, (vb_f, rmse_f, _)), c in zip(lpf_results.items(), colors_lpf):
        ax.plot(t, vb_f[:, 0]*1000, '-', color=c, lw=0.5, alpha=0.7,
                label=f'IMU DR + LPF {cutoff}Hz (RMSE={rmse_f:.1f} mm/s)')
    ax.set_ylabel('vx (mm/s)')
    ax.set_title('IMU Dead-Reckoning: No Filter vs LPF Sweep')
    ax.legend(fontsize=7, loc='upper right')
    ax.grid(True, alpha=0.3)

    # Panel 5: IMU X-axis acceleration (raw vs filtered)
    ax = axes[4]
    ax.plot(t, raw_ax, 'orange', lw=0.3, alpha=0.5, label='Raw acc_x')
    for (cutoff, _), c in zip(lpf_results.items(), colors_lpf):
        lpf_ax = iir_lpf(raw_ax, cutoff, DT)
        ax.plot(t, lpf_ax, '-', color=c, lw=0.5, alpha=0.8,
                label=f'LPF {cutoff}Hz')
    ax.set_ylabel('acc_x (m/s²)')
    ax.set_xlabel('Time (s)')
    ax.set_title(
        'IMU X-axis Acceleration: Raw vs LPF (shows vibration attenuation)')
    ax.legend(fontsize=7, loc='upper right')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    fig_path = fig_dir / "esekf_fusion_diagnosis.png"
    plt.savefig(fig_path, dpi=150)
    print(f"Saved: {fig_path}")


if __name__ == "__main__":
    main()
