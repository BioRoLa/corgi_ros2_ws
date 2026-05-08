#!/usr/bin/env python3
"""
tune_schmitt_trigger.py

Offline Schmitt trigger contact detection tuning for CORGI robot.

Uses:
  - output_data/walk_3m_01m.csv        : raw recorded data (motor state, imu, sim_pos)
  - output_data/walk_3m_01m_result.csv  : disturbance observer output (rm & beta per leg)

Since ground truth contact forces are not available, we validate contact
detection by computing leg-odometry velocity with detected contact and
comparing to sim_pos-derived ground truth velocity.

Adjustable:
  - AND vs OR logic for Schmitt trigger activation
  - RM / beta threshold values
  - Visualization of disturbance signals & detected contact windows

Author: debug helper for corgi_odometry (Step 2)
"""

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from pathlib import Path

# =====================================================================
# Constants
# =====================================================================
DT = 0.001               # 1 kHz
START_INDEX = 0
R_wheel = 0.10
r_tire  = 0.019

LEG_NAMES = ['a', 'b', 'c', 'd']
LEG_LABELS = ['LF (a)', 'RF (b)', 'RH (c)', 'LH (d)']

# Disturbance vector indices: beta_a=4, rm_a=5, beta_b=6, rm_b=7, ...
RM_COLS   = [f'estimated_disturbance_rm_{n}'   for n in LEG_NAMES]
BETA_COLS = [f'estimated_disturbance_beta_{n}' for n in LEG_NAMES]

# =====================================================================
# Link-leg kinematics (Python port, from compare_velocity_methods.py)
# =====================================================================
l1 = 0.8  * R_wheel
l2 = 0.9  * R_wheel
l3 = 1.3  * R_wheel
l4 = 0.4  * R_wheel
l5 = 0.88296634 * R_wheel
l6 = 0.2  * R_wheel
l7 = 2.0  * R_wheel * np.sin(np.radians(101.0 / 2.0))
l8 = 2.0  * R_wheel * np.sin(np.radians(113.0 / 2.0))
l9 = 2.0  * R_wheel * np.sin(np.radians(17.0  / 2.0))
l10= 2.0  * R_wheel * np.sin(np.radians(50.0  / 2.0))

to1 = np.radians(39.5)
to2 = -np.radians(65.0)
tf  = np.radians(6.0)
th  = np.radians(121.0)

OFFSET_X = 0.222
OFFSET_Y = 0.193

# Leg offsets: (x_sign, y_sign) for [LF, RF, RH, LH]
LEG_SIGNS = [(1, 1), (1, -1), (-1, -1), (-1, 1)]


def D_phi(theta, theta_d, _l1, _l3):
    k = _l1 / _l3
    phi = np.arcsin(k * np.sin(theta))
    ph = k * np.cos(theta) / np.cos(np.arcsin(k * np.sin(theta)))
    phi_d = ph * theta_d
    return phi, phi_d


def linkleg_calculate(theta, theta_d):
    phi, phi_d = D_phi(theta, theta_d, l1, l3)
    A  = l1 * np.exp(1j * theta)
    B  = R_wheel * np.exp(1j * theta)
    oe = l1 * np.cos(theta) - l3 * np.cos(phi)
    oe_d = -l1 * np.sin(theta) * theta_d + l3 * np.sin(phi) * phi_d
    E  = oe + 0j
    D  = E + l4 * np.exp(1j * phi)

    db2 = l2**2 + l6**2 - 2*l2*l6*np.cos(np.pi - theta + phi)
    db  = np.sqrt(db2)
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

    C  = B + l7 * np.exp(1j * to_)
    O1 = B + R_wheel * np.exp(1j * (to_ + to1))
    F  = B + l8 * np.exp(1j * (to_ + tf))
    H  = B + l9 * np.exp(1j * (to_ + th))

    rho = np.arcsin(np.clip((R_wheel*np.sin(theta) + l8*np.sin(to_ + tf)) / l10, -1, 1))
    num_rho = R_wheel*np.cos(theta)*theta_d + l8*to_d*np.cos(to_ + tf)
    den_sq_rho = l10**2 - (R_wheel*np.sin(theta) + l8*np.sin(to_ + tf))**2
    rho_d = num_rho / np.sqrt(den_sq_rho) if den_sq_rho > 0 else 0.0

    G   = F - l10 * np.exp(1j * rho)
    O2  = G + R_wheel * np.exp(1j * (rho + to2))

    O1_d_c = (R_wheel * theta_d * np.exp(1j*(theta + np.pi/2)) +
              R_wheel * to_d * np.exp(1j*(to_ + to1 + np.pi/2)))
    G_d_c  = (R_wheel * theta_d * np.exp(1j*(theta + np.pi/2)) +
              l8 * to_d * np.exp(1j*(to_ + tf + np.pi/2)) -
              l10 * rho_d * np.exp(1j*(rho + np.pi/2)))
    O2_d_c = G_d_c + R_wheel * rho_d * np.exp(1j*(rho + to2 + np.pi/2))

    O1_ = np.conj(O1)
    O2_ = np.conj(O2)
    O1_d_c_ = np.conj(O1_d_c)
    O2_d_c_ = np.conj(O2_d_c)

    O1_w  =  to_d
    O1_w_ = -O1_w
    O2_w  =  rho_d
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
    rot  = np.exp(1j * beta)
    rot_v = beta_d * np.exp(1j * (beta + np.pi/2))

    fk = dict(fk0)
    for key in ['O1', 'O2', 'G', 'O1_', 'O2_']:
        fk[key] = rot * fk0[key]
    fk['F'] = rot * fk0['F']

    fk['O1_d']  = rot * fk0['O1_d']  + rot_v * fk0['O1']
    fk['O1_d_'] = rot * fk0['O1_d_'] + rot_v * fk0['O1_']
    fk['O2_d']  = rot * fk0['O2_d']  + rot_v * fk0['O2']
    fk['O2_d_'] = rot * fk0['O2_d_'] + rot_v * fk0['O2_']
    fk['G_d']   = rot * fk0['G_d']   + rot_v * fk0['G']

    fk['O1_w']  = fk0['O1_w']  + beta_d
    fk['O1_w_'] = fk0['O1_w_'] + beta_d
    fk['O2_w']  = fk0['O2_w']  + beta_d
    fk['O2_w_'] = fk0['O2_w_'] + beta_d

    fk['beta'] = beta
    fk['beta_d'] = beta_d
    return fk


# =====================================================================
# ContactMap lookup
# =====================================================================
def rad_mod2(rad):
    while rad > 2*np.pi:
        rad -= 2*np.pi
    while rad <= 0:
        rad += 2*np.pi
    return rad

def b1(t):
    return (-2.61019580e-09*t**5 + 1.24181267e-06*t**4
            - 2.24183011e-04*t**3 + 1.78431692e-02*t**2
            - 1.33151836e-01*t - 1.78362899e+00)

def b2(t):
    return (-1.22581785e-09*t**5 + 5.02932993e-07*t**4
            - 7.37114643e-05*t**3 + 6.47617996e-03*t**2
            - 3.31750539e-01*t + 5.40846840e+01)

def b3(t):
    return (-4.87190741e-07*t**5 + 3.21347467e-04*t**4
            - 8.40604260e-02*t**3 + 1.09041600e+01*t**2
            - 7.02946587e+02*t + 1.82438639e+04)

def contact_lookup(theta_rad, contact_beta_rad):
    beta = rad_mod2(contact_beta_rad)
    theta_deg = np.degrees(theta_rad)
    beta_deg  = np.degrees(beta)
    if theta_deg > 108.3:
        if b1(theta_deg) > beta_deg:        return 'G_POINT'
        elif b2(theta_deg) >= beta_deg:      return 'LOWER_RIM_R'
        elif b3(theta_deg) > beta_deg:       return 'UPPER_RIM_R'
        elif (360 - b3(theta_deg)) > beta_deg: return 'NO_CONTACT'
        elif (360 - b2(theta_deg)) > beta_deg: return 'UPPER_RIM_L'
        elif (360 - b1(theta_deg)) > beta_deg: return 'LOWER_RIM_L'
        else:                                return 'G_POINT'
    else:
        if b1(theta_deg) > beta_deg:        return 'G_POINT'
        elif b2(theta_deg) >= beta_deg:      return 'LOWER_RIM_R'
        elif 180.0 > beta_deg:              return 'UPPER_RIM_R'
        elif (360 - b2(theta_deg)) > beta_deg: return 'UPPER_RIM_L'
        elif (360 - b1(theta_deg)) > beta_deg: return 'LOWER_RIM_L'
        else:                                return 'G_POINT'


# =====================================================================
# Contact velocity (PointVelocity with v=0, no-slip constraint)
# =====================================================================
def complex_to_xz(c):
    return np.array([c.imag, c.real])

def rim_center_pos(fk, rim, leg_offset):
    mapper = {'G_POINT':'G','UPPER_RIM_R':'O1','LOWER_RIM_R':'O2',
              'LOWER_RIM_L':'O2_','UPPER_RIM_L':'O1_'}
    key = mapper.get(rim)
    if key is None:
        return leg_offset.copy()
    c = fk[key]
    return leg_offset + np.array([c.imag, 0.0, c.real])

def rim_center_vel(fk, rim):
    mapper = {'G_POINT':'G_d','UPPER_RIM_R':'O1_d','LOWER_RIM_R':'O2_d',
              'LOWER_RIM_L':'O2_d_','UPPER_RIM_L':'O1_d_'}
    key = mapper.get(rim)
    if key is None:
        return np.zeros(3)
    c = fk[key]
    return np.array([c.imag, 0.0, c.real])

def rim_omega(fk, rim, y_offset):
    if rim == 'G_POINT':
        return fk['O2_w_'] if y_offset < 0 else fk['O2_w']
    elif rim == 'UPPER_RIM_R':  return fk['O1_w']
    elif rim == 'LOWER_RIM_R':  return fk['O2_w']
    elif rim == 'LOWER_RIM_L':  return fk['O2_w_']
    elif rim == 'UPPER_RIM_L':  return fk['O1_w_']
    return 0.0

def rim_radius(rim):
    if rim == 'G_POINT':      return r_tire
    elif rim == 'NO_CONTACT': return 0.0
    else:                     return r_tire + R_wheel

def contact_velocity_noslip(fk, rim, alpha, w_body, leg_offset, y_offset):
    """PointVelocity with v=0 (no-slip), return -cv => body velocity estimate."""
    r = rim_radius(rim)
    rv = rim_center_vel(fk, rim)
    link_w = rim_omega(fk, rim, y_offset)
    rim_p = r * np.exp(1j * (np.pi + alpha))
    rp_vec = np.array([rim_p.imag, 0.0, rim_p.real])
    # contact_point = offset + rim_center + rp  (matching C++ Leg::PointContact)
    cp = rim_center_pos(fk, rim, leg_offset) + rp_vec
    w_rim = np.array([0.0, link_w, 0.0])
    # cv = v + w×cp + rv + w_rim×rp  (v=0 for no-slip)
    cv = np.cross(w_body, cp) + rv + np.cross(w_rim, rp_vec)
    return -cv   # body velocity = -contact_velocity(v=0)


# =====================================================================
# Schmitt trigger implementation
# =====================================================================
def schmitt_trigger(rm_array, beta_array, rm_high, rm_low, beta_high, beta_low,
                    use_and=True, initial_state=False):
    """
    Schmitt trigger contact detection.
    
    Parameters
    ----------
    rm_array, beta_array : 1D arrays of disturbance values
    rm_high/low, beta_high/low : threshold values
    use_and : if True, BOTH rm AND beta must exceed high threshold to activate
              if False, EITHER rm OR beta exceeding high threshold activates (current code)
    initial_state : initial contact state
    
    Returns
    -------
    contact : bool array
    """
    n = len(rm_array)
    contact = np.zeros(n, dtype=bool)
    state = initial_state
    
    for i in range(n):
        rm_val   = abs(rm_array[i])
        beta_val = abs(beta_array[i])
        
        if not state:
            # Currently not in contact → check for activation
            if use_and:
                if rm_val > rm_high and beta_val > beta_high:
                    state = True
            else:
                if rm_val > rm_high or beta_val > beta_high:
                    state = True
        else:
            # Currently in contact → check for deactivation
            if rm_val < rm_low and beta_val < beta_low:
                state = False
        
        contact[i] = state
    
    return contact


# =====================================================================
# Quaternion → rotation matrix
# =====================================================================
def quat_to_rotmat(qw, qx, qy, qz):
    return np.array([
        [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
        [2*(qx*qy+qz*qw),   1-2*(qx*qx+qz*qz),  2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw),     1-2*(qx*qx+qy*qy)]])


# =====================================================================
# Main
# =====================================================================
def main():
    base = Path(__file__).resolve().parents[5]  # corgi_ros2_ws root
    out_dir = Path(__file__).resolve().parent
    
    raw_csv  = base / 'output_data' / 'walk_2m_01.csv'
    dist_csv = base / 'output_data' / 'walk_2m_01_result.csv'
    
    print("Loading data...")
    raw  = pd.read_csv(raw_csv)
    dist = pd.read_csv(dist_csv)
    print(f"  Raw: {len(raw)} rows,  Disturbance: {len(dist)} rows")
    
    # Align: disturbance starts at Index=START_INDEX
    # raw rows [START_INDEX : START_INDEX + len(dist)] correspond to dist rows
    raw_aligned = raw.iloc[START_INDEX : START_INDEX + len(dist)].reset_index(drop=True)
    
    ANALYSIS_SAMPLES = 12000
    raw_aligned = raw_aligned.iloc[:ANALYSIS_SAMPLES].reset_index(drop=True)
    dist        = dist.iloc[:ANALYSIS_SAMPLES].reset_index(drop=True)
    N = len(dist)
    t = np.arange(N) * DT
    
    print(f"  After alignment: {N} samples ({N*DT:.1f}s)")
    print(f"  Eval window: raw [{START_INDEX}:{START_INDEX+N}] = {(N)*DT:.1f}s")
    
    # ---- Extract disturbance signals ----
    rm_signals   = {n: dist[f'estimated_disturbance_rm_{n}'].values   for n in LEG_NAMES}
    beta_signals = {n: dist[f'estimated_disturbance_beta_{n}'].values for n in LEG_NAMES}
    
    # ---- Ground truth velocity: world-frame finite diff + LPF → rotate to body frame ----
    gt_vx_w = np.gradient(raw_aligned['sim_pos_x'].values, DT)
    gt_vy_w = np.gradient(raw_aligned['sim_pos_y'].values, DT)
    gt_vz_w = np.gradient(raw_aligned['sim_pos_z'].values, DT)
    # 10 Hz IIR LPF in world frame
    alpha_lpf = 1.0 - np.exp(-2.0 * np.pi * 10.0 * DT)  # 10 Hz cutoff for GT
    for i in range(1, N):
        gt_vx_w[i] = (1 - alpha_lpf) * gt_vx_w[i-1] + alpha_lpf * gt_vx_w[i]
        gt_vy_w[i] = (1 - alpha_lpf) * gt_vy_w[i-1] + alpha_lpf * gt_vy_w[i]
        gt_vz_w[i] = (1 - alpha_lpf) * gt_vz_w[i-1] + alpha_lpf * gt_vz_w[i]
    # Rotate world → body frame using GT quaternion (R^T * v_world)
    gt_vb = np.zeros((N, 3))
    for i in range(N):
        R_gt = quat_to_rotmat(raw_aligned['sim_orien_w'].iloc[i],
                               raw_aligned['sim_orien_x'].iloc[i],
                               raw_aligned['sim_orien_y'].iloc[i],
                               raw_aligned['sim_orien_z'].iloc[i])
        gt_vb[i] = R_gt.T @ [gt_vx_w[i], gt_vy_w[i], gt_vz_w[i]]
    gt_vx = gt_vb[:, 0]  # body-frame forward velocity
    gt_vz = gt_vb[:, 2]  # body-frame vertical velocity
    
    # ---- Extract leg joint data (using vel_r/vel_l, matching C++ offline_test) ----
    def get_leg_data(leg_name, x_sign, y_sign):
        theta = raw_aligned[f'state_theta_{leg_name}'].values
        beta  = raw_aligned[f'state_beta_{leg_name}'].values
        vel_r = raw_aligned[f'state_vel_r_{leg_name}'].values
        vel_l = raw_aligned[f'state_vel_l_{leg_name}'].values
        
        is_right = leg_name in ['b', 'c']
        # Right-side legs (b, c) need beta negation
        if is_right:
            beta = -beta
        
        # Compute theta_d, beta_d from motor velocities (matching C++ offline_test)
        theta_d = (-vel_r + vel_l) / 2.0
        beta_d  =  (vel_r + vel_l) / 2.0
        if is_right:
            beta_d = -beta_d
        
        offset = np.array([x_sign * OFFSET_X, y_sign * OFFSET_Y, 0.0])
        return theta, beta, theta_d, beta_d, offset, y_sign * OFFSET_Y
    
    leg_data = {}
    for name, (xs, ys) in zip(LEG_NAMES, LEG_SIGNS):
        leg_data[name] = get_leg_data(name, xs, ys)
    
    # ---- IMU angular velocity ----
    w_body_y = raw_aligned['imu_ang_vel_y'].values  # pitch rate
    
    # =================================================================
    # Test multiple threshold configurations
    # =================================================================
    configs = [
        # (label, rm_high, rm_low, beta_high, beta_low, use_and)
        ("Current OR (rm=25,β=10)",       25,  15,  10,   1, False),
    ]
    
    results = []
    
    for cfg_label, rm_h, rm_l, beta_h, beta_l, use_and in configs:
        # Run Schmitt trigger for each leg
        leg_contacts = {}
        for name in LEG_NAMES:
            leg_contacts[name] = schmitt_trigger(
                rm_signals[name], beta_signals[name],
                rm_h, rm_l, beta_h, beta_l, use_and
            )
        
        # Compute velocity estimate using contact legs only
        vx_est = np.zeros(N)
        vz_est = np.zeros(N)
        n_contact = np.zeros(N)
        
        for i in range(N):
            vx_sum, vz_sum, cnt = 0.0, 0.0, 0
            w = np.array([0.0, w_body_y[i], 0.0])
            
            for name, (xs, ys) in zip(LEG_NAMES, LEG_SIGNS):
                if not leg_contacts[name][i]:
                    continue
                
                theta_arr, beta_arr, theta_d_arr, beta_d_arr, offset, y_off = leg_data[name]
                theta_i = theta_arr[i]
                beta_i  = beta_arr[i]
                theta_d_i = theta_d_arr[i]
                beta_d_i  = beta_d_arr[i]
                
                try:
                    fk = leg_calculate(theta_i, theta_d_i, beta_i, beta_d_i)
                    contact_beta = beta_i  # same frame after negation
                    rim = contact_lookup(theta_i, contact_beta)
                    if rim == 'NO_CONTACT':
                        continue
                    alpha = contact_beta - beta_i  # = 0 in this simple case
                    
                    v_est = contact_velocity_noslip(fk, rim, alpha, w, offset, y_off)
                    vx_sum += v_est[0]
                    vz_sum += v_est[2]
                    cnt += 1
                except:
                    continue
            
            if cnt > 0:
                vx_est[i] = vx_sum / cnt
                vz_est[i] = vz_sum / cnt
            n_contact[i] = cnt
        
        # No LPF applied to velocity estimate
        vx_filt = vx_est.copy()
        vz_filt = vz_est.copy()
        
        # Metrics (skip first 2s after skip for filter warmup)
        eval_start = 2000
        err_vx = vx_filt[eval_start:] - gt_vx[eval_start:]
        rmse_vx = np.sqrt(np.mean(err_vx**2))
        mean_contact = np.mean(n_contact[eval_start:])
        contact_ratio = np.mean(n_contact[eval_start:] > 0)
        
        total_per_leg = {}
        for name in LEG_NAMES:
            total_per_leg[name] = np.mean(leg_contacts[name][eval_start:])
        
        results.append({
            'label': cfg_label,
            'rmse_vx': rmse_vx,
            'mean_n_contact': mean_contact,
            'contact_ratio': contact_ratio,
            'per_leg': total_per_leg,
            'vx_filt': vx_filt,
            'leg_contacts': leg_contacts,
            'n_contact': n_contact,
        })
        
        print(f"  [{cfg_label}] (rm: HIGH={rm_h}, LOW={rm_l} | beta: HIGH={beta_h}, LOW={beta_l})")
        print(f"    RMSE_vx={rmse_vx*1000:.1f} mm/s, "
              f"avg_legs={mean_contact:.2f}, coverage={contact_ratio*100:.1f}%, "
              f"per_leg=[{total_per_leg['a']:.2f},{total_per_leg['b']:.2f},"
              f"{total_per_leg['c']:.2f},{total_per_leg['d']:.2f}]")
    
    # =================================================================
    # Plot 1: Disturbance signals overview
    # =================================================================
    fig, axes = plt.subplots(4, 2, figsize=(18, 14), sharex=True)
    fig.suptitle("Disturbance Observer Output — walk_2m_01", fontsize=14)
    
    for idx, (name, label) in enumerate(zip(LEG_NAMES, LEG_LABELS)):
        ax_rm = axes[idx, 0]
        ax_beta = axes[idx, 1]
        
        ax_rm.plot(t, np.abs(rm_signals[name]), 'b-', linewidth=0.3, alpha=0.7)
        ax_rm.axhline(25, color='r', linestyle='--', linewidth=0.8, label='HIGH=25')
        ax_rm.axhline(15, color='orange', linestyle=':', linewidth=0.8, label='LOW=15')
        ax_rm.set_ylabel(f'{label}\n|rm| [N]')
        ax_rm.set_ylim(0, 200)
        ax_rm.legend(loc='upper right', fontsize=7)
        ax_rm.grid(True, alpha=0.3)
        
        ax_beta.plot(t, np.abs(beta_signals[name]), 'g-', linewidth=0.3, alpha=0.7)
        ax_beta.axhline(10, color='r', linestyle='--', linewidth=0.8, label='HIGH=10')
        ax_beta.axhline(1, color='orange', linestyle=':', linewidth=0.8, label='LOW=1')
        ax_beta.set_ylabel(f'{label}\n|beta| [Nm]')
        ax_beta.set_ylim(0, 30)
        ax_beta.legend(loc='upper right', fontsize=7)
        ax_beta.grid(True, alpha=0.3)
    
    axes[-1, 0].set_xlabel('Time [s]')
    axes[-1, 1].set_xlabel('Time [s]')
    plt.tight_layout()
    fig.savefig(out_dir / "disturbance_signals.png", dpi=150)
    print(f"\nSaved disturbance_signals.png")
    plt.close(fig)
    
    # =================================================================
    # Plot 2: Contact detection (single subplot, 4:3)
    # =================================================================
    res = results[0]
    fig, ax = plt.subplots(1, 1, figsize=(8, 6))  # 4:3
    for li, (name, label) in enumerate(zip(LEG_NAMES, LEG_LABELS)):
        contact = res['leg_contacts'][name].astype(float)
        ax.fill_between(t, li, li + contact * 0.8, alpha=0.6, label=label)
    ax.set_ylim(-0.2, 4.5)
    ax.set_yticks([0.4, 1.4, 2.4, 3.4])
    ax.set_yticklabels(LEG_LABELS)
    rmse_str = f"RMSE={res['rmse_vx']*1000:.1f}mm/s"
    ax.set_title(f"{res['label']}  —  {rmse_str}, avg_legs={res['mean_n_contact']:.2f}")
    ax.set_xlabel('Time [s]')
    ax.grid(True, alpha=0.3, axis='x')
    plt.tight_layout()
    fig.savefig(out_dir / "contact_detection_comparison.png", dpi=150)
    print(f"Saved contact_detection_comparison.png")
    plt.close(fig)
    
    # =================================================================
    # Plot 3: Velocity estimation comparison (5-15s, 4:3)
    # =================================================================
    res = results[0]
    t_mask = (t >= 5.0) & (t <= 15.0)
    
    fig, ax = plt.subplots(1, 1, figsize=(8, 6))  # 4:3
    ax.plot(t[t_mask], gt_vx[t_mask], 'k-', linewidth=0.8, label='GT vx', alpha=0.7)
    ax.plot(t[t_mask], res['vx_filt'][t_mask], 'r-', linewidth=0.5, alpha=0.7,
            label=f"est vx (RMSE={res['rmse_vx']*1000:.1f}mm/s)")
    ax.set_ylabel('vx [m/s]')
    ax.set_xlabel('Time [s]')
    ax.set_title(f"Velocity Estimation")
    ax.set_xlim(5, 12)
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    fig.savefig(out_dir / "velocity_estimation_comparison.png", dpi=150)
    print(f"Saved velocity_estimation_comparison.png")
    plt.close(fig)
    
    # =================================================================
    # Plot 4: rm vs beta scatter (for threshold visualization)
    # =================================================================
    fig, axes = plt.subplots(2, 2, figsize=(14, 12))
    fig.suptitle("|rm| vs |beta| Scatter — Threshold Region Visualization", fontsize=14)
    
    for idx, (name, label) in enumerate(zip(LEG_NAMES, LEG_LABELS)):
        ax = axes[idx // 2, idx % 2]
        rm_abs   = np.abs(rm_signals[name])
        beta_abs = np.abs(beta_signals[name])
        
        ax.scatter(rm_abs[::10], beta_abs[::10], s=1, alpha=0.3, c='blue')
        
        # Draw threshold rectangles
        ax.axvline(25, color='r', linestyle='--', linewidth=1, label='rm_HIGH=25')
        ax.axhline(10, color='g', linestyle='--', linewidth=1, label='beta_HIGH=10')
        ax.axhline(2, color='orange', linestyle='--', linewidth=1, label='beta=2')
        ax.axhline(1, color='purple', linestyle=':', linewidth=1, label='beta=1')
        
        # Highlight AND region (rm>25 AND beta>2)
        ax.axvspan(25, ax.get_xlim()[1] if ax.get_xlim()[1] > 25 else 150, 
                   ymin=0, ymax=1, alpha=0.05, color='red')
        
        ax.set_xlabel('|rm| [N]')
        ax.set_ylabel('|beta| [Nm]')
        ax.set_title(label)
        ax.legend(fontsize=7, loc='upper right')
        ax.set_xlim(0, 160)
        ax.set_ylim(0, 12)
        ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    fig.savefig(out_dir / "rm_beta_scatter.png", dpi=150)
    print(f"Saved rm_beta_scatter.png")
    plt.close(fig)
    
    # =================================================================
    # Summary table
    # =================================================================
    print("\n" + "=" * 90)
    print("SUMMARY TABLE")
    print("=" * 90)
    print(f"{'Config':<30} {'RMSE_vx':>10} {'Avg Legs':>10} {'Coverage':>10} {'a':>6} {'b':>6} {'c':>6} {'d':>6}")
    print("-" * 90)
    for res in sorted(results, key=lambda r: r['rmse_vx']):
        p = res['per_leg']
        print(f"{res['label']:<30} {res['rmse_vx']*1000:>8.1f}mm {res['mean_n_contact']:>10.2f} "
              f"{res['contact_ratio']*100:>9.1f}% {p['a']:>5.0%} {p['b']:>5.0%} {p['c']:>5.0%} {p['d']:>5.0%}")
    print("\nSchmitt Trigger Thresholds (absolute value):")
    for cfg_label, rm_h, rm_l, beta_h, beta_l, use_and in configs:
        logic = "AND" if use_and else "OR"
        print(f"  {cfg_label}: logic={logic}")
        print(f"    |rm|  → HIGH (activate)={rm_h}, LOW (deactivate)={rm_l}")
        print(f"    |beta|→ HIGH (activate)={beta_h}, LOW (deactivate)={beta_l}")
    print("=" * 90)
    
    print(f"\nAll plots saved to: {out_dir}/")


if __name__ == '__main__':
    main()
