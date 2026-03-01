#!/usr/bin/env python3
"""
compare_velocity_methods.py

Compare ground truth hip velocity with two kinematic estimation methods:
1. Average velocity (legacy Information Filter approach)
   - Compute contact-point displacement over a window of J steps
   - v_avg = displacement / (J * dt)
2. Instantaneous velocity (ES-EKF approach)
   - Use PointVelocity(v=0, w) at each timestep to get contact velocity
   - v_inst = -contact_velocity(v=0)

CSV input columns:  time, theta, beta, hip_vx, hip_vz
The robot is assumed to be in the ground-contact phase for the entire file,
so no need to handle flight phase exclusion.

Author: debug helper for corgi_odometry
"""

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")          # headless backend – works even without a display
import matplotlib.pyplot as plt
from pathlib import Path

# =====================================================================
# 1. Link-leg mechanism parameters (from LinkLegModel / Leg)
# =====================================================================
R_wheel  = 0.10           # big rim radius [m]  (from Config.hpp)
r_tire   = 0.0125         # tire skin radius [m]
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

# Leg offset (assume single front-left leg for test)
offset = np.array([0.222, 0.193, 0.0])   # from Config.hpp

# =====================================================================
# 2. Forward kinematics  (Python port of LinkLegModel::calculate + Leg)
# =====================================================================

def D_phi(theta, theta_d, l1, l3):
    k = l1 / l3
    phi = np.arcsin(k * np.sin(theta))
    ph = k * np.cos(theta) / np.cos(np.arcsin(k * np.sin(theta)))
    phi_d = ph * theta_d
    return phi, phi_d

def linkleg_calculate(theta, theta_d):
    """
    Port of LinkLegModel::calculate + derivative functions.
    Returns dict with joint positions (complex) and velocities.
    """
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
    # D_theta2
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

    # D_rho
    num_rho = R_wheel*np.cos(theta)*theta_d + l8*to_d*np.cos(to_ + tf)
    den_sq_rho = l10**2 - (R_wheel*np.sin(theta) + l8*np.sin(to_ + tf))**2
    rho_d = num_rho / np.sqrt(den_sq_rho) if den_sq_rho > 0 else 0.0

    G   = F - l10 * np.exp(1j * rho)
    O2  = G + R_wheel * np.exp(1j * (rho + to2))

    # Derivatives (complex velocities)
    O1_d_c = (R_wheel * theta_d * np.exp(1j*(theta + np.pi/2)) +
              R_wheel * to_d * np.exp(1j*(to_ + to1 + np.pi/2)))
    G_d_c  = (R_wheel * theta_d * np.exp(1j*(theta + np.pi/2)) +
              l8 * to_d * np.exp(1j*(to_ + tf + np.pi/2)) -
              l10 * rho_d * np.exp(1j*(rho + np.pi/2)))
    O2_d_c = G_d_c + R_wheel * rho_d * np.exp(1j*(rho + to2 + np.pi/2))

    # Symmetry (left-side mirror: complex conjugate)
    O1_ = np.conj(O1)
    O2_ = np.conj(O2)
    O1_d_c_ = np.conj(O1_d_c)
    O2_d_c_ = np.conj(O2_d_c)

    # Rim angular velocities (from symmetry())
    O1_w  =  to_d            # = -theta2_d + epsilon_d
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
    """
    Port of Leg::Calculate.
    Applies beta rotation to the link-leg model.
    Returns updated joint dict.
    """
    fk = linkleg_calculate(theta, theta_d)
    rot  = np.exp(1j * beta)
    rot_v = beta_d * np.exp(1j * (beta + np.pi/2))

    # Rotate positions
    for key in ['O1', 'O2', 'G', 'O1_', 'O2_']:
        fk[key] = rot * fk[key]
    fk['F']  = rot * fk['F']

    # Rotate velocities and add rotation contribution
    fk['O1_d'] = rot * fk['O1_d'] + rot_v * (fk['O1'] / rot * rot)  # note: fk[O1] is already rotated
    # Correction: We need the original (pre-rotation) positions.
    # Let's redo this properly.

    # Re-do: first get un-rotated from linkleg, then apply rotation
    fk0 = linkleg_calculate(theta, theta_d)
    
    for key in ['O1', 'O2', 'G', 'O1_', 'O2_']:
        orig = fk0[key]
        fk[key] = rot * orig
    fk['F'] = rot * fk0['F']

    fk['O1_d'] = rot * fk0['O1_d'] + rot_v * fk0['O1']
    fk['O1_d_'] = rot * fk0['O1_d_'] + rot_v * fk0['O1_']
    fk['O2_d'] = rot * fk0['O2_d'] + rot_v * fk0['O2']
    fk['O2_d_'] = rot * fk0['O2_d_'] + rot_v * fk0['O2_']
    fk['G_d'] = rot * fk0['G_d'] + rot_v * fk0['G']

    # Rim angular velocities += beta_d
    fk['O1_w'] = fk0['O1_w'] + beta_d
    fk['O1_w_'] = fk0['O1_w_'] + beta_d
    fk['O2_w'] = fk0['O2_w'] + beta_d
    fk['O2_w_'] = fk0['O2_w_'] + beta_d

    fk['beta'] = beta
    fk['beta_d'] = beta_d

    return fk


# =====================================================================
# 3. ContactMap: lookup which rim is in contact
# =====================================================================

def rad_mod2(rad):
    """Wrap angle to (0, 2π]"""
    while rad > 2*np.pi:
        rad -= 2*np.pi
    while rad <= 0:
        rad += 2*np.pi
    return rad

def b1(theta_deg):
    t = theta_deg
    return (-2.61019580e-09*t**5 + 1.24181267e-06*t**4
            - 2.24183011e-04*t**3 + 1.78431692e-02*t**2
            - 1.33151836e-01*t - 1.78362899e+00)

def b2(theta_deg):
    t = theta_deg
    return (-1.22581785e-09*t**5 + 5.02932993e-07*t**4
            - 7.37114643e-05*t**3 + 6.47617996e-03*t**2
            - 3.31750539e-01*t + 5.40846840e+01)

def b3(theta_deg):
    t = theta_deg
    return (-4.87190741e-07*t**5 + 3.21347467e-04*t**4
            - 8.40604260e-02*t**3 + 1.09041600e+01*t**2
            - 7.02946587e+02*t + 1.82438639e+04)

def contact_lookup(theta_rad, contact_beta_rad):
    """
    Port of ContactMap::lookup.
    Returns RIM enum as string.
    """
    beta = rad_mod2(contact_beta_rad)
    theta_deg = np.degrees(theta_rad)
    beta_deg  = np.degrees(beta)

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
# 4. Contact point position & velocity
# =====================================================================

def complex_to_xz(c):
    """Convert complex number to [x, z] (imag → x, real → z)."""
    return np.array([c.imag, c.real])

def rim_center_pos(fk, rim):
    """Get rim center position in body frame [x, y, z]."""
    mapper = {
        'G_POINT': 'G',
        'UPPER_RIM_R': 'O1',
        'LOWER_RIM_R': 'O2',
        'LOWER_RIM_L': 'O2_',
        'UPPER_RIM_L': 'O1_',
    }
    key = mapper.get(rim, None)
    if key is None:
        return offset.copy()
    c = fk[key]
    return offset + np.array([c.imag, 0.0, c.real])

def rim_center_vel(fk, rim):
    """Get rim center velocity [vx, 0, vz] in body frame."""
    mapper = {
        'G_POINT': 'G_d',
        'UPPER_RIM_R': 'O1_d',
        'LOWER_RIM_R': 'O2_d',
        'LOWER_RIM_L': 'O2_d_',
        'UPPER_RIM_L': 'O1_d_',
    }
    key = mapper.get(rim, None)
    if key is None:
        return np.array([0., 0., 0.])
    c = fk[key]
    return np.array([c.imag, 0.0, c.real])

def rim_omega(fk, rim, y_offset):
    """Get the rim angular velocity (about y-axis) for the given rim.
    y_offset: the y-component of the leg offset (determines left/right).
    """
    if rim == 'G_POINT':
        if y_offset < 0:
            return fk['O2_w_']
        else:
            return fk['O2_w']
    elif rim == 'UPPER_RIM_R':
        return fk['O1_w']
    elif rim == 'LOWER_RIM_R':
        return fk['O2_w']
    elif rim == 'LOWER_RIM_L':
        return fk['O2_w_']
    elif rim == 'UPPER_RIM_L':
        return fk['O1_w_']
    else:
        return 0.0

def rim_radius(rim):
    if rim == 'G_POINT':
        return r_tire
    elif rim == 'NO_CONTACT':
        return 0.0
    else:
        return r_tire + R_wheel


def contact_point(fk, rim, alpha):
    """
    Port of Leg::PointContact.
    Returns contact_point [x, y, z] in body frame.
    """
    r = rim_radius(rim)
    rim_p = r * np.exp(1j * (np.pi + alpha))    # complex offset
    center = rim_center_pos(fk, rim)
    return center + np.array([rim_p.imag, 0.0, rim_p.real])


def contact_velocity(fk, rim, alpha, v_body, w_body, y_offset):
    """
    Port of Leg::PointVelocity  (inbody_coord=True).
    
    contact_velocity = v + w × contact_pt + rim_center_vel + w_rim × r_contact

    Parameters
    ----------
    v_body : (3,) body velocity
    w_body : (3,) body angular velocity [rad/s]
    """
    r = rim_radius(rim)
    cp = contact_point(fk, rim, alpha)
    rv = rim_center_vel(fk, rim)
    link_w = rim_omega(fk, rim, y_offset)

    rim_p = r * np.exp(1j * (np.pi + alpha))
    w_rim = np.array([0.0, link_w, 0.0])
    r_rim = np.array([rim_p.imag, 0.0, rim_p.real])

    cv = v_body + np.cross(w_body, cp) + rv + np.cross(w_rim, r_rim)
    return cv


# =====================================================================
# 5. Average velocity (legacy approach)
# =====================================================================

def travel_continuous(rim, beta_from_contact, beta_to_contact, beta_motor):
    """
    Port of ContactMap::travel_contineous.
    Arc-length displacement from contact_beta change.
    """
    r = rim_radius(rim)
    dbeta = beta_to_contact - beta_from_contact
    # wrap
    if dbeta > np.pi:
        dbeta -= 2 * np.pi
    if dbeta < -np.pi:
        dbeta += 2 * np.pi
    angle_body = beta_from_contact - beta_motor
    if angle_body > np.pi:
        angle_body -= 2 * np.pi
    if angle_body < -np.pi:
        angle_body += 2 * np.pi
    d = r * dbeta
    body_travel = np.array([np.cos(angle_body) * d, 0.0, np.sin(-angle_body) * d])
    return body_travel


def compensate_rolling(fk, rim, alpha, theta_d_val, dt, y_offset):
    """
    Port of ContactMap::compensate2 for a single step.
    Rolling compensation from theta_d (link mechanism angular velocity).
    """
    link_w = rim_omega(fk, rim, y_offset)
    r = rim_radius(rim)
    rolling_distance = r * link_w * dt     # simplified: single step
    angle_body = alpha  # contact_beta - beta = alpha
    body_travel = np.array([np.cos(angle_body) * rolling_distance, 0.0, np.sin(-angle_body) * rolling_distance])
    return body_travel


def compute_average_velocity(times, thetas, betas, dt, J):
    """
    Emulate the legacy Information Filter observation.

    For each timestep k (k >= J):
      - Look at the window [k-J .. k]
      - Compute cumulative displacement from contact-point arc-length (travel)
      - Add rolling compensation
      - Subtract IMU displacement (here we assume w=0, a=0, so dz_imu=0)
      - v_avg ≈ displacement / (J * dt)

    For simplicity (no IMU rotation in this test), we skip the rotation
    chain and just accumulate the travel + compensate terms.

    Returns
    -------
    v_avg_x, v_avg_z : arrays of length len(times)
        Average estimated hip velocity at each timestep.
        First J entries are NaN (insufficient window).
    """
    N = len(times)
    v_avg_x = np.full(N, np.nan)
    v_avg_z = np.full(N, np.nan)

    # Pre-compute theta_d from finite differences (as in legacy encoder)
    theta_d_arr = np.zeros(N)
    for i in range(1, N):
        theta_d_arr[i] = (thetas[i] - thetas[i-1]) / dt
    
    # Pre-compute beta_d from finite differences
    beta_d_arr = np.zeros(N)
    for i in range(1, N):
        beta_d_arr[i] = (betas[i] - betas[i-1]) / dt

    # Accumulate contact_beta for w=0 case
    # contact_beta += (beta_d + w_y) * dt ≈ beta  (since w_y=0)
    # Actually in legacy: contact_beta = (encoders(2) + encoders(3)) * dt + prev_contact_beta
    # where encoders(2) = beta_d, encoders(3) = w_y
    # Since w_y = 0 for this test, contact_beta ≈ beta (offset by initial)
    contact_betas = np.zeros(N)
    contact_betas[0] = betas[0]       # initial
    for i in range(1, N):
        w_y = 0.0  # no IMU rotation in this test
        contact_betas[i] = contact_betas[i-1] + (beta_d_arr[i] + w_y) * dt

    for k in range(J, N):
        displacement = np.zeros(3)
        for i in range(k - J, k):
            rim_i = contact_lookup(thetas[i], contact_betas[i])
            rim_i1 = contact_lookup(thetas[i+1], contact_betas[i+1])

            if rim_i == 'NO_CONTACT' or rim_i1 == 'NO_CONTACT':
                break

            # Travel (arc-length from contact_beta change)
            t_step = travel_continuous(rim_i, contact_betas[i], contact_betas[i+1], betas[i])
            displacement += t_step

            # Compensate (rolling from theta_d)
            fk_i = leg_calculate(thetas[i], theta_d_arr[i], betas[i], beta_d_arr[i])
            alpha_i = contact_betas[i] - betas[i]
            c_step = compensate_rolling(fk_i, rim_i, alpha_i, theta_d_arr[i], dt, offset[1])
            displacement += c_step

        # Also add contact point difference (first_point - last_point)
        # In legacy: d = t + first_point - last_point + c - u->dz(dt)
        # first point
        rim_first = contact_lookup(thetas[k-J], contact_betas[k-J])
        fk_first = leg_calculate(thetas[k-J], 0, betas[k-J], 0)
        alpha_first = contact_betas[k-J] - betas[k-J]
        first_pt = contact_point(fk_first, rim_first, alpha_first)

        # last point (rotated by accumulated rotation → identity for w=0)
        rim_last = contact_lookup(thetas[k], contact_betas[k])
        fk_last = leg_calculate(thetas[k], 0, betas[k], 0)
        alpha_last = contact_betas[k] - betas[k]
        last_pt = contact_point(fk_last, rim_last, alpha_last)

        displacement += first_pt - last_pt

        # v_avg = displacement / (J * dt)
        # In legacy the C matrix has dt factors and the filter divides by J
        # Logged as: 1.0 / dt / J * z(dt)
        v_avg = displacement / (J * dt)
        v_avg_x[k] = v_avg[0]
        v_avg_z[k] = v_avg[2]

    return v_avg_x, v_avg_z


# =====================================================================
# 6. Instantaneous velocity (ES-EKF approach)
# =====================================================================

def compute_instantaneous_velocity(times, thetas, betas, dt):
    """
    Emulate ESEKF::update_leg observation.

    At each timestep:
      1. Compute theta_d, beta_d from finite differences
      2. Run forward kinematics (Leg::Calculate)
      3. PointContact(rim, alpha)
      4. PointVelocity(v=0, w=0, rim, alpha, inbody=True)
      5. z_leg = -contact_velocity

    Returns v_inst_x, v_inst_z arrays.
    """
    N = len(times)
    v_inst_x = np.full(N, np.nan)
    v_inst_z = np.full(N, np.nan)

    # Pre-compute derivatives
    theta_d_arr = np.zeros(N)
    beta_d_arr = np.zeros(N)
    for i in range(1, N):
        theta_d_arr[i] = (thetas[i] - thetas[i-1]) / dt
        beta_d_arr[i]  = (betas[i]  - betas[i-1])  / dt

    # Accumulate contact_beta
    contact_betas = np.zeros(N)
    contact_betas[0] = betas[0]
    for i in range(1, N):
        w_y = 0.0
        contact_betas[i] = contact_betas[i-1] + (beta_d_arr[i] + w_y) * dt

    for i in range(1, N):
        fk = leg_calculate(thetas[i], theta_d_arr[i], betas[i], beta_d_arr[i])
        rim = contact_lookup(thetas[i], contact_betas[i])
        if rim == 'NO_CONTACT':
            continue
        alpha = contact_betas[i] - betas[i]

        v_zero = np.array([0.0, 0.0, 0.0])
        w_zero = np.array([0.0, 0.0, 0.0])  # w=0 for this test
        cv = contact_velocity(fk, rim, alpha, v_zero, w_zero, offset[1])

        z_leg = -cv
        v_inst_x[i] = z_leg[0]
        v_inst_z[i] = z_leg[2]

    return v_inst_x, v_inst_z


# =====================================================================
# 7. Main: load CSV, compute, plot
# =====================================================================

def main():
    csv_path = Path(__file__).parent / "single_leg_contact_theta_beta.csv"
    if not csv_path.exists():
        raise FileNotFoundError(f"CSV not found: {csv_path}")

    df = pd.read_csv(csv_path)
    times  = df['time'].values
    thetas = df['theta'].values
    betas  = -df['beta'].values          # negate beta to match body-frame convention
    gt_vx  = df['hip_vx'].values
    gt_vz  = df['hip_vz'].values

    dt = times[1] - times[0]             # should be ~0.001s = 1 kHz
    print(f"Loaded {len(times)} samples, dt = {dt:.6f} s")

    # --- Average velocity with different window sizes ---
    J_values = [5, 10, 20]
    avg_results = {}
    for J in J_values:
        print(f"Computing average velocity (J={J}) ...")
        vx, vz = compute_average_velocity(times, thetas, betas, dt, J)
        avg_results[J] = (vx, vz)

    # --- Instantaneous velocity ---
    print("Computing instantaneous velocity ...")
    v_inst_x, v_inst_z = compute_instantaneous_velocity(times, thetas, betas, dt)

    # --- Plot ---
    fig, axes = plt.subplots(2, 1, figsize=(14, 10), sharex=True)
    fig.suptitle("Velocity Estimation Comparison: Average vs Instantaneous\n"
                 "(Single Leg Contact Phase, no IMU rotation)", fontsize=14)

    # --- Vx plot ---
    ax = axes[0]
    ax.plot(times, gt_vx, 'k-', linewidth=2, label='Ground Truth', alpha=0.8)
    colors = ['C0', 'C1', 'C2']
    for idx, J in enumerate(J_values):
        vx, vz = avg_results[J]
        ax.plot(times, vx, color=colors[idx], linestyle='--', linewidth=1.2,
                label=f'Average (J={J})', alpha=0.8)
    ax.plot(times, v_inst_x, 'r-', linewidth=1.0, label='Instantaneous', alpha=0.7)
    ax.set_ylabel("$v_x$ [m/s]")
    ax.set_title("X-direction Velocity (forward)")
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)

    # --- Vz plot ---
    ax = axes[1]
    ax.plot(times, gt_vz, 'k-', linewidth=2, label='Ground Truth', alpha=0.8)
    for idx, J in enumerate(J_values):
        vx, vz = avg_results[J]
        ax.plot(times, vz, color=colors[idx], linestyle='--', linewidth=1.2,
                label=f'Average (J={J})', alpha=0.8)
    ax.plot(times, v_inst_z, 'r-', linewidth=1.0, label='Instantaneous', alpha=0.7)
    ax.set_ylabel("$v_z$ [m/s]")
    ax.set_xlabel("Time [s]")
    ax.set_title("Z-direction Velocity (vertical)")
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out_path = Path(__file__).parent / "velocity_comparison.png"
    plt.savefig(out_path, dpi=150)
    print(f"\nPlot saved to: {out_path}")

    # --- Print sample values for debugging ---
    print("\n" + "="*60)
    print("Sample values at mid-point (index 1200)")
    print("="*60)
    mid = 1200
    print(f"  Ground truth:   vx = {gt_vx[mid]:.6f},  vz = {gt_vz[mid]:.6f}")
    print(f"  Instantaneous:  vx = {v_inst_x[mid]:.6f},  vz = {v_inst_z[mid]:.6f}")
    for J in J_values:
        vx, vz = avg_results[J]
        if not np.isnan(vx[mid]):
            print(f"  Average(J={J:>3d}):  vx = {vx[mid]:.6f},  vz = {vz[mid]:.6f}")

    # --- Print error statistics ---
    print("\n" + "="*60)
    print("Error Statistics (RMS error vs ground truth)")
    print("="*60)

    valid_inst = ~np.isnan(v_inst_x)
    rmse_inst_x = np.sqrt(np.nanmean((v_inst_x[valid_inst] - gt_vx[valid_inst])**2))
    rmse_inst_z = np.sqrt(np.nanmean((v_inst_z[valid_inst] - gt_vz[valid_inst])**2))
    print(f"Instantaneous:       RMSE_vx = {rmse_inst_x:.6f} m/s,  RMSE_vz = {rmse_inst_z:.6f} m/s")

    for J in J_values:
        vx, vz = avg_results[J]
        valid = ~np.isnan(vx)
        rmse_vx = np.sqrt(np.nanmean((vx[valid] - gt_vx[valid])**2))
        rmse_vz = np.sqrt(np.nanmean((vz[valid] - gt_vz[valid])**2))
        print(f"Average (J={J:>3d}):     RMSE_vx = {rmse_vx:.6f} m/s,  RMSE_vz = {rmse_vz:.6f} m/s")

    # --- Mean value comparison ---
    print("\n" + "="*60)
    print("Mean estimated velocity (should be ≈ GT mean)")
    print("="*60)
    print(f"  Ground truth mean:  vx = {np.mean(gt_vx):.6f},  vz = {np.mean(gt_vz):.6f}")
    print(f"  Instantaneous mean: vx = {np.nanmean(v_inst_x):.6f},  vz = {np.nanmean(v_inst_z):.6f}")
    for J in J_values:
        vx, vz = avg_results[J]
        print(f"  Average(J={J:>3d}) mean: vx = {np.nanmean(vx):.6f},  vz = {np.nanmean(vz):.6f}")

    # --- Additional diagnostic: plot instantaneous velocity error over time ---
    fig2, axes2 = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    fig2.suptitle("Velocity Estimation Error vs Ground Truth", fontsize=14)

    ax = axes2[0]
    ax.plot(times[valid_inst], v_inst_x[valid_inst] - gt_vx[valid_inst], 'r-', linewidth=0.8, label='Instantaneous err', alpha=0.7)
    for idx, J in enumerate(J_values):
        vx, _ = avg_results[J]
        valid = ~np.isnan(vx)
        ax.plot(times[valid], vx[valid] - gt_vx[valid], color=colors[idx], linestyle='--', linewidth=0.8, label=f'Avg (J={J}) err', alpha=0.7)
    ax.set_ylabel("$\\Delta v_x$ [m/s]")
    ax.set_title("X-direction Velocity Error")
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    ax.axhline(0, color='k', linewidth=0.5)

    ax = axes2[1]
    ax.plot(times[valid_inst], v_inst_z[valid_inst] - gt_vz[valid_inst], 'r-', linewidth=0.8, label='Instantaneous err', alpha=0.7)
    for idx, J in enumerate(J_values):
        _, vz = avg_results[J]
        valid = ~np.isnan(vz)
        ax.plot(times[valid], vz[valid] - gt_vz[valid], color=colors[idx], linestyle='--', linewidth=0.8, label=f'Avg (J={J}) err', alpha=0.7)
    ax.set_ylabel("$\\Delta v_z$ [m/s]")
    ax.set_xlabel("Time [s]")
    ax.set_title("Z-direction Velocity Error")
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    ax.axhline(0, color='k', linewidth=0.5)

    plt.tight_layout()
    out_path2 = Path(__file__).parent / "velocity_error.png"
    plt.savefig(out_path2, dpi=150)
    print(f"Error plot saved to: {out_path2}")


if __name__ == "__main__":
    main()
