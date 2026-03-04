#!/usr/bin/env python3
"""
EKF Error Source Analysis (0–10 s)
===================================
Decomposes body-frame velocity estimation error into three sources:

  1. IMU-only dead-reckoning (predict without leg updates)
  2. Pure leg observation z_leg  (encoder FK + **GT** pitch rate)
  3. ESEKF z_leg observation     (encoder FK + **estimated** pitch rate)

Comparison target: GT body-frame velocity derived from sim_pos finite
difference → 10 Hz IIR LPF → R_GT^T * v_world.

Data: walk_3m_01m  (simulation, 0.1 m/s command, 1 kHz)
      Analysis window: 12 s after START_INDEX (raw 5000-17000), eval raw 7000-17000

Author: ekf_error_analysis (2026-03-03)
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
DT            = 0.001          # 1 kHz
START_INDEX   = 5000           # ESEKF starts here
ANALYSIS_SEC  = 12.0           # 12s total, eval raw 7000-17000 (first 10s of motion)
N_ANALYSIS    = int(ANALYSIS_SEC / DT)   # 12 000 samples
WARMUP        = 2000           # skip first 2 s for LPF settling (eval starts at raw 7000)

WHEEL_RADIUS  = 0.10
TIRE_RADIUS   = 0.019
LEG_X_OFFSET  = 0.222
LEG_Y_OFFSET  = 0.193
LEG_Z_OFFSET  = 0.0

# RIM enum
NO_CONTACT   = 0
UPPER_RIM_L  = 1
LOWER_RIM_L  = 2
G_POINT      = 3
LOWER_RIM_R  = 4
UPPER_RIM_R  = 5

# =====================================================================
# Quaternion helpers
# =====================================================================
def quat_to_rotmat(qw, qx, qy, qz):
    return np.array([
        [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
        [2*(qx*qy+qz*qw),   1-2*(qx*qx+qz*qz),  2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw),     1-2*(qx*qx+qy*qy)]])

def quat_to_pitch(qw, qx, qy, qz):
    """Extract pitch (Y-axis, intrinsic) from quaternion."""
    sinp = 2.0 * (qw * qy - qz * qx)
    return np.arcsin(np.clip(sinp, -1.0, 1.0))

# =====================================================================
# LinkLegModel  (ported from C++ LinkLegModel.cpp)
# =====================================================================
class LinkLegModel:
    def __init__(self, r=0.019, R=0.1):
        self.R  = R
        self.r  = r
        self.l1 = 0.8  * R
        self.l2 = 0.9  * R
        self.l3 = 1.3  * R
        self.l4 = 0.4  * R
        self.l5 = 0.88296634 * R
        self.l6 = 0.2  * R
        self.l7 = 2.0 * R * np.sin(np.radians(101.0 / 2.0))
        self.l8 = 2.0 * R * np.sin(np.radians(113.0 / 2.0))
        self.l9 = 2.0 * R * np.sin(np.radians( 17.0 / 2.0))
        self.l10= 2.0 * R * np.sin(np.radians( 50.0 / 2.0))
        self.to1 = np.radians(39.5)
        self.to2 = -np.radians(65.0)
        self.tf  = np.radians(6.0)
        self.th_const = np.radians(121.0)
        # outputs
        self.O1 = 0j;  self.O2 = 0j;  self.O1_ = 0j; self.O2_ = 0j
        self.G  = 0j
        self.O1_d = 0j; self.O2_d = 0j; self.O1_d_ = 0j; self.O2_d_ = 0j
        self.G_d = 0j
        self.F = 0j; self.F_ = 0j; self.H_pt = 0j; self.H_pt_ = 0j
        self.O1_w = 0.0; self.O2_w = 0.0; self.O1_w_ = 0.0; self.O2_w_ = 0.0

    def calculate(self, theta, theta_d, theta_dd=0.0):
        self.theta   = theta
        self.theta_d = theta_d
        self.theta_dd= theta_dd

        # phi
        self.phi = np.arcsin(self.l1 * np.sin(theta) / self.l3)
        k = self.l1 / self.l3
        denom = np.cos(k * np.sin(theta))
        if abs(denom) < 1e-15: denom = 1e-15
        ph = k * np.cos(theta) / denom
        self.phi_d = ph * theta_d

        # oe
        self.oe = self.l1 * np.cos(theta) - self.l3 * np.cos(self.phi)
        self.oe_d = (-self.l1 * np.sin(theta) * theta_d
                     + self.l3 * np.sin(self.phi) * self.phi_d)

        # db
        db_2 = (self.l2**2 + self.l6**2
                - 2*self.l2*self.l6*np.cos(np.pi - theta + self.phi))
        self.db = np.sqrt(max(db_2, 1e-20))
        db2_sin = self.l2 * self.l6 * np.sin(np.pi - theta + self.phi)
        self.db_d = db2_sin * (-theta_d + self.phi_d) / self.db

        # epsilon
        self.epsilon = np.arctan2(
            self.l2*np.sin(self.phi) + self.l6*np.sin(theta),
            self.l2*np.cos(self.phi) + self.l6*np.cos(theta))
        db2v = max(self.db**2, 1e-20)
        self.epsilon_d = (self.l2**2*self.phi_d + self.l6**2*theta_d
            + self.l2*self.l6*np.cos(self.phi-theta)*(self.phi_d+theta_d)) / db2v

        # theta2
        cos_t2 = np.clip((db_2 + self.l7**2 - self.l5**2)
                         / (2*self.db*self.l7), -1, 1)
        self.theta2 = np.arccos(cos_t2)
        # D_theta2
        num_part = (2*self.db*self.l7)**2
        num = self.db_d*self.l7*(4*db_2 - 2*(db_2 + self.l7**2 - self.l5**2))
        den_sq = num_part**2 + (db_2+self.l7**2-self.l5**2)**2 * num_part
        self.theta2_d = num / np.sqrt(max(den_sq, 1e-30))

        # to
        self.to = np.pi - self.theta2 + self.epsilon

        # B, O1, F, H
        self.B  = self.R * np.exp(1j*theta)
        self.O1 = self.B + self.R * np.exp(1j*(self.to + self.to1))
        self.F  = self.B + self.l8 * np.exp(1j*(self.to + self.tf))
        self.H_pt = self.B + self.l9 * np.exp(1j*(self.to + self.th_const))

        # D_O1
        self.O1_d = (self.R*theta_d*np.exp(1j*(theta + np.pi/2))
                   + self.R*(self.epsilon_d-self.theta2_d)
                     *np.exp(1j*(self.to+self.to1+np.pi/2)))

        # rho
        sinarg = (self.R*np.sin(theta) + self.l8*np.sin(self.to+self.tf))
        self.rho = np.arcsin(np.clip(sinarg / self.l10, -1, 1))
        val = sinarg
        den_sq2 = max(self.l10**2 - val**2, 1e-20)
        num_rho = (self.R*np.cos(theta)*theta_d
                 + self.l8*(-self.theta2_d+self.epsilon_d)*np.cos(self.to+self.tf))
        self.rho_d = num_rho / np.sqrt(den_sq2)

        # G
        self.G  = self.F - self.l10 * np.exp(1j*self.rho)
        self.G_d = (self.R*theta_d*np.exp(1j*(theta+np.pi/2))
                  + self.l8*(self.epsilon_d-self.theta2_d)
                    *np.exp(1j*(self.to+self.tf+np.pi/2))
                  - self.l10*self.rho_d*np.exp(1j*(self.rho+np.pi/2)))

        # O2
        self.O2  = self.G + self.R * np.exp(1j*(self.rho + self.to2))
        self.O2_d = self.G_d + self.R*self.rho_d*np.exp(1j*(self.rho+self.to2+np.pi/2))

        # symmetry
        self.O2_  = np.conj(self.O2)
        self.O2_d_= np.conj(self.O2_d)
        self.O1_  = np.conj(self.O1)
        self.O1_d_= np.conj(self.O1_d)
        self.F_   = np.conj(self.F)
        self.H_pt_= np.conj(self.H_pt)

        self.O1_w  = -self.theta2_d + self.epsilon_d
        self.O1_w_ = -self.O1_w
        self.O2_w  =  self.rho_d
        self.O2_w_ = -self.O2_w


# =====================================================================
# Leg  (ported from C++ Leg.cpp)
# =====================================================================
class LegModel:
    def __init__(self, offset, R=0.1, r=0.019):
        self.m = LinkLegModel(r, R)
        self.offset = np.array(offset, dtype=float)
        self.R = R;  self.r = r
        self.contact_point    = np.zeros(3)
        self.contact_velocity = np.zeros(3)

    def Calculate(self, theta, theta_d, beta, beta_d):
        rot = np.exp(1j * beta)
        rot_v = beta_d * np.exp(1j * (beta + np.pi/2))
        self.m.calculate(theta, theta_d)

        # save pre-rotation values
        O1  = self.m.O1;   O1_  = self.m.O1_
        O2  = self.m.O2;   O2_  = self.m.O2_
        G   = self.m.G
        O1d = self.m.O1_d; O1d_ = self.m.O1_d_
        O2d = self.m.O2_d; O2d_ = self.m.O2_d_
        Gd  = self.m.G_d
        F   = self.m.F;  F_  = self.m.F_
        H   = self.m.H_pt; H_ = self.m.H_pt_

        # apply beta rotation
        self.m.O1   = rot * O1;    self.m.O1_  = rot * O1_
        self.m.O2   = rot * O2;    self.m.O2_  = rot * O2_
        self.m.G    = rot * G
        self.m.F    = rot * F;     self.m.F_   = rot * F_
        self.m.H_pt = rot * H;     self.m.H_pt_= rot * H_

        self.m.O1_d  = rot * O1d  + rot_v * O1
        self.m.O1_d_ = rot * O1d_ + rot_v * O1_
        self.m.O2_d  = rot * O2d  + rot_v * O2
        self.m.O2_d_ = rot * O2d_ + rot_v * O2_
        self.m.G_d   = rot * Gd   + rot_v * G

        self.m.O1_w  += beta_d
        self.m.O1_w_ += beta_d
        self.m.O2_w  += beta_d
        self.m.O2_w_ += beta_d

    def PointContact(self, rim, alpha=0.0):
        rim_radius = self.r if rim == G_POINT else self.r + self.R
        rp = rim_radius * np.exp(1j * (np.pi + alpha))
        rc = self._rim_center(rim)
        self.contact_point = self.offset + np.array([rc.imag + rp.imag,
                                                      0.0,
                                                      rc.real + rp.real])

    def PointVelocity(self, v, w, rim, alpha=0.0):
        """Compute contact-point velocity (body-frame)."""
        rim_radius = self.r if rim == G_POINT else self.r + self.R
        rp_c = rim_radius * np.exp(1j * (np.pi + alpha))
        rp_vec = np.array([rp_c.imag, 0.0, rp_c.real])
        rv = self._rim_center_vel(rim)
        rv_vec = np.array([rv.imag, 0.0, rv.real])
        lw = self._link_w(rim)
        w_rim = np.array([0.0, lw, 0.0])
        self.contact_velocity = (v + np.cross(w, self.contact_point)
                                 + rv_vec + np.cross(w_rim, rp_vec))
        return self.contact_velocity

    # ---- helpers ----
    def _rim_center(self, rim):
        if   rim == G_POINT:     return self.m.G
        elif rim == UPPER_RIM_R: return self.m.O1
        elif rim == LOWER_RIM_R: return self.m.O2
        elif rim == LOWER_RIM_L: return self.m.O2_
        elif rim == UPPER_RIM_L: return self.m.O1_
        return 0j

    def _rim_center_vel(self, rim):
        if   rim == G_POINT:     return self.m.G_d
        elif rim == UPPER_RIM_R: return self.m.O1_d
        elif rim == LOWER_RIM_R: return self.m.O2_d
        elif rim == LOWER_RIM_L: return self.m.O2_d_
        elif rim == UPPER_RIM_L: return self.m.O1_d_
        return 0j

    def _link_w(self, rim):
        if rim == G_POINT:
            return self.m.O2_w_ if self.offset[1] < 0 else self.m.O2_w
        elif rim == UPPER_RIM_R: return self.m.O1_w
        elif rim == LOWER_RIM_R: return self.m.O2_w
        elif rim == LOWER_RIM_L: return self.m.O2_w_
        elif rim == UPPER_RIM_L: return self.m.O1_w_
        return 0.0


# =====================================================================
# ContactMap  (ported from C++ ContactMap.hpp)
# =====================================================================
def _b1(theta_deg):
    t = theta_deg
    return (-2.61019580e-09*t**5 + 1.24181267e-06*t**4
            - 2.24183011e-04*t**3 + 1.78431692e-02*t**2
            - 1.33151836e-01*t     - 1.78362899e+00)

def _b2(theta_deg):
    t = theta_deg
    return (-1.22581785e-09*t**5 + 5.02932993e-07*t**4
            - 7.37114643e-05*t**3 + 6.47617996e-03*t**2
            - 3.31750539e-01*t     + 5.40846840e+01)

def _b3(theta_deg):
    t = theta_deg
    return (-4.87190741e-07*t**5 + 3.21347467e-04*t**4
            - 8.40604260e-02*t**3 + 1.09041600e+01*t**2
            - 7.02946587e+02*t     + 1.82438639e+04)

def contact_map_lookup(theta_rad, beta_rad):
    """Return RIM enum given theta, beta in radians."""
    beta = beta_rad % (2.0 * np.pi)
    theta_d = np.degrees(theta_rad)
    beta_d  = np.degrees(beta)

    if theta_d > 108.3:
        if   _b1(theta_d) > beta_d:                     return G_POINT
        elif _b2(theta_d) >= beta_d:                     return LOWER_RIM_R
        elif _b3(theta_d) > beta_d:                      return UPPER_RIM_R
        elif (360 - _b3(theta_d)) > beta_d:              return NO_CONTACT
        elif (360 - _b2(theta_d)) > beta_d:              return UPPER_RIM_L
        elif (360 - _b1(theta_d)) > beta_d:              return LOWER_RIM_L
        else:                                            return G_POINT
    else:
        if   _b1(theta_d) > beta_d:                      return G_POINT
        elif _b2(theta_d) >= beta_d:                     return LOWER_RIM_R
        elif 180.0 > beta_d:                             return UPPER_RIM_R
        elif (360 - _b2(theta_d)) > beta_d:              return UPPER_RIM_L
        elif (360 - _b1(theta_d)) > beta_d:              return LOWER_RIM_L
        else:                                            return G_POINT


# =====================================================================
# z_leg computation  (for a single leg at one timestep)
# =====================================================================
def compute_z_leg_single(leg, theta, theta_d, beta, beta_d, rim, w_y):
    """Compute z_leg = -contact_velocity(v=0, w=(0,w_y,0))."""
    leg.Calculate(theta, theta_d, beta, beta_d)
    leg.PointContact(rim, alpha=0.0)
    w = np.array([0.0, w_y, 0.0])
    v = np.zeros(3)
    leg.PointVelocity(v, w, rim, alpha=0.0)
    return -leg.contact_velocity


# =====================================================================
# Main
# =====================================================================
def main():
    ws = Path(__file__).resolve().parents[5]
    output_dir = ws / "output_data"
    raw_csv    = output_dir / "walk_3m_01m.csv"
    esekf_csv  = output_dir / "walk_3m_01m_esekf.csv"
    fig_dir    = Path(__file__).resolve().parent
    print(f"Workspace: {ws}")

    # ------------------------------------------------------------------
    # Load data
    # ------------------------------------------------------------------
    print("Loading raw CSV …")
    raw = pd.read_csv(raw_csv)
    print(f"  {len(raw)} rows total")

    print("Loading ESEKF CSV …")
    ekf = pd.read_csv(esekf_csv)
    N_ekf = len(ekf)
    print(f"  {N_ekf} rows (starts at raw index {START_INDEX})")

    # Limit to first 10 s
    N = min(N_ANALYSIS, N_ekf)
    ekf = ekf.iloc[:N].reset_index(drop=True)
    # Corresponding raw data
    raw_slice = raw.iloc[START_INDEX : START_INDEX + N].reset_index(drop=True)
    t = np.arange(N) * DT
    print(f"  Analysis window: {N} samples ({N*DT:.1f} s)")

    # ------------------------------------------------------------------
    # GT velocity (body frame)
    # ------------------------------------------------------------------
    print("\nComputing GT body-frame velocity …")
    # Finite diff in world frame
    gt_vx_w = np.gradient(raw_slice.sim_pos_x.values, DT)
    gt_vy_w = np.gradient(raw_slice.sim_pos_y.values, DT)
    gt_vz_w = np.gradient(raw_slice.sim_pos_z.values, DT)
    # 10 Hz IIR LPF
    alpha_lpf = 1.0 - np.exp(-2*np.pi*10*DT)
    for i in range(1, N):
        gt_vx_w[i] = (1-alpha_lpf)*gt_vx_w[i-1] + alpha_lpf*gt_vx_w[i]
        gt_vy_w[i] = (1-alpha_lpf)*gt_vy_w[i-1] + alpha_lpf*gt_vy_w[i]
        gt_vz_w[i] = (1-alpha_lpf)*gt_vz_w[i-1] + alpha_lpf*gt_vz_w[i]
    # Rotate to body frame using GT quaternion (sim_orien)
    gt_vb = np.zeros((N, 3))
    for i in range(N):
        R_gt = quat_to_rotmat(raw_slice.sim_orien_w.iloc[i],
                               raw_slice.sim_orien_x.iloc[i],
                               raw_slice.sim_orien_y.iloc[i],
                               raw_slice.sim_orien_z.iloc[i])
        gt_vb[i] = R_gt.T @ [gt_vx_w[i], gt_vy_w[i], gt_vz_w[i]]

    # ------------------------------------------------------------------
    # GT pitch rate  (from sim GT quaternion)
    # ------------------------------------------------------------------
    print("Computing GT pitch rate …")
    gt_pitch = np.array([
        quat_to_pitch(raw_slice.sim_orien_w.iloc[i],
                       raw_slice.sim_orien_x.iloc[i],
                       raw_slice.sim_orien_y.iloc[i],
                       raw_slice.sim_orien_z.iloc[i])
        for i in range(N)])
    gt_pitch_rate = np.gradient(gt_pitch, DT)
    # 30 Hz LPF to smooth numerical derivative
    a30 = 1.0 - np.exp(-2*np.pi*30*DT)
    for i in range(1, N):
        gt_pitch_rate[i] = (1-a30)*gt_pitch_rate[i-1] + a30*gt_pitch_rate[i]

    # IMU pitch rate (raw)
    imu_wy = raw_slice.imu_ang_vel_y.values

    # ESEKF bias-corrected pitch rate
    est_wy = imu_wy - ekf.bw_y.values

    print(f"  GT  pitch rate mean: {gt_pitch_rate[WARMUP:].mean():.4f} rad/s")
    print(f"  IMU w_y        mean: {imu_wy[WARMUP:].mean():.4f} rad/s")
    print(f"  EST w_y        mean: {est_wy[WARMUP:].mean():.4f} rad/s")

    # ------------------------------------------------------------------
    # Build leg objects
    # ------------------------------------------------------------------
    legs = [
        LegModel([+LEG_X_OFFSET, +LEG_Y_OFFSET, LEG_Z_OFFSET]),  # LF (a)
        LegModel([+LEG_X_OFFSET, -LEG_Y_OFFSET, LEG_Z_OFFSET]),  # RF (b)
        LegModel([-LEG_X_OFFSET, -LEG_Y_OFFSET, LEG_Z_OFFSET]),  # RH (c)
        LegModel([-LEG_X_OFFSET, +LEG_Y_OFFSET, LEG_Z_OFFSET]),  # LH (d)
    ]
    leg_labels = ['LF(a)', 'RF(b)', 'RH(c)', 'LH(d)']

    # Encoder columns
    theta_cols = ['state_theta_a', 'state_theta_b', 'state_theta_c', 'state_theta_d']
    beta_cols  = ['state_beta_a',  'state_beta_b',  'state_beta_c',  'state_beta_d']
    vr_cols    = ['state_vel_r_a', 'state_vel_r_b', 'state_vel_r_c', 'state_vel_r_d']
    vl_cols    = ['state_vel_l_a', 'state_vel_l_b', 'state_vel_l_c', 'state_vel_l_d']
    contact_cols = ['contact_a', 'contact_b', 'contact_c', 'contact_d']

    is_right = [False, True, True, False]

    # Read contact from ESEKF CSV (already computed by Schmitt trigger + rim check)
    contacts = ekf[contact_cols].values.astype(int)                # (N, 4)

    # ==================================================================
    # Analysis 1:  IMU-only dead-reckoning
    # ==================================================================
    print("\n" + "="*65)
    print("Analysis 1: IMU Dead-Reckoning (predict only, no leg update)")
    print("="*65)

    imu_v_body = np.zeros((N, 3))   # integrated body-frame velocity
    # Use IMU quaternion for attitude (since no ESEKF correction)
    # Start from v = 0 (same as ESEKF init)
    v_b = np.zeros(3)
    for i in range(N):
        # IMU measurements
        a_m = np.array([raw_slice.imu_lin_acc_x.iloc[i],
                        raw_slice.imu_lin_acc_y.iloc[i],
                        raw_slice.imu_lin_acc_z.iloc[i]])
        w_m = np.array([raw_slice.imu_ang_vel_x.iloc[i],
                        raw_slice.imu_ang_vel_y.iloc[i],
                        raw_slice.imu_ang_vel_z.iloc[i]])
        # Rotation matrix from IMU quaternion
        R_imu = quat_to_rotmat(raw_slice.imu_orien_w.iloc[i],
                                raw_slice.imu_orien_x.iloc[i],
                                raw_slice.imu_orien_y.iloc[i],
                                raw_slice.imu_orien_z.iloc[i])
        g_world = np.array([0, 0, -9.81])
        g_body = R_imu.T @ g_world

        # Rotation increment
        dw = w_m * DT
        angle = np.linalg.norm(dw)
        if angle > 1e-8:
            axis = dw / angle
            K = np.array([[0, -axis[2], axis[1]],
                          [axis[2], 0, -axis[0]],
                          [-axis[1], axis[0], 0]])
            R_delta = np.eye(3) + np.sin(angle)*K + (1-np.cos(angle))*(K@K)
        else:
            R_delta = np.eye(3)

        # Propagate: v_{k+1} = R_delta^T * v_k + (a_m + g_body) * dt
        v_b = R_delta.T @ v_b + (a_m + g_body) * DT
        imu_v_body[i] = v_b

    imu_err = imu_v_body - gt_vb    # (N, 3)

    # RMSE (skip warmup)
    sl = slice(WARMUP, N)
    imu_rmse_x = np.sqrt(np.mean(imu_err[sl, 0]**2))
    imu_rmse_y = np.sqrt(np.mean(imu_err[sl, 1]**2))
    imu_rmse_z = np.sqrt(np.mean(imu_err[sl, 2]**2))
    imu_bias_x = np.mean(imu_err[sl, 0])
    imu_bias_z = np.mean(imu_err[sl, 2])

    print(f"  RMSE vx: {imu_rmse_x*1000:8.2f} mm/s   bias: {imu_bias_x*1000:+8.2f} mm/s")
    print(f"  RMSE vy: {imu_rmse_y*1000:8.2f} mm/s")
    print(f"  RMSE vz: {imu_rmse_z*1000:8.2f} mm/s   bias: {imu_bias_z*1000:+8.2f} mm/s")
    print(f"  At t=2s : vx={imu_v_body[WARMUP,0]*1000:.1f} mm/s (GT: {gt_vb[WARMUP,0]*1000:.1f})")
    print(f"  At t=5s : vx={imu_v_body[5000,0]*1000:.1f} mm/s (GT: {gt_vb[5000,0]*1000:.1f})")
    print(f"  At t=10s: vx={imu_v_body[-1,0]*1000:.1f} mm/s (GT: {gt_vb[-1,0]*1000:.1f})")

    # ==================================================================
    # Analysis 2 & 3:  Leg FK observations
    # ==================================================================
    print("\n" + "="*65)
    print("Analysis 2: Leg FK with GT pitch rate (GT attitude)")
    print("Analysis 3: Leg FK with ESEKF bias-corrected pitch rate")
    print("="*65)

    # Also compute z_leg_raw (with raw IMU w_y) to validate against CSV z_avg
    z_gt   = np.full((N, 3), np.nan)   # z_leg with GT w_y
    z_est  = np.full((N, 3), np.nan)   # z_leg with EST w_y
    z_raw  = np.full((N, 3), np.nan)   # z_leg with raw IMU w_y (validation)
    n_legs = np.zeros(N, dtype=int)    # number of contact legs

    for i in range(N):
        if i % 2000 == 0:
            print(f"  FK computation: {i}/{N} …", end='\r')

        sum_gt  = np.zeros(3)
        sum_est = np.zeros(3)
        sum_raw = np.zeros(3)
        nc = 0

        for j in range(4):
            if contacts[i, j] == 0:
                continue

            theta = raw_slice[theta_cols[j]].iloc[i]
            beta  = raw_slice[beta_cols[j]].iloc[i]
            vr    = raw_slice[vr_cols[j]].iloc[i]
            vl    = raw_slice[vl_cols[j]].iloc[i]

            # Sign conventions (matching offline_test.cpp)
            beta_signed = -beta if is_right[j] else beta
            theta_d = (-vr + vl) / 2.0
            beta_d  = -(vr + vl) / 2.0
            if is_right[j]:
                beta_d = -beta_d

            # RIM lookup (using theta, beta_signed in radians)
            rim = contact_map_lookup(theta, beta_signed)
            if rim == NO_CONTACT:
                continue

            # z_leg with GT w_y
            z1 = compute_z_leg_single(legs[j], theta, theta_d,
                                       beta_signed, beta_d, rim,
                                       gt_pitch_rate[i])
            sum_gt += z1

            # z_leg with EST w_y
            z2 = compute_z_leg_single(legs[j], theta, theta_d,
                                       beta_signed, beta_d, rim,
                                       est_wy[i])
            sum_est += z2

            # z_leg with raw IMU w_y (for validation)
            z3 = compute_z_leg_single(legs[j], theta, theta_d,
                                       beta_signed, beta_d, rim,
                                       imu_wy[i])
            sum_raw += z3

            nc += 1

        if nc > 0:
            z_gt[i]  = sum_gt  / nc
            z_est[i] = sum_est / nc
            z_raw[i] = sum_raw / nc
        n_legs[i] = nc

    print(f"  FK computation: {N}/{N} done.       ")

    # ------------------------------------------------------------------
    # Validation: compare z_raw vs CSV z_avg
    # ------------------------------------------------------------------
    csv_zavg = ekf[['z_avg_x','z_avg_y','z_avg_z']].values
    valid = ~np.isnan(z_raw[:,0]) & (n_legs > 0)
    diff_val = z_raw[valid] - csv_zavg[valid]
    print(f"\n  Validation (Python z_raw vs CSV z_avg):")
    print(f"    max|diff| x: {np.max(np.abs(diff_val[:,0]))*1000:.4f} mm/s")
    print(f"    max|diff| z: {np.max(np.abs(diff_val[:,2]))*1000:.4f} mm/s")
    print(f"    mean|diff|:  {np.mean(np.abs(diff_val))*1000:.4f} mm/s")

    # ------------------------------------------------------------------
    # Error statistics (skip warmup, only where contacts exist)
    # ------------------------------------------------------------------
    mask = np.zeros(N, dtype=bool)
    mask[WARMUP:] = True
    mask &= ~np.isnan(z_gt[:,0])

    err_gt  = z_gt[mask]  - gt_vb[mask]
    err_est = z_est[mask] - gt_vb[mask]
    err_raw = z_raw[mask] - gt_vb[mask]

    def stats(e, label, axis_name):
        rmse = np.sqrt(np.mean(e**2))
        bias = np.mean(e)
        std  = np.std(e)
        return f"  {label:18s} {axis_name}: RMSE={rmse*1000:7.2f}  bias={bias*1000:+7.2f}  std={std*1000:6.2f} mm/s"

    print(f"\n--- Leg FK with GT w_y (Analysis 2) ---")
    print(stats(err_gt[:,0], "z_leg(GT_wy)", "vx"))
    print(stats(err_gt[:,2], "z_leg(GT_wy)", "vz"))

    print(f"\n--- Leg FK with EST w_y (Analysis 3, ESEKF bias-corrected) ---")
    print(stats(err_est[:,0], "z_leg(EST_wy)", "vx"))
    print(stats(err_est[:,2], "z_leg(EST_wy)", "vz"))

    print(f"\n--- Leg FK with raw IMU w_y (CSV z_avg, reference) ---")
    print(stats(err_raw[:,0], "z_leg(IMU_wy)", "vx"))
    print(stats(err_raw[:,2], "z_leg(IMU_wy)", "vz"))

    # ==================================================================
    # ESEKF fused velocity error (for comparison)
    # ==================================================================
    est_vb = ekf[['est_vel_x','est_vel_y','est_vel_z']].values[:N]
    err_esekf = est_vb[mask] - gt_vb[mask]

    print(f"\n--- ESEKF fused velocity (for comparison) ---")
    print(stats(err_esekf[:,0], "ESEKF fused", "vx"))
    print(stats(err_esekf[:,2], "ESEKF fused", "vz"))

    # ==================================================================
    # Summary table
    # ==================================================================
    print("\n" + "="*75)
    print("                    SUMMARY:  vx RMSE / bias  (mm/s, after 2 s warmup)")
    print("="*75)
    headers = f"{'Source':<30s} {'RMSE':>8s} {'Bias':>8s} {'Std':>8s} {'Note':>20s}"
    print(headers)
    print("-"*75)

    def row(label, e, note=""):
        rmse = np.sqrt(np.mean(e**2))*1000
        bias = np.mean(e)*1000
        std  = np.std(e)*1000
        print(f"{label:<30s} {rmse:8.2f} {bias:+8.2f} {std:8.2f} {note:>20s}")

    row("1. IMU integration",         imu_err[WARMUP:, 0],        "dead reckoning")
    row("2. Leg FK (GT wy)",          err_gt[:,0],                 "best-case leg obs")
    row("3. Leg FK (EST wy)",         err_est[:,0],                "ESEKF w_y - bw_y")
    row("   Leg FK (raw IMU wy)",     err_raw[:,0],                "CSV z_avg")
    row("   ESEKF fused velocity",    err_esekf[:,0],              "after Kalman update")

    # vz summary
    print()
    print(f"{'Source':<30s} {'RMSE':>8s} {'Bias':>8s} {'Std':>8s}   (vz)")
    print("-"*75)
    row("1. IMU integration",         imu_err[WARMUP:, 2],        "")
    row("2. Leg FK (GT wy)",          err_gt[:,2],                 "")
    row("3. Leg FK (EST wy)",         err_est[:,2],                "")
    row("   ESEKF fused velocity",    err_esekf[:,2],              "")

    # ==================================================================
    # Angular velocity comparison
    # ==================================================================
    print("\n" + "="*65)
    print("Angular velocity (w_y) comparison  (after 2 s warmup)")
    print("="*65)
    wy_err_imu = imu_wy[WARMUP:N] - gt_pitch_rate[WARMUP:N]
    wy_err_est = est_wy[WARMUP:N] - gt_pitch_rate[WARMUP:N]
    bw_y_mean  = ekf.bw_y.values[WARMUP:N].mean()
    print(f"  IMU w_y  vs GT:  RMSE = {np.sqrt(np.mean(wy_err_imu**2))*1000:.3f} mrad/s"
          f"  bias = {np.mean(wy_err_imu)*1000:+.3f} mrad/s")
    print(f"  EST w_y  vs GT:  RMSE = {np.sqrt(np.mean(wy_err_est**2))*1000:.3f} mrad/s"
          f"  bias = {np.mean(wy_err_est)*1000:+.3f} mrad/s")
    print(f"  ESEKF bw_y mean: {bw_y_mean*1000:.4f} mrad/s")

    # ==================================================================
    # w_y × contact_point decomposition
    # ==================================================================
    # Show how much of z_leg error comes from angular velocity vs kinematic
    z_kin = np.full((N, 3), np.nan)  # z_leg with w=0 (pure kinematic)
    for i in range(N):
        if n_legs[i] == 0: continue
        sumk = np.zeros(3)
        nc = 0
        for j in range(4):
            if contacts[i, j] == 0: continue
            theta = raw_slice[theta_cols[j]].iloc[i]
            beta  = raw_slice[beta_cols[j]].iloc[i]
            vr    = raw_slice[vr_cols[j]].iloc[i]
            vl    = raw_slice[vl_cols[j]].iloc[i]
            beta_signed = -beta if is_right[j] else beta
            theta_d = (-vr + vl) / 2.0
            beta_d  = -(vr + vl) / 2.0
            if is_right[j]: beta_d = -beta_d
            rim = contact_map_lookup(theta, beta_signed)
            if rim == NO_CONTACT: continue
            zk = compute_z_leg_single(legs[j], theta, theta_d,
                                       beta_signed, beta_d, rim, 0.0)
            sumk += zk; nc += 1
        if nc > 0: z_kin[i] = sumk / nc

    mask_kin = mask & ~np.isnan(z_kin[:,0])
    err_kin = z_kin[mask_kin] - gt_vb[mask_kin]
    wr_component = z_raw[mask_kin] - z_kin[mask_kin]    # w×r contribution

    print(f"\n{'='*65}")
    print("Decomposition: z_leg = z_kinematic(w=0) + w×r_contact")
    print("="*65)
    print(f"  z_kinematic(w=0) error vx:  RMSE = {np.sqrt(np.mean(err_kin[:,0]**2))*1000:.2f} mm/s"
          f"  bias = {np.mean(err_kin[:,0])*1000:+.2f} mm/s")
    print(f"  w×r contribution vx:        mean = {np.mean(wr_component[:,0])*1000:+.2f} mm/s"
          f"  std = {np.std(wr_component[:,0])*1000:.2f} mm/s")
    print(f"  z_kinematic(w=0) error vz:  RMSE = {np.sqrt(np.mean(err_kin[:,2]**2))*1000:.2f} mm/s"
          f"  bias = {np.mean(err_kin[:,2])*1000:+.2f} mm/s")
    print(f"  w×r contribution vz:        mean = {np.mean(wr_component[:,2])*1000:+.2f} mm/s"
          f"  std = {np.std(wr_component[:,2])*1000:.2f} mm/s")

    # ==================================================================
    # Time-series plot
    # ==================================================================
    print("\nGenerating plots …")
    fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)

    # — Panel 1: vx comparison
    ax = axes[0]
    ax.plot(t, gt_vb[:, 0]*1000, 'k-', lw=1, label='GT vx')
    ax.plot(t, imu_v_body[:, 0]*1000, 'r-', lw=0.6, alpha=0.7, label='IMU integ')
    m_raw = ~np.isnan(z_raw[:,0])
    ax.plot(t[m_raw], z_raw[m_raw, 0]*1000, 'b-', lw=0.5, alpha=0.5, label='z_leg(IMU wy)')
    m_gt2 = ~np.isnan(z_gt[:,0])
    ax.plot(t[m_gt2], z_gt[m_gt2, 0]*1000, 'g-', lw=0.5, alpha=0.5, label='z_leg(GT wy)')
    ax.plot(t, est_vb[:, 0]*1000, 'm-', lw=0.6, alpha=0.7, label='ESEKF fused')
    ax.axvline(WARMUP*DT, color='gray', ls='--', lw=0.5, label='warmup end')
    ax.set_ylabel('vx (mm/s)')
    ax.set_title('Body-frame Forward Velocity (vx)')
    ax.legend(loc='upper right', fontsize=7, ncol=3)
    ax.grid(True, alpha=0.3)

    # — Panel 2: vx errors
    ax = axes[1]
    ax.plot(t[WARMUP:], imu_err[WARMUP:, 0]*1000, 'r-', lw=0.5, alpha=0.5, label='IMU integ err')
    ax.plot(t[mask], err_raw[:,0]*1000, 'b-', lw=0.4, alpha=0.4, label='z_leg(IMU) err')
    ax.plot(t[mask], err_gt[:,0]*1000, 'g-', lw=0.4, alpha=0.4, label='z_leg(GT) err')
    ax.plot(t[mask], err_esekf[:,0]*1000, 'm-', lw=0.5, alpha=0.6, label='ESEKF fused err')
    ax.axhline(0, color='k', ls='-', lw=0.3)
    ax.set_ylabel('vx error (mm/s)')
    ax.set_title('vx Estimation Error')
    ax.legend(loc='upper right', fontsize=7, ncol=2)
    ax.grid(True, alpha=0.3)

    # — Panel 3: vz comparison
    ax = axes[2]
    ax.plot(t, gt_vb[:, 2]*1000, 'k-', lw=1, label='GT vz')
    ax.plot(t, imu_v_body[:, 2]*1000, 'r-', lw=0.6, alpha=0.7, label='IMU integ')
    ax.plot(t[m_raw], z_raw[m_raw, 2]*1000, 'b-', lw=0.5, alpha=0.5, label='z_leg(IMU wy)')
    ax.plot(t, est_vb[:, 2]*1000, 'm-', lw=0.6, alpha=0.7, label='ESEKF fused')
    ax.set_ylabel('vz (mm/s)')
    ax.set_title('Body-frame Vertical Velocity (vz)')
    ax.legend(loc='upper right', fontsize=7, ncol=2)
    ax.grid(True, alpha=0.3)

    # — Panel 4: angular velocity
    ax = axes[3]
    ax.plot(t, gt_pitch_rate*1000, 'k-', lw=0.8, label='GT pitch rate')
    ax.plot(t, imu_wy*1000, 'b-', lw=0.4, alpha=0.5, label='IMU w_y')
    ax.plot(t, est_wy*1000, 'm-', lw=0.4, alpha=0.5, label='EST w_y (IMU-bw)')
    ax.set_ylabel('w_y (mrad/s)')
    ax.set_xlabel('Time (s)')
    ax.set_title('Pitch Angular Velocity')
    ax.legend(loc='upper right', fontsize=7)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    fig_path = fig_dir / "error_source_analysis.png"
    plt.savefig(fig_path, dpi=150)
    print(f"  Saved: {fig_path}")

    # ==================================================================
    # Extra Plot 1 & 2: Vx / Vz  — GT, z_leg(IMU_wy), IMU integrate
    # ==================================================================
    m_raw_full = ~np.isnan(z_raw[:, 0])

    rmse_zleg_vx  = np.sqrt(np.mean((z_raw[mask, 0] - gt_vb[mask, 0])**2)) * 1000
    rmse_imu_vx   = np.sqrt(np.mean((imu_v_body[WARMUP:, 0] - gt_vb[WARMUP:, 0])**2)) * 1000
    rmse_zleg_vz  = np.sqrt(np.mean((z_raw[mask, 2] - gt_vb[mask, 2])**2)) * 1000
    rmse_imu_vz   = np.sqrt(np.mean((imu_v_body[WARMUP:, 2] - gt_vb[WARMUP:, 2])**2)) * 1000

    print(f"\n--- Extra plots RMSE (after {WARMUP*DT:.0f}s warmup) ---")
    print(f"  Vx  z_leg(IMU_wy): {rmse_zleg_vx:7.2f} mm/s")
    print(f"  Vx  IMU integrate: {rmse_imu_vx:7.2f} mm/s")
    print(f"  Vz  z_leg(IMU_wy): {rmse_zleg_vz:7.2f} mm/s")
    print(f"  Vz  IMU integrate: {rmse_imu_vz:7.2f} mm/s")

    for axis_idx, axis_name, unit in [(0, 'Vx', 'vx [m/s]'), (2, 'Vz', 'vz [m/s]')]:
        rmse_zleg = np.sqrt(np.mean((z_raw[mask, axis_idx] - gt_vb[mask, axis_idx])**2)) * 1000
        rmse_imu  = np.sqrt(np.mean((imu_v_body[WARMUP:, axis_idx] - gt_vb[WARMUP:, axis_idx])**2)) * 1000

        fig2, ax2 = plt.subplots(figsize=(8, 6))
        ax2.plot(t, gt_vb[:, axis_idx] * 1000,
                 'k-', lw=1.2, label='GT', zorder=3)
        ax2.plot(t[m_raw_full], z_raw[m_raw_full, axis_idx] * 1000,
                 color='steelblue', lw=0.6, alpha=0.8,
                 label=f'z_leg (IMU w_y)  RMSE={rmse_zleg:.1f} mm/s')
        ax2.plot(t, imu_v_body[:, axis_idx] * 1000,
                 color='tomato', lw=0.6, alpha=0.8,
                 label=f'IMU integrate  RMSE={rmse_imu:.1f} mm/s')
        ax2.set_xlabel('Time [s]', fontsize=10)
        ax2.set_ylabel(unit, fontsize=10)
        ax2.set_title(f'{axis_name}: GT vs z_leg(IMU w_y) vs IMU integrate', fontsize=11)
        ax2.legend(fontsize=9, loc='upper left')
        ax2.grid(True, alpha=0.3)
        ax2.set_xlim(WARMUP * DT, t[-1])
        plt.tight_layout()
        out = fig_dir / f"extra_{axis_name.lower()}_comparison.png"
        plt.savefig(out, dpi=150)
        plt.close(fig2)
        print(f"  Saved: {out}")

    # ==================================================================
    # Contact-phase-aware analysis
    # ==================================================================
    # Split: stable contact (all legs > 100ms) vs transitioning
    contact_age = np.zeros((N, 4), dtype=int)
    for j in range(4):
        age = 0
        for i in range(N):
            if contacts[i, j] == 1:
                age += 1
                contact_age[i, j] = age
            else:
                age = 0

    min_age = np.zeros(N, dtype=int)
    for i in range(N):
        ages = [contact_age[i,j] for j in range(4) if contacts[i,j]==1]
        min_age[i] = min(ages) if ages else 0

    T_stable = 100  # ms threshold
    mask_stable = mask & (min_age >= T_stable)
    mask_trans  = mask & (min_age > 0) & (min_age < T_stable)

    print(f"\n{'='*65}")
    print(f"Contact phase breakdown (stable = all legs in contact > {T_stable}ms)")
    print("="*65)
    ns = mask_stable.sum()
    nt = mask_trans.sum()
    print(f"  Stable samples:  {ns}  ({ns/(ns+nt)*100:.1f}%)")
    print(f"  Transitioning:   {nt}  ({nt/(ns+nt)*100:.1f}%)")

    for label, m in [("Stable", mask_stable), ("Transitioning", mask_trans)]:
        if m.sum() == 0: continue
        e_gt2 = z_gt[m] - gt_vb[m]
        e_est2 = z_est[m] - gt_vb[m]
        e_esekf2 = est_vb[m] - gt_vb[m]

        print(f"\n  --- {label} (N={m.sum()}) ---")
        print(f"    z_leg(GT wy)  vx: RMSE={np.sqrt(np.mean(e_gt2[:,0]**2))*1000:.2f}  "
              f"bias={np.mean(e_gt2[:,0])*1000:+.2f} mm/s")
        print(f"    z_leg(EST wy) vx: RMSE={np.sqrt(np.mean(e_est2[:,0]**2))*1000:.2f}  "
              f"bias={np.mean(e_est2[:,0])*1000:+.2f} mm/s")
        print(f"    ESEKF fused   vx: RMSE={np.sqrt(np.mean(e_esekf2[:,0]**2))*1000:.2f}  "
              f"bias={np.mean(e_esekf2[:,0])*1000:+.2f} mm/s")

    print("\nDone.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
