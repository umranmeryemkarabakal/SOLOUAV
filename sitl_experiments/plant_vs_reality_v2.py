#!/usr/bin/env python3
"""
Adim 93b: Adim 93'un tekrari, ama PX4'un KOMUTLADIGI aktuator durumu yerine
Gazebo'nun KENDI joint_state'inden (gz-sim-joint-state-publisher-system,
GERCEK eklem pozisyonu/hizi) T/delta turetilerek.
"""
import numpy as np
import pickle
from pyulog import ULog
import sys

ULOG = sys.argv[1] if len(sys.argv) > 1 else '/home/omer/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-08-25/06_45_09.ulg'
PKL = sys.argv[2] if len(sys.argv) > 2 else '/tmp/claude-1000/-home-omer/2ebc1a5b-f0c9-4937-8214-37f5d48fdea3/scratchpad/joint_state_parsed.pkl'

with open(PKL, 'rb') as f:
    js = pickle.load(f)

rpos = np.array([[0.27, 0.27, -0.55],
                  [0.35, -0.35, 0.00],
                  [-0.11, -0.11, -0.07]])
km = np.array([-0.06, 0.06, -0.06])
I = np.diag([0.2, 0.25, 0.25])
Iinv = np.linalg.inv(I)
ROTOR_KF = 2.0e-5
SLOWDOWN = 20.0

u = ULog(ULOG)
def ds(name):
    for d in u.data_list:
        if d.name == name:
            return d
avg = ds('vehicle_angular_velocity_groundtruth')
tw_full = avg.data['timestamp']/1e6
w0f = avg.data['xyz[0]']; w1f = avg.data['xyz[1]']; w2f = avg.data['xyz[2]']

# gz joint_state time base: header.stamp is SIM time (lockstep), should be
# directly comparable to ulog timestamps (also sim time under lockstep).
# Find overlap.
tjs = js['t']
t0 = max(tjs[0], tw_full[0])
t1 = min(tjs[-1], tw_full[-1])
print(f"ulog t range {tw_full[0]:.2f}-{tw_full[-1]:.2f}, joint_state t range {tjs[0]:.2f}-{tjs[-1]:.2f}, overlap {t0:.2f}-{t1:.2f}")

mask_w = (tw_full >= t0) & (tw_full <= t1)
tw = tw_full[mask_w]
w0 = w0f[mask_w]; w1 = w1f[mask_w]; w2 = w2f[mask_w]

def resample(t_src, y_src, t_dst):
    return np.interp(t_dst, t_src, y_src)

delta0 = resample(tjs, js['motor_0_joint_pos'], tw)
delta1 = resample(tjs, js['motor_1_joint_pos'], tw)
delta2 = resample(tjs, js['motor_2_joint_pos'], tw)
w0j = resample(tjs, js['rotor_0_joint_vel'], tw) * SLOWDOWN
w1j = resample(tjs, js['rotor_1_joint_vel'], tw) * SLOWDOWN
w2j = resample(tjs, js['rotor_2_joint_vel'], tw) * SLOWDOWN

T0 = ROTOR_KF * w0j**2
T1 = ROTOR_KF * w1j**2
T2 = ROTOR_KF * w2j**2

N = len(tw)
tau_pred = np.zeros((N,3))
for k in range(N):
    T = [T0[k], T1[k], T2[k]]
    de = [delta0[k], delta1[k], delta2[k]]
    tau = np.zeros(3)
    for i in range(3):
        s, c = np.sin(de[i]), np.cos(de[i])
        dir_i = np.array([s, 0, -c])
        f_i = T[i]*dir_i
        m_i = km[i]*T[i]*dir_i
        r_i = rpos[:,i]
        tau += np.cross(r_i, f_i) + m_i
    tau_pred[k] = tau

omega = np.vstack([w0,w1,w2]).T
Iw = (I @ omega.T).T
gyro = np.cross(omega, Iw)
omega_dot_pred = (Iinv @ (tau_pred - gyro).T).T

def smooth(y, win=9):
    kernel = np.ones(win)/win
    return np.convolve(y, kernel, mode='same')

omega_s = np.column_stack([smooth(omega[:,i]) for i in range(3)])
dt = np.gradient(tw)
omega_dot_meas = np.gradient(omega_s, axis=0) / dt[:,None]

# also smooth the predicted signal the same way for a fair spectral comparison
tau_pred_s = np.column_stack([smooth(tau_pred[:,i]) for i in range(3)])
omega_dot_pred_s = (Iinv @ (tau_pred_s - gyro).T).T

mask = (tw >= 30) & (tw <= 90)
print("\nPITCH (axis 1) -- gercek eklem durumundan, 30-90s penceresi:")
print("t(s)   T0      T1      T2    delta0  delta1  delta2   tau_pred_pitch  qdot_pred  qdot_meas")
idxs = np.where(mask)[0]
step = max(1, len(idxs)//25)
for i in idxs[::step]:
    print(f"{tw[i]:6.1f} {T0[i]:6.2f}  {T1[i]:6.2f}  {T2[i]:6.2f}  {np.degrees(delta0[i]):5.1f}  {np.degrees(delta1[i]):5.1f}  {np.degrees(delta2[i]):5.1f}   {tau_pred[i,1]:8.3f}    {omega_dot_pred[i,1]:8.3f}   {omega_dot_meas[i,1]:8.3f}")

pred_rms = np.sqrt(np.mean(omega_dot_pred[mask,1]**2))
pred_s_rms = np.sqrt(np.mean(omega_dot_pred_s[mask,1]**2))
meas_rms = np.sqrt(np.mean(omega_dot_meas[mask,1]**2))
corr = np.corrcoef(omega_dot_pred[mask,1], omega_dot_meas[mask,1])[0,1]
corr_s = np.corrcoef(omega_dot_pred_s[mask,1], omega_dot_meas[mask,1])[0,1]
print(f"\nRMS qdot_pred(raw)={pred_rms:.3f}  RMS qdot_pred(smoothed)={pred_s_rms:.3f}  RMS qdot_meas(smoothed)={meas_rms:.3f}")
print(f"corr(pred_raw, meas)={corr:.3f}   corr(pred_smoothed, meas)={corr_s:.3f}")

# thrust sanity check
print(f"\nT0/T1/T2 mean over window: {T0[mask].mean():.2f} {T1[mask].mean():.2f} {T2[mask].mean():.2f} N (hover weight ~= {5.0*9.81:.1f} N)")
