#!/usr/bin/env python3
"""
Adim 93: gercek SITL telemetrisinden (aktuator durumu) beklenen (model) momenti
hesaplayip, gercekte olculen acisal ivmeyle karsilastirir -- pitch ekseninde.
Ayni fizik: effectiveness_matrix.m / sf_wls_alloc.m'in nu0 hesabi
(tau_i = cross(r_i, f_i) + m_i, f_i = T_i*dir_i, m_i = km_i*T_i*dir_i).
"""
import numpy as np
from pyulog import ULog
import sys

LOG = sys.argv[1] if len(sys.argv) > 1 else '/home/omer/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-08-25/05_31_34.ulg'

# --- sabitler (tiltrotor_params.m ile birebir) ---
rpos = np.array([[0.27, 0.27, -0.55],
                  [0.35, -0.35, 0.00],
                  [-0.11, -0.11, -0.07]])  # columns = rotors 0,1,2
km = np.array([-0.06, 0.06, -0.06])
I = np.diag([0.2, 0.25, 0.25])
Iinv = np.linalg.inv(I)

ROTOR_KF = 2.0e-5
ROTOR_WMAX = 1500.0
ROTOR_WMIN = 10.0
ROTOR_TMAX = ROTOR_KF * ROTOR_WMAX**2
TILT_MIN = 0.0
TILT_MAX = np.pi/2

u = ULog(LOG)
def ds(name):
    for d in u.data_list:
        if d.name == name:
            return d

mot = ds('actuator_motors')
serv = ds('actuator_servos')
avg = ds('vehicle_angular_velocity_groundtruth')

tm = mot.data['timestamp']/1e6
ts = serv.data['timestamp']/1e6
tw = avg.data['timestamp']/1e6

# resample everything onto avg's (angular velocity) time grid
w0 = avg.data['xyz[0]']; w1 = avg.data['xyz[1]']; w2 = avg.data['xyz[2]']

def resample(t_src, y_src, t_dst):
    return np.interp(t_dst, t_src, y_src)

m0 = resample(tm, mot.data['control[0]'], tw)
m1 = resample(tm, mot.data['control[1]'], tw)
m2 = resample(tm, mot.data['control[2]'], tw)
d0 = resample(ts, serv.data['control[5]'], tw)
d1 = resample(ts, serv.data['control[6]'], tw)
d2 = resample(ts, serv.data['control[7]'], tw)

# normalized motor -> thrust (T = KF*w^2, w = WMIN + m*(WMAX-WMIN))
def motor_to_T(m):
    w = ROTOR_WMIN + np.clip(m,0,1)*(ROTOR_WMAX-ROTOR_WMIN)
    return ROTOR_KF*w**2

T0 = motor_to_T(m0); T1 = motor_to_T(m1); T2 = motor_to_T(m2)
# normalized tilt -> delta (delta = TILT_MIN + (norm+1)/2*(TILT_MAX-TILT_MIN))
def servo_to_delta(d):
    return TILT_MIN + (np.clip(d,-1,1)+1)/2.0*(TILT_MAX-TILT_MIN)

delta0 = servo_to_delta(d0); delta1 = servo_to_delta(d1); delta2 = servo_to_delta(d2)

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

# predicted angular acceleration: Iinv*(tau - omega x I*omega)
omega = np.vstack([w0,w1,w2]).T
Iw = (I @ omega.T).T
gyro = np.cross(omega, Iw)
omega_dot_pred = (Iinv @ (tau_pred - gyro).T).T

# measured angular acceleration: finite difference of omega (groundtruth),
# LIGHTLY SMOOTHED first (raw sample-to-sample gradient is dominated by
# differentiation noise/jitter at this log rate -- washes out any real
# correlation with the ~2-2.5s oscillation we're trying to see).
def smooth(y, win=9):
    kernel = np.ones(win)/win
    return np.convolve(y, kernel, mode='same')

omega_s = np.column_stack([smooth(omega[:,i]) for i in range(3)])
dt = np.gradient(tw)
omega_dot_meas = np.gradient(omega_s, axis=0) / dt[:,None]
omega_dot_meas_raw = np.gradient(omega, axis=0) / dt[:,None]

# focus window: t = 20..60s (oscillation period per Adim 92)
mask = (tw >= 20) & (tw <= 60)
print("PITCH (axis 1) -- 20-60s penceresi:")
print("t(s)   tau_pred_pitch(Nm)  qdot_pred(rad/s^2)  qdot_meas(rad/s^2)   ratio(meas/pred)")
idxs = np.where(mask)[0]
step = max(1, len(idxs)//30)
for i in idxs[::step]:
    r = omega_dot_meas[i,1]/omega_dot_pred[i,1] if abs(omega_dot_pred[i,1])>1e-6 else np.nan
    print(f"{tw[i]:6.2f}  {tau_pred[i,1]:8.3f}          {omega_dot_pred[i,1]:8.3f}          {omega_dot_meas[i,1]:8.3f}          {r:6.2f}")

pred_rms = np.sqrt(np.mean(omega_dot_pred[mask,1]**2))
meas_rms = np.sqrt(np.mean(omega_dot_meas[mask,1]**2))
corr = np.corrcoef(omega_dot_pred[mask,1], omega_dot_meas[mask,1])[0,1]
corr_raw = np.corrcoef(omega_dot_pred[mask,1], omega_dot_meas_raw[mask,1])[0,1]
print(f"\nRMS qdot_pred={pred_rms:.3f}  RMS qdot_meas(smoothed)={meas_rms:.3f}  corr(pred,meas_smoothed)={corr:.3f}  corr(pred,meas_raw)={corr_raw:.3f}")
print(f"median dt (angular velocity log)={np.median(dt)*1000:.2f} ms, n_samples in window={mask.sum()}")

# also try a range of small time-SHIFTS between pred and meas, in case there's
# a fixed lag (e.g. logger buffering / topic latency) rather than pure noise --
# a real lag would show up as a correlation PEAK at a nonzero shift.
print('\nshift(samples)  shift(ms)   corr(pred, meas_smoothed shifted)')
best = (-2,-1)
for shift in range(-15, 16):
    if shift < 0:
        a = omega_dot_pred[mask,1][-shift:]
        b = omega_dot_meas[mask,1][:len(a)]
    elif shift > 0:
        b = omega_dot_meas[mask,1][shift:]
        a = omega_dot_pred[mask,1][:len(b)]
    else:
        a = omega_dot_pred[mask,1]; b = omega_dot_meas[mask,1]
    if len(a) > 10:
        c = np.corrcoef(a,b)[0,1]
        if abs(c) > abs(best[1]) if not np.isnan(c) else False:
            best = (shift, c)
        if shift % 3 == 0:
            print(f"{shift:8d}      {shift*np.median(dt)*1000:8.1f}    {c:.3f}")
print(f"\nbest |corr| at shift={best[0]} samples ({best[0]*np.median(dt)*1000:.1f} ms): {best[1]:.3f}")
