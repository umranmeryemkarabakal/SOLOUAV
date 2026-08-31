#!/usr/bin/env python3
"""Adim 24: golge tilt modelinin GERCEK Gazebo servosuna sadakati — CEVRIMDISI dogrulama.

Kayitli u_cmd (= u_actual + du, log'dan) ile iki model ayni komut dizisiyle
surulur ve GERCEK eklem acisiyla (gz joint_state) karsilastirilir:

  A) MEVCUT  : 1. derece gecikme  ddelta = (cmd-d)/TILT_TAU, |ddelta| <= TILT_RATE_MAX
  B) SADIK   : gz JointPositionController + eklem dinamigi
               err = clamp(cmd - d, +-err_max)
               tau = clamp(p_gain*err, +-cmd_max)
               Coulomb surtunme (friction) + atalet J:  J*ddd = tau -+ friction

SDF'den (tiltrotor_indi/model.sdf):
  p_gain=100, i=d=0, cmd_max=2 Nm, err_max=0.2 rad
  motor_N iyy=0.0166704, rotor_N iyy=0.000166704 (cocuk link, birlikte doner)
  joint <friction>1.0</friction>, damping yok, limit [0, 1.57]
Sonuc: surtunmeyi yenmek icin |err| >= 1.0/100 = 0.01 rad = 0.57 deg -> OLU BANT.
"""
import sys
import numpy as np
from pyulog import ULog

ULG, GZ = sys.argv[1], sys.argv[2]
TILT_TAU, TILT_RATE_MAX = 0.15, 2.0
P_GAIN, CMD_MAX, ERR_MAX = 100.0, 2.0, 0.2
J = 0.0166704 + 0.000166704
FRIC = 1.0
TILT_MIN, TILT_MAX = 0.0, 1.57

u = ULog(ULG, ['tiltrotor_indi_status'])
st = {d.name: d for d in u.data_list}['tiltrotor_indi_status']
t = st.data['timestamp'] / 1e6
k = np.concatenate([[True], np.diff(t) > 1e-6]); t = t[k]
sh = np.column_stack([st.data[f'u_actual[{3+i}]'][k] for i in range(3)])
du = np.column_stack([st.data[f'du[{3+i}]'][k] for i in range(3)])
cmd = sh + du                      # modulun servoya gonderdigi mutlak tilt komutu

gz = np.loadtxt(GZ, delimiter=',')
gz = gz[np.argsort(gz[:, 0])]
gz = gz[np.concatenate([[True], np.diff(gz[:, 0]) > 1e-9])]
gz_t, real = gz[:, 0], gz[:, 1:4]

lo, hi = max(t[0], gz_t[0]) + 1.0, min(t[-1], gz_t[-1]) - 1.0
grid = np.arange(lo, hi, 0.004)
dt = 0.004
cmd_i = np.column_stack([np.interp(grid, t, cmd[:, i]) for i in range(3)])
real_i = np.column_stack([np.interp(grid, gz_t, real[:, i]) for i in range(3)])
print(f'pencere {lo:.1f}..{hi:.1f} s, {len(grid)} ornek @250Hz')
print(f'sadik model: max ivme = (cmd_max-friction)/J = '
      f'{(CMD_MAX-FRIC)/J:.1f} rad/s^2, olu bant = friction/p_gain = '
      f'{np.degrees(FRIC/P_GAIN):.3f} deg')


def model_first_order(c, d0):
    d = np.zeros(len(c)); d[0] = d0
    for n in range(1, len(c)):
        dd = (c[n-1] - d[n-1]) / TILT_TAU
        dd = np.clip(dd, -TILT_RATE_MAX, TILT_RATE_MAX)
        d[n] = np.clip(d[n-1] + dt*dd, TILT_MIN, TILT_MAX)
    return d


def model_faithful(c, d0):
    d = np.zeros(len(c)); d[0] = d0
    v = 0.0
    for n in range(1, len(c)):
        err = np.clip(c[n-1] - d[n-1], -ERR_MAX, ERR_MAX)
        tau = np.clip(P_GAIN * err, -CMD_MAX, CMD_MAX)
        if abs(v) < 1e-4:
            # stiction: surtunmeyi yenemiyorsa hic kimildama
            if abs(tau) <= FRIC:
                tau_net = 0.0
                v = 0.0
            else:
                tau_net = tau - np.sign(tau) * FRIC
        else:
            tau_net = tau - np.sign(v) * FRIC
        v += dt * tau_net / J
        dn = d[n-1] + dt * v
        if dn <= TILT_MIN:
            dn, v = TILT_MIN, max(v, 0.0)
        elif dn >= TILT_MAX:
            dn, v = TILT_MAX, min(v, 0.0)
        d[n] = dn
    return d


def model_deadband(c, d0):
    """MEVCUT 1. derece model + Coulomb OLU BANDI (friction/p_gain = 0.573 deg).
    Baskin hata kalici ofset oldugu icin en ucuz hedefli duzeltme bu."""
    db = FRIC / P_GAIN
    d = np.zeros(len(c)); d[0] = d0
    for n in range(1, len(c)):
        err = c[n-1] - d[n-1]
        if abs(err) <= db:
            d[n] = d[n-1]                      # surtunmeyi yenemiyor
            continue
        dd = np.clip(err / TILT_TAU, -TILT_RATE_MAX, TILT_RATE_MAX)
        d[n] = np.clip(d[n-1] + dt*dd, TILT_MIN, TILT_MAX)
    return d


print(f'\n{"":4s} {"model":>10s} {"ort sapma":>11s} {"RMS":>9s} {"p99":>9s} {"max":>8s}')
print('-' * 60)
for i in range(3):
    for nm, fn in (('MEVCUT', model_first_order), ('SADIK', model_faithful),
                   ('OLU BANT', model_deadband)):
        sim = fn(cmd_i[:, i], real_i[0, i])
        e = np.degrees(sim - real_i[:, i])
        print(f'  d{i} {nm:>10s} {e.mean():10.4f}d {np.sqrt((e**2).mean()):8.4f}d '
              f'{np.percentile(np.abs(e),99):8.4f}d {np.abs(e).max():7.3f}d')
    print()
