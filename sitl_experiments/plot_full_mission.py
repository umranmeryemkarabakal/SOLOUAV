#!/usr/bin/env python3
"""
Tam gorevin TEK SAYFALIK grafigi: kalkis -> ileri gecis -> sabit kanat ->
geri gecis -> hover -> inis.

NEDEN VAR (2026-08-30). Gorev alti fazdan olusuyor ve her fazin kendi arıza
imzasi var; metin ciktisi bunlari yan yana gostermiyor. Bu dosya butun fazlari
ayni zaman ekseninde, faz bantlariyla birlikte cizer -- boylece "salinim nerede
basladi", "hangi faz hangi fazi bozdu" gozle ayirt edilebilir.

KULLANIM
    python3 plot_full_mission.py                # en son ulog
    python3 plot_full_mission.py <ulog yolu>
"""
import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from pyulog import ULog

LOGDIR = os.path.expanduser('~/PX4-Autopilot/build/px4_sitl_default/rootfs/log')


def newest():
    best, bt = None, 0
    for r, _d, fs in os.walk(LOGDIR):
        for f in fs:
            if f.endswith('.ulg'):
                p = os.path.join(r, f)
                if os.path.getmtime(p) > bt:
                    best, bt = p, os.path.getmtime(p)
    return best


def topic(u, n):
    for d in u.data_list:
        if d.name == n:
            return d.data
    return None


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else newest()
    u = ULog(path)
    st = topic(u, 'tiltrotor_indi_status')
    lp = topic(u, 'vehicle_local_position')
    av = topic(u, 'vehicle_angular_velocity')
    at = topic(u, 'vehicle_attitude')
    ld = topic(u, 'vehicle_land_detected')

    ts = st['timestamp'] / 1e6
    t0 = ts[0]
    ts -= t0
    ip = lambda d, k: np.interp(ts + t0, d['timestamp'] / 1e6, d[k])

    # AGL: kalkis datumuna gore (ham -z DEGIL -- adim 116/117'nin kok nedeni)
    tl = lp['timestamp'] / 1e6
    alt = -lp['z']
    datum = float(alt[0])
    if ld is not None:
        air = np.where(~ld['landed'].astype(bool))[0]
        if len(air):
            pre = alt[tl < (ld['timestamp'] / 1e6)[air[0]]]
            if len(pre):
                datum = float(pre[-1])
    agl = -ip(lp, 'z') - datum

    T = [st[f'u_actual[{i}]'] + st[f'du[{i}]'] for i in range(3)]
    D = [np.degrees(st[f'u_actual[{i+3}]'] + st[f'du[{i+3}]']) for i in range(3)]
    q = [ip(at, f'q[{i}]') for i in range(4)]
    w, x, y, z = q
    roll = np.degrees(np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y)))
    pitch = np.degrees(np.arcsin(np.clip(2*(w*y - z*x), -1, 1)))
    p_r = ip(av, 'xyz[0]'); q_r = ip(av, 'xyz[1]'); r_r = ip(av, 'xyz[2]')
    vh = np.hypot(ip(lp, 'vx'), ip(lp, 'vy'))
    sat = np.stack([st[f'sat_flag[{i}]'] for i in range(3)], 1).any(1).astype(float)

    # --- faz bantlari ---
    ft, bt_, fw = st['ft_state'], st['bt_state'], st['fw_state']
    PH = [('hover',      (ft == 0) & (bt_ == 0) & (fw == 0), '#e8f0fe'),
          ('ileri gecis', (ft > 0) & (fw == 0),               '#fff3cd'),
          ('SABIT KANAT', fw > 0,                             '#d4edda'),
          ('geri gecis',  (bt_ > 0) & (fw == 0),              '#f8d7da')]

    fig, ax = plt.subplots(6, 1, figsize=(15, 16), sharex=True)
    fig.suptitle(f'TAM GOREV — {os.path.basename(path)}\n'
                 f'kalkis → ileri gecis → sabit kanat → geri gecis → hover → inis',
                 fontsize=13, fontweight='bold')

    def bands(a):
        for _, m, c in PH:
            if not m.any():
                continue
            d = np.diff(m.astype(int))
            starts = list(np.where(d == 1)[0] + 1)
            ends = list(np.where(d == -1)[0] + 1)
            if m[0]:
                starts = [0] + starts
            if m[-1]:
                ends = ends + [len(m) - 1]
            for s_, e_ in zip(starts, ends):
                a.axvspan(ts[s_], ts[e_], color=c, alpha=0.55, lw=0)

    # 1) irtifa
    a = ax[0]; bands(a)
    a.plot(ts, agl, 'b-', lw=1.4, label='AGL (kalkis datumuna gore)')
    a.axhline(0, color='k', lw=0.8, ls=':')
    a.axhline(2.0, color='gray', lw=0.8, ls='--', label='land_diff kapisi (2 m)')
    a.set_ylabel('irtifa [m]'); a.legend(loc='upper right', fontsize=8); a.grid(alpha=0.3)

    # 2) hiz
    a = ax[1]; bands(a)
    a.plot(ts, vh, 'g-', lw=1.3, label='yatay hiz')
    a.plot(ts, st['cruise_v_fwd'], 'c-', lw=1.0, alpha=0.8, label='govde ileri hizi')
    a.axhline(8.0, color='orange', ls='--', lw=0.8, label='FT_CRUISE_V')
    a.set_ylabel('hiz [m/s]'); a.legend(loc='upper right', fontsize=8); a.grid(alpha=0.3)

    # 3) rotor itkileri -- SABIT KANATTA SIFIRA INMELI
    a = ax[2]; bands(a)
    for i, (c, l) in enumerate([('r', 'T0 sag kanat'), ('b', 'T1 sol kanat'), ('m', 'T2 kuyruk')]):
        a.plot(ts, T[i], c, lw=1.1, label=l)
    a.axhline(45, color='k', ls='--', lw=0.8, alpha=0.6, label='ROTOR_TMAX')
    a.set_ylabel('itki [N]'); a.legend(loc='upper right', fontsize=8, ncol=2); a.grid(alpha=0.3)

    # 4) tilt acilari
    a = ax[3]; bands(a)
    for i, (c, l) in enumerate([('r', 'd0'), ('b', 'd1'), ('m', 'd2 kuyruk')]):
        a.plot(ts, D[i], c, lw=1.1, label=l)
    a.axhline(90, color='k', ls='--', lw=0.8, alpha=0.6)
    a.axhline(20, color='m', ls=':', lw=1.0, alpha=0.8, label='TAIL_TILT_MAX (20 deg)')
    a.set_ylabel('tilt [deg]'); a.legend(loc='upper right', fontsize=8, ncol=2); a.grid(alpha=0.3)

    # 5) tutum -- SALINIM BURADA GORULUR
    a = ax[4]; bands(a)
    a.plot(ts, roll, 'r-', lw=1.0, label='roll')
    a.plot(ts, pitch, 'b-', lw=1.0, label='pitch')
    a.set_ylabel('tutum [deg]'); a.legend(loc='upper right', fontsize=8); a.grid(alpha=0.3)

    # 6) acisal hizlar + doyum
    a = ax[5]; bands(a)
    a.plot(ts, p_r, 'r-', lw=0.8, alpha=0.8, label='p')
    a.plot(ts, q_r, 'b-', lw=0.8, alpha=0.8, label='q')
    a.plot(ts, r_r, 'g-', lw=0.9, label='r (yaw)')
    a2 = a.twinx()
    a2.fill_between(ts, 0, sat, color='k', alpha=0.25, step='mid')
    a2.set_ylabel('doyum', fontsize=8); a2.set_ylim(0, 1)
    a.set_ylabel('acisal hiz [rad/s]'); a.set_xlabel('zaman [s]')
    a.legend(loc='upper right', fontsize=8, ncol=3); a.grid(alpha=0.3)

    fig.legend(handles=[Patch(facecolor=c, label=n) for n, _, c in PH],
               loc='lower center', ncol=4, fontsize=9, frameon=False)
    fig.tight_layout(rect=[0, 0.025, 1, 0.975])
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'tam_gorev.png')
    fig.savefig(out, dpi=110)
    print(f'grafik: {out}')

    # --- sayisal ozet ---
    print(f'\n=== FAZ OZETI ===  (datum {datum:+.2f} m)')
    for n, m, _ in PH:
        if m.sum() < 10:
            continue
        print(f'  {n:<13} {ts[m][0]:6.1f}-{ts[m][-1]:6.1f} s  '
              f'T=[{T[0][m].mean():5.2f} {T[1][m].mean():5.2f} {T[2][m].mean():5.2f}] N  '
              f'tilt={D[0][m].mean():5.1f} deg  |roll|max={np.abs(roll[m]).max():5.1f}')
    land = agl < 3.0
    late = ts > ts[-1] - 60
    ml = land & late
    if ml.sum() > 10:
        print(f'\n=== INIS (son 60 s, AGL<3 m) ===')
        print(f'  en dusuk AGL      : {agl[ml].min():+.2f} m')
        print(f'  max |T0-T1|       : {np.abs(T[0]-T[1])[ml].max():.2f} N')
        print(f'  yaw donusu        : {np.degrees(np.trapezoid(r_r[ml], ts[ml])):+.1f} deg')
        print(f'  temas darbesi max |p_dot| : '
              f'{np.abs(np.gradient(p_r[ml], ts[ml])).max():.1f} rad/s^2')
    print(f'  BIG_M ornek       : {int(sat.sum())}')


if __name__ == '__main__':
    main()
