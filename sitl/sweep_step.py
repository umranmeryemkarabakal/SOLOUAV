#!/usr/bin/env python3
"""Adim 23b: TILT_SLEW_BOX_RATE taramasi — her degerde AYNI uyarim (+30 deg yaw adimi).
Faz sinirlari yaw_sp degisimlerinden, aktif kutu degeri |du(3)| p99.5 / TS_BOX'tan
BAGIMSIZ olarak geri okunur.
MATLAB referansi (efektif kutu 3.0 rad/s): asim %24.1, ts(+-2 deg) 3.69 s.
"""
import sys
import numpy as np
from pyulog import ULog

ULG = sys.argv[1]
TS_BOX = 1.0 / 250.0

u = ULog(ULG, ['tiltrotor_indi_status', 'tiltrotor_indi_setpoint', 'vehicle_angular_velocity',
               'vehicle_attitude', 'vehicle_local_position'])
D = {d.name: d for d in u.data_list}
st = D['tiltrotor_indi_status']; t = st.data['timestamp'] / 1e6
k = np.concatenate([[True], np.diff(t) > 1e-6]); t = t[k]
du3 = np.abs(st.data['du[3]'][k])
av = D['vehicle_angular_velocity']; av_t = av.data['timestamp'] / 1e6; r = av.data['xyz[2]']
at = D['vehicle_attitude']; at_t = at.data['timestamp'] / 1e6
q = np.column_stack([at.data[f'q[{i}]'] for i in range(4)])
yaw = np.degrees(np.arctan2(2*(q[:,0]*q[:,3]+q[:,1]*q[:,2]), 1-2*(q[:,2]**2+q[:,3]**2)))
lp = D['vehicle_local_position']; lp_t = lp.data['timestamp'] / 1e6
vh = np.hypot(lp.data['vx'], lp.data['vy'])
sp = D['tiltrotor_indi_setpoint']; sp_t = sp.data['timestamp'] / 1e6
sy = np.degrees(sp.data['yaw_sp'])

ups = [sp_t[i+1] for i in range(len(sy)-1) if sy[i+1] - sy[i] > 20]
print(f'+30 deg adimlari: {[f"{x:.1f}" for x in ups]}')

print(f'\n{"kutu(geri-okunan)":>18s} {"vh":>6s} {"tepe":>7s} {"asim%":>7s} {"ts+-2deg":>10s} '
      f'{"son deger":>10s} {"max|r|":>8s} {"son5s rRMS":>11s}')
print('-' * 92)
for t_up in ups:
    a, b = t_up, t_up + 18
    ms = (t >= a) & (t < b); mv = (av_t >= a) & (av_t < b)
    ma = (at_t >= a) & (at_t < b); ml = (lp_t >= a) & (lp_t < b)
    if ms.sum() < 50:
        continue
    rate = np.percentile(du3[ms], 99.5) / TS_BOX
    tt, yy = at_t[ma], yaw[ma]
    y0 = np.interp(t_up, at_t, yaw)
    tgt = y0 + 30.0
    os = max(0.0, yy.max() - tgt)
    err = np.abs(yy - tgt)
    out = np.where(err > 2.0)[0]
    ts = np.nan if (len(out) and out[-1] >= len(tt) - 1) else (tt[out[-1]+1] - t_up if len(out) else 0.0)
    tail = (av_t >= b - 5) & (av_t < b)
    print(f'{rate:18.3f} {vh[ml].mean():6.2f} {yy.max():6.1f}d {100*os/30:6.1f} '
          + (f'{ts:9.2f}s' if not np.isnan(ts) else '  OTURMADI')
          + f' {yy[-1]:9.1f}d {np.abs(r[mv]).max():8.4f} {np.sqrt((r[tail]**2).mean()):11.4f}')

print('\nMATLAB referansi (efektif kutu 3.0 rad/s, 400 Hz): asim %24.1, ts 3.69 s')
print('tgt = adim anindaki yaw + 30 (arac surekli suruklendigi icin mutlak degil bagil)')
