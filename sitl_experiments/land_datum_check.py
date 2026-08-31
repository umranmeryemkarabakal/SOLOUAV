#!/usr/bin/env python3
"""
Adim 116: iniste kullanilan "irtifa" sinyalinin DATUM KAYMASINI olcer.

NEDEN: Adim 112'nin kanat-itki-farki siniri `(-lpos.z) < LAND_DIFF_ALT` ile
kapilanir. `-lpos.z` YERDEN yukseklik DEGIL, EKF yerel orijinden yukseklitir;
orijin kalkista bir kez kurulur ve o anin GPS/baro hatasini kalici bir OFSET
olarak tasir. Adim 113 "arac 2,36 m'de havada kilitlendi, temas YOK" derken bu
ofseti olcmemisti.

YONTEM: datum, aracin kalkistan HEMEN ONCE (land detector `landed` 1 -> 0
gecisinin son ornegi) yerde otururken okudugu `-z`'dir -- ucus yazilimindaki
captureGroundDatum() ile AYNI an. Gercek AGL = rapor edilen irtifa - datum.

KAYMA sutunu (Adim 117): ayni olcumun ucus SONUNDAKI hali. Kalkis datumu
init hatasini siler ama ucus boyunca biriken EKF z kaymasini SILMEZ;
5 kosumda olculen kayma +0.12 .. +1.15 m. Yani duzeltilmis AGL de tam degil,
yalnizca cok daha iyi.

KULLANIM:
    python3 land_datum_check.py                 # butun kosumlarin ofset tablosu
    python3 land_datum_check.py 11_26_27        # tek kosumun temas-ani izi
"""
import os
import sys
import glob

import numpy as np
from pyulog import ULog

LOGDIR = os.path.expanduser(
    '~/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-08-29/')

# Temas darbesi esigi: |p|'nin sicramasi (rad/s). Olculen darbeler 0.21-0.51.
IMPULSE_P = 0.15
# Ucus sonu oturma penceresi (s): motorlar kesildikten sonraki dinlenme.
SETTLE_S = 5.0


def _topic(ulog, name):
    for d in ulog.data_list:
        if d.name == name:
            return d.data
    return None


def load(path):
    u = ULog(path)
    lp = _topic(u, 'vehicle_local_position')
    st = _topic(u, 'tiltrotor_indi_status')
    av = _topic(u, 'vehicle_angular_velocity')
    at = _topic(u, 'vehicle_attitude')
    ld = _topic(u, 'vehicle_land_detected')
    if lp is None or st is None:
        return None

    ts = st['timestamp'] / 1e6
    ip = lambda d, k: np.interp(ts, d['timestamp'] / 1e6, d[k])

    tl = lp['timestamp'] / 1e6
    alt_raw = -lp['z']
    # Ucus yazilimiyla ayni an: kalkistan hemen onceki son yerde-oturma ornegi.
    datum = float(alt_raw[0])
    drift = float('nan')

    if ld is not None:
        t_ld = ld['timestamp'] / 1e6
        airborne = np.where(~ld['landed'].astype(bool))[0]

        if len(airborne):
            pre = alt_raw[tl < t_ld[airborne[0]]]

            if len(pre):
                datum = float(pre[-1])

        settled = alt_raw[tl > tl[-1] - SETTLE_S]

        if len(settled):
            drift = float(np.mean(settled)) - datum

    R = {'t': ts, 'datum': datum, 'drift': drift,
         'alt': ip(lp, 'z') * -1.0, 'vz': ip(lp, 'vz'),
         'x': ip(lp, 'x'), 'y': ip(lp, 'y'),
         'p': ip(av, 'xyz[0]'), 'q': ip(av, 'xyz[1]'), 'r': ip(av, 'xyz[2]'),
         'fz_sp': st['fz_sp']}
    R['agl'] = R['alt'] - datum
    for i in range(3):
        R[f'T{i}'] = st[f'u_actual[{i}]'] + st[f'du[{i}]']
    R['diff'] = np.abs(R['T0'] - R['T1'])
    q = [ip(at, f'q[{i}]') for i in range(4)]
    w, x, y, z = q
    R['roll'] = np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    R['pitch'] = np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)))
    return R


def contact_index(R):
    """Temas darbesi: alcalmanin sonunda |p|'nin ilk sicramasi."""
    late = R['t'] > R['t'][0] + 60.0
    low = R['agl'] < 3.0
    m = late & low & (np.abs(R['p']) > IMPULSE_P)
    idx = np.where(m)[0]
    return int(idx[0]) if len(idx) else None


def table():
    print(f"{'log':>9} {'datum':>7} {'kayma':>7} {'min rapor':>10} {'min AGL':>8} "
          f"{'temas?':>7} {'temas rapor':>12} {'max|T0-T1|':>11} {'yaw60':>9}")
    for f in sorted(glob.glob(os.path.join(LOGDIR, '*.ulg'))):
        R = load(f)
        if R is None or R['t'][-1] - R['t'][0] < 120:
            continue
        m = R['t'] > R['t'][-1] - 90
        i = contact_index(R)
        yaw = np.degrees(np.trapezoid(R['r'][R['t'] > R['t'][-1] - 60],
                                      R['t'][R['t'] > R['t'][-1] - 60]))
        tag = os.path.basename(f)[:-4]
        c_rep = f"{R['alt'][i]:12.2f}" if i is not None else f"{'-':>12}"
        print(f"{tag:>9} {R['datum']:+7.2f} {R['drift']:+7.2f} {R['alt'][m].min():10.2f} "
              f"{R['agl'][m].min():8.2f} {'EVET' if i is not None else 'yok':>7} "
              f"{c_rep} {R['diff'][m].max():11.2f} {yaw:9.1f}")


def trace(tag):
    R = load(os.path.join(LOGDIR, tag + '.ulg'))
    i = contact_index(R)
    if i is None:
        print(f"{tag}: temas darbesi bulunamadi")
        return
    print(f"=== {tag}  datum ofseti = {R['datum']:+.2f} m  "
          f"temas t={R['t'][i]:.2f}s  rapor={R['alt'][i]:.2f} m  "
          f"GERCEK={R['agl'][i]:.2f} m ===")
    cols = ['t', 'alt', 'agl', 'vz', 'roll', 'pitch', 'p', 'q', 'r',
            'T0', 'T1', 'diff']
    print(''.join(f"{c:>9}" for c in cols))
    for k in range(max(0, i - 40), min(len(R['t']), i + 400), 10):
        print(''.join(f"{R[c][k]:9.3f}" for c in cols))


if __name__ == '__main__':
    if len(sys.argv) > 1:
        trace(sys.argv[1])
    else:
        table()
