#!/usr/bin/env python3
"""BRAKE'te geri kacis ve 'attitude LOST' teshisi -- Adim 38 yan bulgulari.

Iki soruyu OLCUMLE ayirir, tahminle degil:

(A) BRAKE -> HANDOFF gecisi neden olmadi?
    Gecis kosulu `v_h < BT_HANDOFF_V` ve v_h bir BUYUKLUK (hypot(vx,vy)).
    Arac ileri hizini sifirdan gecirip GERI hizlanirsa buyukluk yeniden
    buyur; ayrica yanal bir bilesen varsa ileri bilesen sifirdan gecerken
    bile buyukluk esigin altina hic inmeyebilir. Bu betik BRAKE penceresinde
    v_h'nin minimumunu, o andaki GOVDE ileri hizini (u) ve yanal hizi (v)
    ayri ayri basar -- yani "esik yanlis sinyale mi bakiyor" sorusunu
    dogrudan cevaplar.

(B) 'attitude LOST (tilt=0)' neden tetiklendi?
    `tilt_aligned` iki sarta bagli: estimator_status_flags 3 s'den taze
    olmali VE cs_tilt_align set olmali. EKF2 bu topic'i ~1 Hz yayinliyor,
    yani ikisi de mumkun: (i) topic bayatladi, (ii) EKF gercekten hizalamayi
    dusurdu. Betik topic'in yayin ARALIKLARINI ve cs_tilt_align'in
    degisimlerini basar; hangisi oldugu boylece tahmin edilmez, gorulur.

Kullanim:
    python3 diag_brake_reversal.py <ulog>
"""

from __future__ import annotations

import math
import sys

import numpy as np
from pyulog import ULog

BT_HANDOFF_V = 3.0


def diag(path: str) -> None:
    ulog = ULog(path, ["tiltrotor_indi_status", "vehicle_local_position",
                       "vehicle_attitude", "estimator_status_flags"])
    d = {m.name: m.data for m in ulog.data_list}

    # --- (A) BRAKE penceresinde isaretli hiz ---
    st = d["tiltrotor_indi_status"]
    t = np.asarray(st["timestamp"], float) / 1e6
    keep = np.concatenate(([True], np.diff(t) > 1e-6))
    t = t[keep]
    bt = np.asarray(st["bt_state"], float)[keep]

    lp = d["vehicle_local_position"]
    tl = np.asarray(lp["timestamp"], float) / 1e6
    vx = np.asarray(lp["vx"], float)
    vy = np.asarray(lp["vy"], float)

    att = d["vehicle_attitude"]
    ta = np.asarray(att["timestamp"], float) / 1e6
    q = np.column_stack([np.asarray(att["q[%d]" % i], float) for i in range(4)])
    yaw = np.arctan2(2.0 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]),
                     1.0 - 2.0 * (q[:, 2] ** 2 + q[:, 3] ** 2))

    print(f"=== {path.split('/')[-1]} ===")
    edges = np.nonzero(np.diff(bt) != 0)[0] + 1
    bounds = np.concatenate(([0], edges, [len(bt) - 1]))
    print("\n(A) BRAKE evreleri -- esigin baktigi BUYUKLUK vs govde ILERI hizi")
    for a, b in zip(bounds[:-1], bounds[1:]):
        if int(bt[a]) != 2 or b - a < 5:
            continue
        m = (tl >= t[a]) & (tl <= t[b])
        if not m.any():
            continue
        vh = np.hypot(vx[m], vy[m])
        i = int(np.argmin(vh))
        tm = tl[m][i]
        psi = float(np.interp(tm, ta, np.unwrap(yaw)))
        u = vx[m][i] * math.cos(psi) + vy[m][i] * math.sin(psi)
        v = -vx[m][i] * math.sin(psi) + vy[m][i] * math.cos(psi)
        print(f"  BRAKE {t[a]:6.1f}-{t[b]:6.1f} s: min|v_h| = {vh[i]:5.2f} m/s "
              f"(esik {BT_HANDOFF_V}) -> govde u = {u:+6.2f}, yanal v = {v:+6.2f} m/s")
        print(f"     v_h son = {vh[-1]:5.2f} m/s "
              f"({'GERI KACIS' if vh[-1] > vh[i] + 1.0 else 'kacis yok'})")

    # --- (B) estimator_status_flags tazeligi / cs_tilt_align ---
    print("\n(B) estimator_status_flags -- 'tilt=0' iki sebepten hangisi")
    esf = d.get("estimator_status_flags")
    if esf is None:
        print("  topic loglanmamis")
        return
    te = np.asarray(esf["timestamp"], float) / 1e6
    align = np.asarray(esf["cs_tilt_align"], float)
    gaps = np.diff(te)
    print(f"  yayin araligi: ortalama {gaps.mean():.2f} s, max {gaps.max():.2f} s "
          f"(>3 s olan: {int((gaps > 3.0).sum())})")
    print(f"  cs_tilt_align: her zaman {'1' if align.min() > 0.5 else 'DEGIL'}"
          f"  (min {align.min():.0f}, 0 olan ornek {int((align < 0.5).sum())})")
    if (gaps > 3.0).any():
        i = int(np.argmax(gaps))
        print(f"  -> EN UZUN BOSLUK t = {te[i]:.1f} .. {te[i+1]:.1f} s ({gaps[i]:.2f} s): "
              f"bu sure boyunca tilt_aligned FALSE olur, EKF saglikli olsa bile")


if __name__ == "__main__":
    for p in sys.argv[1:]:
        diag(p)
