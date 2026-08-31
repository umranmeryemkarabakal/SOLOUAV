#!/usr/bin/env python3
"""Ucus icinde CIKIS KESILMESI ve duruş boslugu taramasi (herhangi bir ulog).

Iki soruyu olcer, ikisi de kontrol listesinde ACIK madde:

1. **Havadayken motorlara NaN gitti mi?** Adim 32'de bulunan imzasiz-tasma
   hatasi ucus basina 7-13 kez tum motorlara NaN yaziyordu ve GORUNMUYORDU,
   cunku hicbir sey loglamiyordu. Duzeltildi, ama adim 35 duruş kaybini SERT
   ON KOSUL yapinca bu yol yeniden bir failsafe davranisi hâline geldi: >50 ms
   duruş boslugu artik cikisi kesiyor. Yani "kac kez kesildi" surekli
   izlenmesi gereken bir regresyon olcusu oldu.

2. **`vehicle_attitude` bosluklari ne kadar?** `FS_ATT_TIMEOUT_US = 50 ms`.
   Kontrol listesi bu boslugun gercek EKF'te ne siklikta/uzunlukta oldugunun
   BILINMEDIGINI yaziyor. SITL en azindan bir alt sinir verir; donanimda
   yerde/tetherli tekrar olculmeli.

Not: NaN ornekleri ayrica `tiltrotor_indi_status`'te "tum u_actual = 0" olarak
gorunur (kesme yollari status'u sifir-doldurulmus yayinliyor). Bu betik ikisini
capraz kontrol eder; ayni zaman damgalarinda cikmalari beklenir.

Kullanim:
    python3 check_output_cuts.py [ulog ...]      # yoksa en yeni ulog
"""

from __future__ import annotations

import os
import sys

import numpy as np
from pyulog import ULog

ULOG_DIR = "/home/umran/PX4-Autopilot/build/px4_sitl_default/rootfs/log"
FS_ATT_TIMEOUT_S = 0.050


def newest_ulog() -> str:
    best, best_m = "", 0.0
    for root, _d, files in os.walk(ULOG_DIR):
        for f in files:
            if f.endswith(".ulg"):
                p = os.path.join(root, f)
                if os.path.getmtime(p) > best_m:
                    best, best_m = p, os.path.getmtime(p)
    return best


def check(path: str) -> bool:
    u = ULog(path, ["actuator_motors", "actuator_armed", "vehicle_attitude",
                    "tiltrotor_indi_status"])
    d = {x.name: x.data for x in u.data_list}

    for need in ("actuator_motors", "actuator_armed"):
        if need not in d:
            print(f"  ATLANDI: {need} ulog'da yok")
            return True

    ar = d["actuator_armed"]
    t_ar = np.asarray(ar["timestamp"], float) / 1e6
    armed = np.asarray(ar["armed"], float) > 0.5
    if not armed.any():
        print("  ATLANDI: bu ucusta hic arm yok")
        return True
    idx = np.nonzero(armed)[0]
    t_arm, t_dis = t_ar[idx[0]], t_ar[idx[-1]]

    am = d["actuator_motors"]
    t_m = np.asarray(am["timestamp"], float) / 1e6
    ctl = np.column_stack([np.asarray(am["control[%d]" % i], float) for i in range(3)])

    # ARM SINIRI AYRI SAYILIR. `actuator_armed` ile modulun gordugu
    # `vehicle_control_mode.flag_armed` ayni tick'te degismiyor, yani arm anininda
    # bir tick'lik NaN normaldir (modul hâlâ disarmed yolunda). Bunu "havada
    # kesinti" saymak olcutu anlamsizlastirir; ayri raporlanir.
    ARM_EDGE_S = 0.2
    w_air = (t_m >= t_arm + ARM_EDGE_S) & (t_m <= t_dis)
    w_edge = (t_m >= t_arm) & (t_m < t_arm + ARM_EDGE_S)
    nan = np.isnan(ctl[w_air]).any(axis=1)
    n_nan = int(nan.sum())
    n_edge = int(np.isnan(ctl[w_edge]).any(axis=1).sum())

    print(f"  arm penceresi   : {t_arm:.1f} - {t_dis:.1f} s  ({w_air.sum()} motor ornegi, "
          f"ilk {ARM_EDGE_S:.1f} s haric)")
    print(f"  havada NaN cikis: {n_nan} ornek (%{100 * nan.mean() if w_air.sum() else 0:.4f})"
          + (f"  t = {np.round(t_m[w_air][nan][:8], 2)}" if n_nan else "   <- temiz"))
    if n_edge:
        print(f"  (arm sinirinda {n_edge} ornek NaN -- beklenen, olcute dahil degil)")

    ok = (n_nan == 0)

    if "vehicle_attitude" in d:
        t_at = np.asarray(d["vehicle_attitude"]["timestamp"], float) / 1e6
        ww = (t_at >= t_arm) & (t_at <= t_dis)
        gaps = np.diff(t_at[ww])
        if gaps.size:
            n_over = int((gaps > FS_ATT_TIMEOUT_S).sum())
            print(f"  duruş boslugu   : max {gaps.max() * 1e3:.1f} ms, p99 {np.percentile(gaps, 99) * 1e3:.1f} ms, "
                  f">{FS_ATT_TIMEOUT_S * 1e3:.0f} ms sayisi {n_over}  "
                  f"(pay {FS_ATT_TIMEOUT_S / gaps.max():.1f}x)")
            ok = ok and (n_over == 0)

    if "tiltrotor_indi_status" in d:
        st = d["tiltrotor_indi_status"]
        t_st = np.asarray(st["timestamp"], float) / 1e6
        ua = np.column_stack([np.asarray(st["u_actual[%d]" % i], float) for i in range(3)])
        ws = (t_st >= t_arm) & (t_st <= t_dis)
        zero = (ua[ws] == 0.0).all(axis=1)
        print(f"  sifir-doldurulmus status ornegi: {int(zero.sum())} "
              f"(kesme yollarinin imzasi; yukaridaki NaN sayisiyla eslesmeli)")

    print(f"  -> {'TEMIZ' if ok else 'KESINTI VAR'}")
    return ok


def main() -> int:
    paths = sys.argv[1:] or [newest_ulog()]
    all_ok = True
    for p in paths:
        print(f"=== {p} ===")
        all_ok &= check(p)
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
