#!/usr/bin/env python3
"""BRAKE evresinde tilt tavanina ne kadar DAYANILDIGI -- BT_BRAKE_CEIL payi.

Adim 38 (2026-07-31), madde (R) icin yazildi.

NEDEN: adim 37'nin 1. genel dersi -- "bir manevranin 'tamamlandi' kaydi, onun
ne kadar PAYLA tamamlandigini soylemez". BT_RELEASE_V 8 -> 10 yukseltildiginde
BRAKE artik daha HIZLI girilir, ve BRAKE'in kacisi tutan sey bir KUTU KISITI
(BT_BRAKE_CEIL = 20 deg). Tavan baglayici hale geliyorsa manevra tavanin
kendi payini yemeye baslamis demektir; bu, esigin daha da yukseltilemeyecegini
soyleyen olcumdur.

Olculen: BRAKE penceresinde kanat tilt komutunun tavana dayandigi sure yuzdesi
ve max degeri. Pencere -- her zamanki gibi -- olculen sinyalden BAGIMSIZ bir
sinyalden (bt_state) kurulur.

Kullanim:
    python3 brake_ceiling_margin.py <ulog> [<ulog> ...]
"""

from __future__ import annotations

import math
import sys

import numpy as np
from pyulog import ULog

PIN_TOL_DEG = 0.20   # bu kadar yakinsa "tavana dayali" sayilir


def margin(path: str) -> None:
    ulog = ULog(path, ["tiltrotor_indi_status"])
    d = {m.name: m.data for m in ulog.data_list}
    st = d.get("tiltrotor_indi_status")
    if st is None or "bt_state" not in st:
        print(f"{path}: bt_state yok -- atlandi")
        return

    t = np.asarray(st["timestamp"], float) / 1e6
    keep = np.concatenate(([True], np.diff(t) > 1e-6))   # tuzak #3: yinelenen damga
    t = t[keep]
    bt = np.asarray(st["bt_state"], float)[keep]
    ceil = np.degrees(np.asarray(st["bt_tilt_ceil"], float)[keep])
    wing = np.degrees(np.maximum(np.asarray(st["u_actual[3]"], float)[keep],
                                 np.asarray(st["u_actual[4]"], float)[keep]))

    win = bt == 2   # BRAKE
    if not win.any():
        print(f"{path}: BRAKE evresi yok")
        return

    # Gecis tick'ini AT: tavan 9 -> 20 deg'e o tick'te siciyor ve kanat tilt'i
    # hala 9 deg'de oldugu icin pay bir ornek boyunca 0 olcuyor. Ayiklanmazsa
    # her ucus "tavana dayandi" gorunur -- olcumu anlamsizlastiran bir sinir
    # artefakti (adim 37'nin iki ucusunda tam olarak bu cikti: %0.3 = 1 ornek).
    t_brake0 = t[win][0]
    win &= t >= t_brake0 + 0.1

    gap = ceil[win] - wing[win]
    pinned = gap <= PIN_TOL_DEG
    dt = np.diff(t[win], prepend=t[win][0])
    t_pin = float(dt[pinned].sum())
    t_tot = float(t[win][-1] - t[win][0])

    print(f"{path.split('/')[-1]}:")
    print(f"  BRAKE suresi        : {t_tot:5.1f} s")
    print(f"  tavana dayali sure  : {t_pin:5.1f} s  ({100.0 * t_pin / max(t_tot, 1e-6):.1f}%)")
    print(f"  en kucuk pay        : {gap.min():+.2f} deg  (tavan {ceil[win].max():.1f} deg)")
    print(f"  max kanat tilt      : {wing[win].max():5.1f} deg")


if __name__ == "__main__":
    for p in sys.argv[1:]:
        margin(p)
