#!/usr/bin/env python3
"""Geri gecis (cruise -> hover) kriter analizi -- ulog'dan, MAKINE ile.

Neden ayri bir betik: `run_backtrans_test.py`'nin docstring'i bes gecme olcutu
sayiyor ama kosu sirasinda yalnizca ucunu (v_h, irtifa bandi, konsolda
pos_hold) olcebiliyordu; yaw toplam donusu ve itki doyumu ELLE, ad-hoc
komutlarla bakiliyordu. Bu projede tam olarak bu sinif bir bosluk daha once
yanlis "GECTI" uretti (Adim 34e: olcut penceresi olculen sinyalin kendisinden
turetiliyordu). Burada her olcut ulog'dan hesaplanir ve pencere BAGIMSIZ bir
sinyalden (bt_state) gelir.

Olcutler (Adim 31 / Faz 2'de olculen degerlerle birlikte):
  1. v_h son 10 s ortalamasi < 1.0 m/s          (olculen: 0.09-0.12)
  2. HANDOFF durumuna girildi + pos_hold istendi
  3. irtifa bandi <= 3.0 m                       (olculen: 0.95)
  4. |toplam yaw donusu| <= 45 deg               (olculen: 8.1-10.1)
  5. itki sat_flag %0 ve BIG_M = 0
  6. GERI KACIS YOK: govde ileri hizi >= -2.0 m/s (Adim 39, madde (S))

Olcut 6 NEDEN SONRADAN EKLENDI (Adim 39, 2026-08-03): madde (S) bes olcutun
BESINI DE gecerek gozden kacti. Arac ileri yonde durup GERI yonde 12.8 m/s'ye
kactiginda v_h BUYUKLUK olarak buyuyor, ama olcut 1 yalnizca SON 10 s'ye
bakiyor ve manevra zaman asimina ugradiginda o pencere baska bir yerde
olabiliyor; irtifa, yaw ve doyum ise gercekten temizdi. Yani kriter kumesi
"arac dogru YONE gidiyor mu" sorusunu hic sormuyordu. Isaretli buyukluk
olcmek, buyukluk olcmekten farklidir -- testin kendisi de kontrol yasasiyla
ayni hatayi yapiyordu.

Yaw HER ZAMAN hizdan integre edilir, acidan ORNEKLENMEZ (Adim 12b, olcum
tuzagi #1: 5 s'de bir aci ornegi surekli donusu "sinirli gezinme" gosterir).
`tiltrotor_indi_status`'te ~%1.5 yinelenen zaman damgasi var (tuzak #3), bu
yuzden dedupe ediliyor.

Kullanim:
    python3 analyze_backtrans.py [ulog_yolu]     # yoksa en yeni ulog
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np
from pyulog import ULog

ULOG_DIR = "/home/umran/PX4-Autopilot/build/px4_sitl_default/rootfs/log"

# TiltrotorIndiStatus.msg BT_* sabitleri
BT_NAMES = {0: "IDLE", 1: "RETRACT", 2: "BRAKE", 3: "HANDOFF"}

# Gecme esikleri
VH_FINAL_MAX = 1.0     # m/s
ALT_BAND_MAX = 3.0     # m
YAW_TURN_MAX = 45.0    # deg
FINAL_WINDOW_S = 10.0  # s, "orada kaldi mi" penceresi
# RUNAWAY_MAX: madde (S)'nin ASIL zarari "arac frenlerken YENIDEN HIZLANDI"dir
# (olculen: min 3.08 -> 12.8 m/s). Olcut bu yuzden isaretli ileri hiza degil,
# v_h'nin KENDI KOSAN MINIMUMUNDAN ne kadar yukseldigine bakar.
#
# ILK YAZILISI ISARETLI v_fwd ESIGIYDI VE YANLISTI (ayni gun duzeltildi):
# probe_lateral_handoff.py'de arac frenlerken heading 186.7 deg dondu; v_h
# 9.74 -> 3.00 m/s ile MONOTON azalirken v_fwd -3.95'e indi -- cunku v_fwd,
# DONEN bir cerceveye izdusumdur. Yani "geri kacis" diye olculen sey aracin
# hizlanmasi degil, cercevenin donmesiydi. Yeniden-hizlanma olcusu donmeden
# ETKILENMEZ. Ders, madde (S)'nin kendisinin aynasi: bir esik hangi sinyali
# okudugunu bilmek zorunda -- ISARETLI ileri hiz "ne zaman devredeyim"in dogru
# sinyali (fren yasasinin kontrol ettigi eksen), BUYUKLUK ise "kactim mi"nin
# dogru sinyali. Ayni kosuda ikisi de gerekli, ve yerleri degistirilemez.
RUNAWAY_MAX = 1.0      # m/s, fren penceresinde izin verilen yeniden-hizlanma


def newest_ulog() -> str:
    best, best_m = "", 0.0
    for root, _d, files in os.walk(ULOG_DIR):
        for f in files:
            if f.endswith(".ulg"):
                p = os.path.join(root, f)
                if os.path.getmtime(p) > best_m:
                    best, best_m = p, os.path.getmtime(p)
    return best


def analyze(path: str) -> bool:
    u = ULog(path, ["tiltrotor_indi_status", "tiltrotor_indi_setpoint",
                    "vehicle_angular_velocity", "vehicle_local_position",
                    "vehicle_attitude"])
    d = {x.name: x.data for x in u.data_list}

    if "tiltrotor_indi_status" not in d:
        print("HATA: tiltrotor_indi_status ulog'da yok -- logger_topics profili kurulmamis")
        return False

    st = d["tiltrotor_indi_status"]
    ts = np.asarray(st["timestamp"], float)
    keep = np.concatenate(([True], np.diff(ts) > 1e-6))    # olcum tuzagi #3
    ts = ts[keep]
    t_st = ts / 1e6

    sat = np.column_stack([np.asarray(st["sat_flag[%d]" % i], float)[keep] for i in range(6)])
    ua = np.column_stack([np.asarray(st["u_actual[%d]" % i], float)[keep] for i in range(6)])

    have_bt = "bt_state" in st
    if have_bt:
        bt = np.asarray(st["bt_state"], float)[keep]
        ceil = np.asarray(st["bt_tilt_ceil"], float)[keep]
    else:
        # Eski loglar: durum makinesi telemetrisi yoktu. Pencereyi setpoint'in
        # bt_enable bayragindan kur; durum ayrimi yapilamaz.
        print("NOT: bu ulog'da bt_state yok (Adim 37 oncesi) -- pencere bt_enable'dan")
        sp = d.get("tiltrotor_indi_setpoint")
        if sp is None:
            print("HATA: ne bt_state ne tiltrotor_indi_setpoint var")
            return False
        t_sp = np.asarray(sp["timestamp"], float) / 1e6
        en = np.asarray(sp["bt_enable"], float)
        idx = np.nonzero(en > 0.5)[0]
        if idx.size == 0:
            print("HATA: bu kosuda bt_enable hic set edilmemis")
            return False
        # Pencere ILK ve SON bt_enable yayini arasi. Sadece "ilk"ten sonrasi
        # denirse pencere inise ve yere temasa kadar uzar -- orada dogal olarak
        # itki doyumu ve irtifa degisimi var, yani manevra bunlarla haksiz yere
        # suclanir (ilk surumde tam bu oldu: %18 doyum, 8995 BIG_M).
        bt = np.where((t_st >= t_sp[idx[0]]) & (t_st <= t_sp[idx[-1]]), 1.0, 0.0)
        ceil = np.full_like(bt, float("nan"))

    active = bt > 0.5
    if not active.any():
        print("HATA: bt_state hic IDLE disina cikmadi -- geri gecis baslamadi")
        return False

    i0 = int(np.argmax(active))
    t_bt0 = t_st[i0]
    t_end = t_st[active][-1]

    av = d["vehicle_angular_velocity"]
    t_av = np.asarray(av["timestamp"], float) / 1e6
    r = np.asarray(av["xyz[2]"], float)

    lp = d["vehicle_local_position"]
    t_lp = np.asarray(lp["timestamp"], float) / 1e6
    z = np.asarray(lp["z"], float)
    vz = np.asarray(lp["vz"], float)
    vx = np.asarray(lp["vx"], float)
    vy = np.asarray(lp["vy"], float)
    vh = np.hypot(vx, vy)

    # Madde (S): ISARETLI govde ileri hizi ve yanal hiz. Kontrol yasasinin
    # (fren pitch'i) gercekten etkiledigi eksen ileri eksendir; buyukluk,
    # manevranin kaldiramadigi bir yanal bilesenle yukarida tutulabilir.
    at = d.get("vehicle_attitude")
    if at is not None:
        t_at = np.asarray(at["timestamp"], float) / 1e6
        q = np.column_stack([np.asarray(at["q[%d]" % i], float) for i in range(4)])
        yaw_series = np.arctan2(2.0 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]),
                                1.0 - 2.0 * (q[:, 2] ** 2 + q[:, 3] ** 2))
        psi = np.interp(t_lp, t_at, np.unwrap(yaw_series))
        v_fwd = vx * np.cos(psi) + vy * np.sin(psi)
        v_lat = -vx * np.sin(psi) + vy * np.cos(psi)
    else:
        v_fwd = np.full_like(vh, np.nan)
        v_lat = np.full_like(vh, np.nan)

    win_st = (t_st >= t_bt0) & (t_st <= t_end)
    win_av = (t_av >= t_bt0) & (t_av <= t_end)
    win_lp = (t_lp >= t_bt0) & (t_lp <= t_end)

    print(f"=== GERI GECIS ANALIZI ===\n  ulog : {path}")
    print(f"  pencere: t = {t_bt0:.1f} .. {t_end:.1f} s  ({t_end - t_bt0:.1f} s)")

    # --- durum makinesi dizisi ---
    if have_bt:
        print("\n  durum dizisi (tilt tavani gercekten aktuatoru surdu mu):")
        print("    durum      giris_t  sure   v_h giris->cikis   v_fwd giris->cikis   "
              "tavan giris->cikis   max kanat tilt")
        edges = np.nonzero(np.diff(bt) != 0)[0] + 1
        bounds = np.concatenate(([i0], edges[edges > i0], [len(bt) - 1]))
        seq = []
        for a, b in zip(bounds[:-1], bounds[1:]):
            s = int(bt[a])
            if s == 0:
                continue
            ta, tb = t_st[a], t_st[b]
            vha = float(np.interp(ta, t_lp, vh))
            vhb = float(np.interp(tb, t_lp, vh))
            vfa = float(np.interp(ta, t_lp, v_fwd))
            vfb = float(np.interp(tb, t_lp, v_fwd))
            wing = np.degrees(ua[a:b + 1, 3:5].max(axis=1)) if b > a else np.array([float("nan")])
            seq.append((s, ta))
            print(f"    {BT_NAMES.get(s, s):<9} {ta:7.1f}  {tb - ta:5.1f}  "
                  f"{vha:6.2f} -> {vhb:5.2f}      "
                  f"{vfa:+6.2f} -> {vfb:+5.2f}      "
                  f"{math.degrees(ceil[a]):5.1f} -> {math.degrees(ceil[b]):5.1f} deg     "
                  f"{np.nanmax(wing):5.1f} deg")

        # Tavan gercekten BAGLAYICI mi: kanat tilti tavani asmamali (kutu kisiti)
        wing_max = np.degrees(ua[win_st, 3:5].max(axis=1))
        over = wing_max - np.degrees(ceil[win_st])
        print(f"    tavan ihlali (kanat tilt - tavan) max: {over.max():+.2f} deg "
              f"(kutu kisiti: <= 0 olmali)")

    # --- olcutler ---
    tail = (t_lp >= t_end - FINAL_WINDOW_S) & (t_lp <= t_end)
    vh_final = float(vh[tail].mean()) if tail.any() else float("nan")
    alt_band = float(z[win_lp].max() - z[win_lp].min())
    yaw_turn = math.degrees(np.trapezoid(r[win_av], t_av[win_av]))
    thrust_sat = 100.0 * float(sat[win_st, :3].mean())
    big_m = int(sat[win_st, :3].any(axis=1).sum())
    handoff = bool((bt > 2.5).any()) if have_bt else float("nan")
    vz_max = float(np.abs(vz[win_lp]).max())
    vh_entry = float(np.interp(t_bt0, t_lp, vh))

    # Olcut 6'nin PENCERESI, bt penceresinin tamami DEGIL: madde (S) fren
    # yasasinin kusuru, ve fren yasasi pitch'in sahibi yalnizca BRAKE'te ve
    # pos_hold HENUZ DEVREYE GIRMEDIGI HANDOFF'ta. pos_hold devraldiktan sonra
    # geri yonde birkac m/s gormek NORMALDIR -- pozisyon dongusu yakaladigi
    # noktaya donuyordur; ilk kosuda tam bu olctuldu (BRAKE hic +2.98'in altina
    # inmedi, -1.99 pos_hold devraldiktan 2.8 s sonra olustu). Pencereyi
    # genis birakmak testin baska bir yasayi suclamasina yol acardi.
    if "pos_hold_active" in st:
        pha = np.asarray(st["pos_hold_active"], float)[keep] > 0.5
        brake_owner_st = (bt >= 1.5) & (~pha)
        owner_note = ""
    else:
        # Adim 39 oncesi loglar: bayrak yok, BRAKE ile yetin (HANDOFF'ta pitch'in
        # sahibinin kim oldugu bilinemez).
        brake_owner_st = (bt >= 1.5) & (bt <= 2.5)
        owner_note = "  [eski log: yalnizca BRAKE]"

    if brake_owner_st.any():
        t_bo0 = t_st[brake_owner_st][0]
        t_bo1 = t_st[brake_owner_st][-1]
        win_bo = (t_lp >= t_bo0) & (t_lp <= t_bo1)
    else:
        win_bo = np.zeros_like(t_lp, dtype=bool)

    v_fwd_min = float(np.nanmin(v_fwd[win_bo])) if win_bo.any() else float("nan")
    v_lat_max = float(np.nanmax(np.abs(v_lat[win_bo]))) if win_bo.any() else float("nan")
    if win_bo.any():
        vh_bo = vh[win_bo]
        reaccel = float((vh_bo - np.minimum.accumulate(vh_bo)).max())
    else:
        reaccel = float("nan")

    c1 = vh_final < VH_FINAL_MAX
    c2 = bool(handoff) if have_bt else True
    c3 = alt_band <= ALT_BAND_MAX
    c4 = abs(yaw_turn) <= YAW_TURN_MAX
    c5 = (thrust_sat == 0.0) and (big_m == 0)
    c6 = (not math.isfinite(reaccel)) or (reaccel <= RUNAWAY_MAX)

    def mark(ok: bool) -> str:
        return "GECTI" if ok else "KALDI"

    print(f"\n  giris hizi          : {vh_entry:.2f} m/s")
    print(f"  1) v_h son {FINAL_WINDOW_S:.0f} s ort : {vh_final:.2f} m/s  (< {VH_FINAL_MAX}) .......... {mark(c1)}")
    print(f"  2) HANDOFF'a girildi : {handoff} ................................ {mark(c2)}")
    print(f"  3) irtifa bandi      : {alt_band:.2f} m  (<= {ALT_BAND_MAX}) ............... {mark(c3)}")
    print(f"  4) toplam yaw donusu : {yaw_turn:+.1f} deg  (|.| <= {YAW_TURN_MAX}) ......... {mark(c4)}")
    print(f"  5) itki sat / BIG_M  : {thrust_sat:.2f}% / {big_m} ......................... {mark(c5)}")
    print(f"     (|vz| max {vz_max:.2f} m/s, itki {ua[win_st, :3].min():.2f}-{ua[win_st, :3].max():.2f} N)")
    print(f"  6) yeniden-hizlanma  : {reaccel:.2f} m/s  (<= {RUNAWAY_MAX}) ............. {mark(c6)}{owner_note}")
    print(f"     (fren yasasinin pitch'e sahip oldugu pencerede, v_h'nin kosan "
          f"minimumundan yukselisi -- madde (S)'de 3.08 -> 12.8 idi)")
    print(f"     teshis: min v_fwd {v_fwd_min:+.2f} m/s, max |yanal| {v_lat_max:.2f} m/s "
          f"(v_fwd DONEN cerceveye izdusumdur: heading donerse olcut olamaz, "
          f"yalnizca teshistir)")

    # Kanal bazinda itki tabani. sat_flag %0 iken bile bir rotor 0 N'a
    # OTURABILIR: sat_flag WLS'in ARTIMI kutu sinirinda kirpildiginda set olur,
    # mutlak deger tabana degdiginde degil. Adim 11'in ariza imzasi tam olarak
    # "dusuk itkili rotor tabana itiliyor" oldugu icin ayrica raporlanir.
    print("     kanal itki tabani: " + "  ".join(
        f"T{i}: min {ua[win_st, i].min():5.2f} N, <1N %{100.0 * (ua[win_st, i] < 1.0).mean():.1f}"
        for i in range(3)))

    # Madde (S)'nin BILINEN KALINTISI, olcum olarak: handoff ILERI eksende
    # istenir ama pos_hold BUYUKLUK kapisindan gecer. Aradaki sure sifir
    # olmayabilir; olcumu kayitta tutmak, "kisa surer herhalde" demekten
    # farklidir (adim 35'in dersi: bir mekanizmanin GEREKCESI de bir savdir).
    if "pos_hold_active" in st:
        ho = np.nonzero(bt > 2.5)[0]
        eng = np.nonzero(np.asarray(st["pos_hold_active"], float)[keep] > 0.5)[0]
        if ho.size and eng.size and eng[-1] > ho[0]:
            eng_after = eng[eng >= ho[0]]
            wait = (t_st[eng_after[0]] - t_st[ho[0]]) if eng_after.size else float("nan")
            print(f"\n  madde (S) kalintisi : HANDOFF -> pos_hold devri {wait:.2f} s bekledi "
                  f"(buyukluk kapisi POS_ENGAGE_V_MAX)")
        elif ho.size:
            print("\n  madde (S) kalintisi : HANDOFF istendi ama pos_hold HIC devreye girmedi")

    ok = c1 and c2 and c3 and c4 and c5 and c6
    print(f"\n  SONUC: {mark(ok)}")
    return ok


def main() -> int:
    path = sys.argv[1] if len(sys.argv) > 1 else newest_ulog()
    if not path:
        print("HATA: ulog bulunamadi")
        return 2
    return 0 if analyze(path) else 1


if __name__ == "__main__":
    sys.exit(main())
