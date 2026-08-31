#!/usr/bin/env python3
"""Kademeli failsafe (engelleyici B1, Adim 32) SITL testi -- her seviye icin bir ucus.

NEDEN GEREKLI. Adim 32 kademeli failsafe'i yazdi ve `sitl-lockup-check`'i gecti,
ama o kosuda `failsafe_level` %100 sifirda kaldi: bozulma dallari da sonlandirma
yolu da HIC CALISTIRILMAMIS koddu. Bu betik kestirimi gercekten elinden alarak
her dali tetikler.

ADIM 35 (2026-07-30) -- SEVIYE 3'UN ANLAMI DEGISTI. `FsLevel::RATE_ONLY`
KALDIRILDI. Adim 34'te seviye 3 aracin 35 m'den serbest dusmesiyle sonuclandi;
ardindan MATLAB (`run_rate_only_test.m`, orada commander yok) savi saf haliyle
olctu ve CURUTTU: uc senaryonun ucu de ters dondu ve yere carpti, en temizi
179.6 deg roll ile 11.7 m/s'de (serbest dusus 26.2 m/s). Duruş kestirimi artik
kademelendirilebilir bir girdi degil, SERT ON KOSUL: kaybolursa modul cikisi
kesiyor. Bu yuzden `--level 3` artik "bozulup uctu mu" degil, "duruş kaybinda
temiz, kayitli ve ZAMANINDA kesti mi" testidir -- olcutleri de tersine cevrildi.

TASARIM KARARI: seviyeyi zorlayan bir konsol kancasi (`slewbox`/`tiltceil`
tarzi) DELIBERATELY kullanilmadi. Kod her dongude o donginin KENDI girdisinin
gecerliligine bakiyor (`alt_ok`, `pos_ok`), `_fs_level` yalnizca en-kotu
OZETI -- yani seviyeyi zorlamak dal govdelerini calistirmazdi. Bunun yerine
EKF2'nin fuzyon kaynaklari kapatiliyor / ekf2 durduruluyor: algilama ile tepki
birlikte sinaniyor ve rebuild gerekmiyor.

SEVIYE 1 (NO_POS)   : EKF2_GPS_CTRL = 2  (yalnizca yukseklik fuzyonu kalir)
                      -> yatay yardim yok -> xy_valid false, irtifa saglam.
SEVIYE 2 (NO_ALT)   : EKF2_BARO_CTRL = 0 + EKF2_GPS_CTRL = 0 (TUM yardim kesilir)
                      -> EKF2 `fake_hgt` fuzyonuna duser, `isLocalVertical*Valid()`
                      ikisi de false olur (ekf.h:294-302'deki `&& !fake_hgt`),
                      z_valid/v_z_valid ANINDA duser. Adim 36'da olculdu: fs=2
                      ilk ornekte geliyor, zaman asimi beklemeye gerek yok.

                      IKI ONCEKI DENEME NEDEN TUTMADI (adim 34, "ulasilamaz"
                      diye kayda gecmisti -- YANLIS). Ikisi de en az bir yardim
                      kaynagini ayakta biraktı; ozellikle GPS lon/lat acik
                      kalinca (GPS_CTRL=1) dikey dead-reckon bayraklari HIC
                      dolmuyor. Adim 36 probe'u (probe_no_alt.py) bunu olctu ve
                      cok daha kotu bir sey buldu: o konfigurasyonda EKF'in z'si
                      -20.04'te DONUYOR ve `z_valid` TRUE kaliyor, oysa arac
                      gercekten 19.95 -> 10.15 m aliciliyor. Yani seviye 2
                      tetiklenmiyordu cunku KESTIRIM SESSIZCE YANLISTI, kendini
                      gecersiz ilan etmiyordu. Bu ayri bir tehlike ve kontrol
                      listesine ayri kalem olarak yazildi.

                      NOT: bu enjeksiyon xy'yi de goturur (fake_pos), yani izole
                      bir "z gitti, xy iyi" testi DEGIL. Merdiven !alt_ok'u once
                      kontrol ettigi icin seviye yine 2 raporlanir ve dikey dalin
                      GOVDESI calisir -- olcmek istedigimiz sey o.
SEVIYE 3 (DURUŞ KAYBI): ekf2 stop -> vehicle_attitude VE vehicle_local_position
                      birlikte durur, ama vehicle_angular_velocity (Run()'i
                      TETIKLEYEN topic) akmaya devam eder. Duruş 50 ms'de
                      (FS_ATT_TIMEOUT_US), lpos 200 ms'de bayatlar, yani sert on
                      kosul once tetiklenir ve seviye hic bildirilmez.
                      BEKLENEN: motorlara NaN, tek seferlik "attitude LOST"
                      hatasi, ve fs alanı 3'e ASLA cikmaz.

Her ucus `commander lockdown on` ile bitiyor: sonlandirma YOLUNUN hala
motorlari kestigini dogrular (o yolda kesmek DOGRU davranis).

UYARI -- bozulma sirasinda yatay surukleme BEKLENIYOR. Madde (N): tek yonlu
tilt araligi yuzunden yaw trim'i kalici bir ileri kuvvet uretir ve onu tutan
tek sey `pos_hold`; xy gecersizken pozisyon donusu kapanmak ZORUNDA. Yani arac
ileri kacar. Bu bir kusur degil, olculecek bir sonuc.

Kullanim:
    python3 run_failsafe_test.py --level 1
"""

from __future__ import annotations

import argparse
import math
import os
import shutil
import sys
import time

import numpy as np
from pyulog import ULog

import indi_sitl_common as sc

CLIMB_STAGE = 5.0      # m, kademeli tirmanis (tek adimda 20 m istemiyoruz)
BASELINE_S = 12.0
RESTORED_S = 12.0
LOCKDOWN_S = 4.0

# Irtifa payi ve bozulma suresi seviyeye gore. Seviye 2'de dikey kanal ACIK
# CEVRIME dusuyor (`_fz_sp = -MASS*GRAVITY*FS_FZ_OPENLOOP`, 0.97), yani net
# 0.29 m/s^2 asagi: 12 s'de ~21 m. 20 m'den baslamak yere carpmak demek, o yuzden
# 35 m'den kosuyor. Seviye 1'de irtifa dongusu calismaya devam ettigi icin 20 m
# yeterli. Seviye 3'te arac ZATEN dusecek (cikis kesiliyor) -- 35 m yalnizca
# olcume yer birakmak icin; bu kosuda "dusmedi" bir olcut DEGIL.
PROFILE = {
    1: dict(alt=20.0, degraded_s=18.0),
    2: dict(alt=35.0, degraded_s=12.0),
    3: dict(alt=35.0, degraded_s=8.0),
}

LOG_TOPICS_SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logger_topics_shadow.txt")
LOG_TOPICS_DST_DIR = "/home/umran/PX4-Autopilot/build/px4_sitl_default/rootfs/etc/logging"
LOG_TOPICS_DST = os.path.join(LOG_TOPICS_DST_DIR, "logger_topics.txt")
ULOG_DIR = "/home/umran/PX4-Autopilot/build/px4_sitl_default/rootfs/log"

# 3 yok: RATE_ONLY adim 35'te kaldirildi ve deger bir daha kullanilmayacak
# (adim 35 oncesi loglarda "duruş gitti ama hala uçuyor" demek). Bir ulog'da
# 3 gorulmesi eski bir binary demektir.
LEVEL_NAME = {0: "NONE", 1: "NO_POS", 2: "NO_ALT"}
# --level 3 artik bir failsafe SEVIYESI degil, bir senaryo adi.
SCENARIO_NAME = {1: "NO_POS", 2: "NO_ALT", 3: "DURUS KAYBI (cikis kesilmeli)"}

# Gazebo yer-gercegi ornekleri: (faz, faz_ici_t, failsafe_level, gz_z).
# Neden gerekli: her PX4 irtifa sinyali tam da bozdugumuz kestirimden geciyor,
# yani bozulma sirasinda ARACIN GERCEKTEN DUSUP DUSMEDIGI ulog'dan okunamaz.
TRUTH: list[tuple[str, float, int, float]] = []


def newest_ulog() -> str:
    best, best_m = "", 0.0
    for root, _d, files in os.walk(ULOG_DIR):
        for f in files:
            if f.endswith(".ulg"):
                p = os.path.join(root, f)
                if os.path.getmtime(p) > best_m:
                    best, best_m = p, os.path.getmtime(p)
    return best


def inject(px4: sc.Px4Client, level: int) -> None:
    if level == 1:
        px4.param_set("EKF2_GPS_CTRL", 2)
    elif level == 2:
        # GPS_CTRL = 0, 1 DEGIL: tek bir yardim kaynagi bile ayakta kalirsa EKF
        # kendini gecerli saymaya devam eder (bkz. yukaridaki not).
        px4.param_set("EKF2_BARO_CTRL", 0)
        px4.param_set("EKF2_GPS_CTRL", 0)
    elif level == 3:
        print("  ekf2 stop ->", px4.stop_module("ekf2").strip()[:120])


def restore(px4: sc.Px4Client, level: int) -> None:
    if level == 1:
        px4.param_set("EKF2_GPS_CTRL", 7)
    elif level == 2:
        px4.param_set("EKF2_BARO_CTRL", 1)
        px4.param_set("EKF2_GPS_CTRL", 7)
    # Senaryo 3 geri alinamaz: ekf2 yeniden baslasa bile yeniden hizalanmasi
    # gerekir -- ve zaten olculen sey cikisin kesilmesi, geri donus degil.


def observe(px4: sc.Px4Client, seconds: float, yaw_sp: float, z_sp: float,
            label: str, keep_setpoint: bool = True) -> None:
    """Setpoint'i tazeleyerek bekle ve saniyede bir yer-gercegi irtifasini bas.

    Setpoint tazelemesi SUS DEGIL: modulun kendi girdisi olay-guduml (bkz.
    FsLevel notu), ama `pos_hold` istegi ve z_sp'yi tutmak icin yayina devam
    ediyoruz -- boylece failsafe'in setpoint'i degil KESTIRIMI kaybetmenin
    sonucu oldugu ayrik kalir.
    """
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        if keep_setpoint:
            px4.set_setpoint(roll=0.0, pitch=0.0, yaw=yaw_sp, fx=0.0, z_sp=z_sp,
                             leso_enable=(True, True, False), pos_hold=True)
        st = px4.status()
        fs = int(st.get("failsafe_level", -1))
        gz_z = px4.gz_truth_z()
        TRUTH.append((label, time.monotonic() - t0, fs, gz_z))
        print(f"  [{label}] t={time.monotonic() - t0:5.1f}s  fs={fs}({LEVEL_NAME.get(fs, '?')})"
              f"  gz_z={gz_z:6.2f} m")
        time.sleep(2.0)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--level", type=int, choices=(1, 2, 3), required=True)
    args = ap.parse_args()
    level = args.level
    hover_alt = PROFILE[level]["alt"]
    degraded_s = PROFILE[level]["degraded_s"]

    px4 = sc.Px4Client()
    out_dir = os.path.dirname(os.path.abspath(__file__))
    os.makedirs(LOG_TOPICS_DST_DIR, exist_ok=True)
    shutil.copyfile(LOG_TOPICS_SRC, LOG_TOPICS_DST)

    print(f"=== SITL baslatiliyor (senaryo {level} = {SCENARIO_NAME[level]}) ===")
    sc.launch_sitl("gz_tiltrotor_indi", os.path.join(out_dir, f"px4_failsafe_L{level}.log"))

    try:
        if not sc.wait_until(px4.preflight_check_ok, timeout=20.0, poll_interval=0.5):
            print("UYARI: preflight temizlenmedi")

        px4.arm(force=True)
        time.sleep(0.5)
        z0 = px4.local_position().get("z", 0.0)

        yaw0_deg = float("nan")
        for _ in range(20):
            _, _, yaw0_deg = px4.attitude_euler_deg()
            if math.isfinite(yaw0_deg):
                break
            time.sleep(0.5)
        if not math.isfinite(yaw0_deg):
            raise RuntimeError("heading okunamadi")
        yaw_sp = math.radians(yaw0_deg)
        print(f"arm heading = {yaw0_deg:.2f} deg -> yaw_sp")

        # Kademeli tirmanis. pos_hold ilk anda aciliyor: POS_ENGAGE_V_MAX = 3 m/s
        # kapisi var, yani once surüklenip sonra acmaya kalkmak REDDEDILIR.
        z_sp = z0
        while z_sp > z0 - hover_alt + 1e-6:
            z_sp = max(z0 - hover_alt, z_sp - CLIMB_STAGE)
            for _ in range(5):
                px4.set_setpoint(roll=0.0, pitch=0.0, yaw=yaw_sp, fx=0.0, z_sp=z_sp,
                                 leso_enable=(True, True, False), pos_hold=True)
                time.sleep(1.0)
        print(f"tirmanis tamam, z_sp = {z_sp:.1f} m")

        observe(px4, BASELINE_S, yaw_sp, z_sp, "baseline")

        print(f"=== ENJEKSIYON: senaryo {level} ({SCENARIO_NAME[level]}) ===")
        inject(px4, level)
        observe(px4, degraded_s, yaw_sp, z_sp, "degraded")

        if level != 3:
            print("=== GERI ALMA ===")
            restore(px4, level)
            observe(px4, RESTORED_S, yaw_sp, z_sp, "restored")

        print("=== SONLANDIRMA: commander lockdown on ===")
        print("  ->", px4.lockdown(True).strip()[:120])
        observe(px4, LOCKDOWN_S, yaw_sp, z_sp, "lockdown")

        # Logger'in dosyayi DUZGUN kapatmasi icin: kill_sitl() ile process'i
        # oldurmek ulog'un kuyrugunu yarim birakiyor.
        px4.disarm(force=True)
        time.sleep(2.0)

    finally:
        sc.kill_sitl()
        try:
            os.remove(LOG_TOPICS_DST)
        except FileNotFoundError:
            pass

    if level == 3:
        return analyze_att_loss(newest_ulog())

    if level == 2:
        return analyze_no_alt(newest_ulog())

    return analyze(newest_ulog(), level)


def analyze(f: str, level: int) -> int:
    """Pencereleri ULOG'UN KENDISINDEN cikarir (duvar saati eslemesi yok):
    arm ani `actuator_armed`'dan, bozulma penceresi `failsafe_level != 0`
    araliginin kendisinden, sonlandirma `actuator_armed.lockdown`'dan."""
    u = ULog(f, ["tiltrotor_indi_status", "vehicle_angular_velocity",
                 "vehicle_local_position", "actuator_motors", "actuator_armed"])
    d = {x.name: x.data for x in u.data_list}

    st = d["tiltrotor_indi_status"]
    ts = np.asarray(st["timestamp"], float)
    keep = np.concatenate(([True], np.diff(ts) > 1e-6))    # olcum tuzagi #3
    ts = ts[keep]
    t0 = ts[0]
    tS = (ts - t0) / 1e6
    fs = np.asarray(st["failsafe_level"], float)[keep]
    sat = np.column_stack([np.asarray(st["sat_flag[%d]" % i], float)[keep] for i in range(6)])
    ua = np.column_stack([np.asarray(st["u_actual[%d]" % i], float)[keep] for i in range(6)])

    am = d["actuator_motors"]
    tM = (np.asarray(am["timestamp"], float) - t0) / 1e6
    mot = np.column_stack([np.asarray(am["control[%d]" % i], float) for i in range(3)])

    ar = d["actuator_armed"]
    tA = (np.asarray(ar["timestamp"], float) - t0) / 1e6
    lock = np.asarray(ar["lockdown"], float)

    av = d["vehicle_angular_velocity"]
    tW = (np.asarray(av["timestamp"], float) - t0) / 1e6
    w = np.column_stack([np.asarray(av["xyz[%d]" % i], float) for i in range(3)])

    lp = d["vehicle_local_position"]
    tL = (np.asarray(lp["timestamp"], float) - t0) / 1e6
    z = np.asarray(lp["z"], float)
    vz = np.asarray(lp["vz"], float)
    vh = np.hypot(np.asarray(lp["vx"], float), np.asarray(lp["vy"], float))

    deg = fs != 0
    if not deg.any():
        print("\nBASARISIZ: failsafe_level hic 0'dan cikmadi -- enjeksiyon tutmadi.")
        print(f"  ulog: {f}")
        return 1

    i_deg = int(np.argmax(deg))
    j_deg = len(deg) - 1 - int(np.argmax(deg[::-1]))
    t_deg0, t_deg1 = tS[i_deg], tS[j_deg]

    t_lock = float(tA[np.argmax(lock > 0.5)]) if (lock > 0.5).any() else float("inf")
    # Bozulma penceresi lockdown'da bitmeli: sonlandirmadan sonrasi baska bir test.
    #
    # BU KIRPMA BIR YANLIS GECIS URETTI (2026-07-30, seviye 3): commander
    # kestirim olunce nav_state'i NAVIGATION_STATE_TERMINATION'a aliyor ve
    # `actuator_armed.lockdown` bozulmanin algilanmasindan 0.05 s SONRA true
    # oluyor. Pencere boylece TEK ORNEGE cokuyor, her olcut o tek ornekte
    # hesaplaniyor ve arac 35 m serbest dusup carptigi halde "GECTI" cikiyordu.
    # Bu yuzden asagida hem asgari pencere suresi hem de "bozulma sirasinda
    # lockdown yoktu" olcutu ZORUNLU.
    t_deg1 = min(t_deg1, t_lock)
    # "Onceledi" NIYET EDILEN sureye gore olculur, gozlenene gore DEGIL: termination
    # modulun `_fs_level`'ini NONE'a sifirliyor (publishDisarmed yolu), yani gozlenen
    # pencere lockdown ile birlikte kapaniyor ve kendisiyle karsilastirilamaz.
    lock_preempted = t_lock < t_deg0 + 0.5 * PROFILE[level]["degraded_s"]

    windows = [("baseline", max(0.0, t_deg0 - BASELINE_S), t_deg0),
               ("degraded", t_deg0, t_deg1)]
    if t_deg1 + 1.0 < t_lock:
        windows.append(("restored", t_deg1, min(t_deg1 + RESTORED_S, t_lock)))
    if math.isfinite(t_lock):
        windows.append(("lockdown", t_lock, tS[-1]))

    seen = sorted({int(v) for v in np.unique(fs)})
    print(f"\n=== ULOG {os.path.basename(f)} ===")
    print(f"  gorulen failsafe seviyeleri: {[f'{v}({LEVEL_NAME.get(v, chr(63))})' for v in seen]}")
    print(f"  bozulma penceresi          : {t_deg0:.1f} - {t_deg1:.1f} s ({t_deg1 - t_deg0:.0f} s)")

    hdr = f"\n  {'pencere':<10} {'fs':>10} {'motor NaN':>10} {'motor min-max':>14} " \
          f"{'itki N':>13} {'sat%':>6} {'|w|max':>7} {'vz':>7} {'v_h':>6} {'z':>7}"
    print(hdr)
    print("  " + "-" * (len(hdr) - 4))

    results = {}
    for name, a, b in windows:
        kS = (tS >= a) & (tS < b)
        kM = (tM >= a) & (tM < b)
        kW = (tW >= a) & (tW < b)
        kL = (tL >= a) & (tL < b)
        if not kS.any() or not kM.any():
            continue

        nan_frac = 100.0 * float(np.isnan(mot[kM]).mean())
        fs_mode = int(np.bincount(fs[kS].astype(int)).argmax())
        finite = mot[kM][~np.isnan(mot[kM])]
        mm = f"{finite.min():.2f}/{finite.max():.2f}" if finite.size else "  --  "
        thr = f"{ua[kS, :3].min():5.2f}-{ua[kS, :3].max():5.2f}"
        satp = 100.0 * float(sat[kS, :3].mean())
        wmax = float(np.abs(w[kW]).max()) if kW.any() else float("nan")
        vzs = f"{np.abs(vz[kL]).max():.2f}" if kL.any() else "  --"
        vhs = f"{vh[kL].mean():.1f}" if kL.any() else " --"
        zs = f"{z[kL].mean():.1f}" if kL.any() else "  --"

        print(f"  {name:<10} {fs_mode:>10} {nan_frac:>9.1f}% {mm:>14} {thr:>13} "
              f"{satp:>5.1f}% {wmax:>7.3f} {vzs:>7} {vhs:>6} {zs:>7}")
        results[name] = dict(fs=fs_mode, nan=nan_frac, sat=satp, wmax=wmax,
                             big_m=int(sat[kS, :3].any(axis=1).sum()))

    # Yer-gercegi alcalma hizi: bozulma fazinda arac DUSTU mu? ulog'daki z
    # bozulan kestirimden geliyor, yani bunu yalnizca gz soyleyebilir.
    dz_rate = float("nan")
    dseg = [(t, z_) for lab, t, _fsv, z_ in TRUTH if lab == "degraded" and math.isfinite(z_)]
    if len(dseg) >= 2 and (dseg[-1][0] - dseg[0][0]) > 1.0:
        dz_rate = (dseg[0][1] - dseg[-1][1]) / (dseg[-1][0] - dseg[0][0])   # +: aliciliyor

    print("\n  --- olcutler ---")
    dg = results.get("degraded", {})
    checks = []
    # ONCE gecerlilik: bunlar tutmazsa asagidaki metrikler bir avuc ornekten
    # hesaplanmis olur ve anlamsizdir (bkz. yukaridaki yanlis-gecis notu).
    checks.append(("bozulma penceresi anlamli uzunlukta",
                   (t_deg1 - t_deg0) >= 0.5 * PROFILE[level]["degraded_s"],
                   f"{t_deg1 - t_deg0:.1f} s / beklenen ~{PROFILE[level]['degraded_s']:.0f} s"))
    checks.append(("bozulma SONLANDIRMA ile kesilmedi", not lock_preempted,
                   "commander lockdown bozulmayi onceledi" if lock_preempted else "lockdown yok"))
    if dseg:
        checks.append(("bozulmada arac dusmedi (<2 m/s alcalma)",
                       math.isfinite(dz_rate) and dz_rate < 2.0,
                       f"gz alcalma {dz_rate:.2f} m/s"))
    else:
        # Ulog'u sonradan yeniden analiz ederken (canli kosu degil) gz ornekleri
        # yok; olcut atlanir, sessizce GECMIS sayilmaz.
        print("  [ATLA ] bozulmada arac dusmedi -- gz yer-gercegi ornegi yok (offline analiz)")
    checks.append(("bozulma seviyesi dogru", dg.get("fs") == level, f"fs={dg.get('fs')}, beklenen {level}"))
    checks.append(("bozulmada motor KESILMEDI", dg.get("nan", 100.0) == 0.0, f"NaN %{dg.get('nan', float('nan')):.1f}"))
    checks.append(("bozulmada aktuator kilitlenmesi yok", dg.get("big_m", 1) == 0, f"BIG_M {dg.get('big_m')}"))
    checks.append(("bozulmada acisal hizlar sinirli (<1.5 rad/s)",
                   dg.get("wmax", 9.9) < 1.5, f"|w|max {dg.get('wmax', float('nan')):.3f}"))
    if "restored" in results:
        checks.append(("geri alindiginda temizlendi", results["restored"]["fs"] == 0,
                       f"fs={results['restored']['fs']}"))
    if "lockdown" in results:
        checks.append(("sonlandirma motorlari KESTI", results["lockdown"]["nan"] > 50.0,
                       f"NaN %{results['lockdown']['nan']:.1f}"))

    ok = True
    for label, passed, detail in checks:
        print(f"  [{'OK ' if passed else 'HAYIR'}] {label}  ({detail})")
        ok &= bool(passed)

    print(f"\n  SONUC: {'GECTI' if ok else 'KALDI'}")
    print(f"  ulog: {f}")
    return 0 if ok else 1


def analyze_no_alt(f: str) -> int:
    """Senaryo 2: NO_ALT ULASILABILIR (adim 34'un "ulasilamaz" hukmu YANLIS),
    ama SURESI modulun elinde degil.

    Adim 36'da olculen tablo. `probe_no_alt.py` uc konfigurasyon denedi:
      A) BARO_CTRL=0, GPS_CTRL=1              -> z_valid TRUE kaldi, z DONDU
      C) A + HGT_REF=2 (var olmayan kaynak)   -> ayni: z_valid TRUE, z dondu
      B) BARO_CTRL=0, GPS_CTRL=0              -> fake_hgt -> z_valid ANINDA false
    Yani "z gecersiz, xy gecerli" durumu PX4 param yuzeyinden URETILEMIYOR:
    yukseklik yardimi kesilip yatay yardim devam ederse EKF2 dikey kestirimi
    GECERLI ilan etmeye devam ediyor. Seviye 2'ye ancak TUM yardimi keserek
    varilir, o da xy'yi goturur, o da commander'i ~1 s icinde TERMINATION'a
    sokar -- RATE_ONLY'yi olduren yapisal tavanin aynisi (adim 34/35).

    Bu yuzden olcut MODULUN KONTROL ETTIGI seye indirgendi: seviye gercekten
    raporlandi mi, ve o sirada modul motorlari kesmedi mi. Commander'in
    sonlandirmasi bir BASARISIZLIK degil, kaydedilen bir TAVAN.
    """
    u = ULog(f, ["tiltrotor_indi_status", "actuator_motors", "actuator_armed"])
    d = {x.name: x.data for x in u.data_list}

    st = d["tiltrotor_indi_status"]
    ts = np.asarray(st["timestamp"], float)
    keep = np.concatenate(([True], np.diff(ts) > 1e-6))    # olcum tuzagi #3
    ts = ts[keep]
    t0 = ts[0]
    tS = (ts - t0) / 1e6
    fs = np.asarray(st["failsafe_level"], float)[keep]
    sat = np.column_stack([np.asarray(st["sat_flag[%d]" % i], float)[keep] for i in range(6)])

    am = d["actuator_motors"]
    tM = (np.asarray(am["timestamp"], float) - t0) / 1e6
    mot = np.column_stack([np.asarray(am["control[%d]" % i], float) for i in range(3)])
    nan_row = np.isnan(mot).all(axis=1)

    ar = d["actuator_armed"]
    tA = (np.asarray(ar["timestamp"], float) - t0) / 1e6
    lock = np.asarray(ar["lockdown"], float)
    t_lock = float(tA[np.argmax(lock > 0.5)]) if (lock > 0.5).any() else float("inf")

    at2 = fs == 2
    seen = sorted({int(v) for v in np.unique(fs)})

    print(f"\n=== ULOG {os.path.basename(f)} (senaryo 2: NO_ALT) ===")
    print(f"  gorulen failsafe seviyeleri : {seen}")

    if not at2.any():
        print("  fs=2 HIC gorulmedi -- enjeksiyon tutmadi.")
        print(f"\n  SONUC: KALDI\n  ulog: {f}")
        return 1

    t2a, t2b = float(tS[at2][0]), float(tS[at2][-1])
    k = at2 & (tS >= t2a) & (tS <= t2b)
    kM = (tM >= t2a) & (tM <= min(t2b, t_lock))
    nan2 = 100.0 * float(nan_row[kM].mean()) if kM.any() else float("nan")
    big_m = int(sat[k, :3].any(axis=1).sum())

    print(f"  NO_ALT penceresi            : {t2a:.2f} - {t2b:.2f} s ({t2b - t2a:.2f} s)")
    print(f"  commander lockdown          : {t_lock:.2f} s  (fs=2 baslangicindan {t_lock - t2a:+.2f} s)")
    print(f"  NO_ALT sirasinda motor NaN  : %{nan2:.1f}")
    print(f"  NO_ALT sirasinda BIG_M      : {big_m}")

    checks = [
        # Adim 34'un hukmunu duzelten olcut: seviye GERCEKTEN raporlandi.
        ("NO_ALT (fs=2) raporlandi -- 'ulasilamaz' degil", True, f"{t2b - t2a:.2f} s"),
        ("NO_ALT sirasinda modul motorlari KESMEDI", nan2 == 0.0, f"NaN %{nan2:.1f}"),
        ("NO_ALT sirasinda aktuator kilitlenmesi yok", big_m == 0, f"BIG_M {big_m}"),
    ]

    print("\n  --- olcutler (yalnizca modulun kontrol ettigi seyler) ---")
    ok = True
    for label, passed, detail in checks:
        print(f"  [{'OK ' if passed else 'HAYIR'}] {label}  ({detail})")
        ok &= bool(passed)

    print("\n  --- kaydedilen TAVAN (olcut degil) ---")
    print(f"  Seviye 2'ye yalnizca TUM yardimi keserek varilabiliyor; bu xy'yi de")
    print(f"  goturuyor ve commander {t_lock - t2a:+.2f} s icinde TERMINATION'a giriyor.")
    print(f"  Yani NO_ALT dal govdesi SITL'de anlamli bir sure calistirilamiyor --")
    print(f"  RATE_ONLY'yi olduren yapisal tavanin aynisi (adim 34/35).")

    print(f"\n  SONUC: {'GECTI' if ok else 'KALDI'}")
    print(f"  ulog: {f}")
    return 0 if ok else 1


def analyze_att_loss(f: str) -> int:
    """Senaryo 3: duruş kaybi bir failsafe SEVIYESI degil, sert on kosul.

    Adim 34'te bu kosu "seviye 3 calisti mi" diye bakiyordu ve arac dustu; adim
    35'te RATE_ONLY kaldirildi, yani artik dogru davranis KESMEK. Olculen sey de
    o: kesme zamaninda mi geldi, MODULUN KENDISINDEN mi geldi (commander'in
    lockdown'indan once ya da onunla ayni anda), ve tam mi.

    Zamanlama referansi `vehicle_attitude`'un SON ornegi: ekf2 durunca o topic
    susuyor, ve modul FS_ATT_TIMEOUT_US = 50 ms sonra bayatlik goruyor. ulog'un
    z'si bu noktadan sonra donuk oldugu icin irtifa yalnizca gz'den okunabilir
    (TRUTH), ve bu kosuda dusus BEKLENEN sonuc -- olcut degil.
    """
    u = ULog(f, ["tiltrotor_indi_status", "vehicle_attitude", "actuator_motors",
                 "actuator_armed"])
    d = {x.name: x.data for x in u.data_list}

    st = d["tiltrotor_indi_status"]
    ts = np.asarray(st["timestamp"], float)
    keep = np.concatenate(([True], np.diff(ts) > 1e-6))    # olcum tuzagi #3
    ts = ts[keep]
    t0 = ts[0]
    tS = (ts - t0) / 1e6
    fs = np.asarray(st["failsafe_level"], float)[keep]

    at = d["vehicle_attitude"]
    tAtt = (np.asarray(at["timestamp"], float) - t0) / 1e6

    am = d["actuator_motors"]
    tM = (np.asarray(am["timestamp"], float) - t0) / 1e6
    mot = np.column_stack([np.asarray(am["control[%d]" % i], float) for i in range(3)])
    nan_row = np.isnan(mot).all(axis=1)

    ar = d["actuator_armed"]
    tA = (np.asarray(ar["timestamp"], float) - t0) / 1e6
    arm = np.asarray(ar["armed"], float)
    lock = np.asarray(ar["lockdown"], float)
    t_arm = float(tA[np.argmax(arm > 0.5)]) if (arm > 0.5).any() else float("nan")
    t_lock = float(tA[np.argmax(lock > 0.5)]) if (lock > 0.5).any() else float("inf")

    # Duruş yayininin bittigi an = SON vehicle_attitude ornegi. `ekf2 stop`
    # sonrasi topic bir daha yayin YAPMIYOR, yani zaman serisinde bir BOSLUK
    # olusmuyor, seri bitiyor -- ilk yazdigim "en buyuk boslugu bul" sezgisi tam
    # bu yuzden nan dondu. Diger topicler (actuator_motors, angular_velocity)
    # kosunun sonuna kadar aktigi icin bu son ornek gercekten enjeksiyon anidir.
    t_att_last = float(tAtt[-1]) if len(tAtt) else float("nan")

    after = nan_row & (tM > t_att_last) if math.isfinite(t_att_last) else np.zeros_like(nan_row)
    t_cut = float(tM[int(np.argmax(after))]) if after.any() else float("nan")

    print(f"\n=== ULOG {os.path.basename(f)} (senaryo 3: duruş kaybi) ===")
    print(f"  son vehicle_attitude ornegi : {t_att_last:.2f} s")
    print(f"  ilk tam-NaN aktuator ornegi : {t_cut:.2f} s")
    print(f"  commander lockdown          : {t_lock:.2f} s  (kesmeden {t_lock - t_cut:+.3f} s sonra)")
    print(f"  gorulen failsafe seviyeleri : {sorted({int(v) for v in np.unique(fs)})}")

    # Enjeksiyondan ONCEKI pencere: adim 32'nin isaretsiz-tasma hatasi burada
    # ucus basina 7-13 kez havada NaN uretiyordu. Sifir kalmali. ARM'dan sonrasi
    # sayilir -- disarmed'ken publishDisarmed() zaten surekli NaN yaziyor, o
    # dogru davranis ve olcume karistirilirsa olcut anlamsizlasir.
    pre = ((tM > t_arm + 2.0) & (tM < t_att_last - 1.0)) if math.isfinite(t_att_last) \
        else np.zeros_like(nan_row)
    pre_nan = 100.0 * float(nan_row[pre].mean()) if pre.any() else float("nan")
    post = (tM >= t_cut) & (tM < t_cut + 2.0) if math.isfinite(t_cut) else np.zeros_like(nan_row)
    post_nan = 100.0 * float(nan_row[post].mean()) if post.any() else float("nan")

    dseg = [(t, z_) for lab, t, _fsv, z_ in TRUTH if lab == "degraded" and math.isfinite(z_)]
    if len(dseg) >= 2:
        print(f"  gz irtifa (bozulma)         : {dseg[0][1]:.2f} -> {dseg[-1][1]:.2f} m "
              f"({dseg[-1][0] - dseg[0][0]:.0f} s) -- dusus BEKLENEN")

    checks = [
        # Enjeksiyonun TUTTUGUNU dogrular: duruş yayini bitmis ama log devam
        # ediyor olmali. Yalnizca "son ornek var mi" diye bakmak her kosuda
        # gecerdi -- olcut, akisin OTEKI topicler akarken kesilmesi.
        ("duruş yayini gercekten kesildi (log devam ederken)",
         math.isfinite(t_att_last) and (float(tM[-1]) - t_att_last) > 1.0,
         f"son duruş t={t_att_last:.2f} s, log sonu t={float(tM[-1]):.2f} s"),
        ("RATE_ONLY (fs=3) HIC bildirilmedi", not (fs == 3).any(),
         "eski binary?" if (fs == 3).any() else "yok"),
        ("cikis kesildi", math.isfinite(t_cut), f"t={t_cut:.2f} s"),
        ("kesme ZAMANINDA (duruş kaybindan <0.5 s sonra)",
         math.isfinite(t_cut) and (t_cut - t_att_last) < 0.5,
         f"gecikme {t_cut - t_att_last:.3f} s"),
        # Tasarim savi: karar MODULUN. Commander de ayni sinyalle ~50 ms sonra
        # sonlandiriyor, ama modul onu beklemiyor.
        ("kesme commander'in lockdown'ini beklemedi",
         math.isfinite(t_cut) and t_cut <= t_lock + 0.05,
         f"cut {t_cut:.2f} s vs lockdown {t_lock:.2f} s"),
        ("kesme TAM (kesintiden sonra %100 NaN)", post_nan == 100.0, f"NaN %{post_nan:.1f}"),
        ("kesmeden ONCE havada NaN yok (adim 32 tasma regresyonu)",
         pre_nan == 0.0, f"NaN %{pre_nan:.1f}"),
    ]

    print("\n  --- olcutler ---")
    ok = True
    for label, passed, detail in checks:
        print(f"  [{'OK ' if passed else 'HAYIR'}] {label}  ({detail})")
        ok &= bool(passed)

    print(f"\n  SONUC: {'GECTI' if ok else 'KALDI'}")
    print(f"  ulog: {f}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
