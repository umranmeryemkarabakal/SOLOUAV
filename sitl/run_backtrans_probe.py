#!/usr/bin/env python3
"""Adim 31 / Faz 0 -- SEBEP OLCUMU, kontrol yasasi denemesi DEGIL.

Soru: Adim 30'da geri gecis denemesi sirasinda tilt'ler neden ILERIYE kacti
(43 -> 80 deg)? Rapordaki aciklama "Fx cok zayif bir WLS amaci (Ws_Fx = 0.05)"
-- ama goz ardi edilen bir terim tilt'i surmez, sadece SURMEZ. Tilt'i aktif
olarak ileri iten baska bir talep olmali.

HIPOTEZ (H1, irtifa/Fz kanali): sf_wls_alloc.m'de dFz/ddelta = T*sin(delta),
dFz/dT = -cos(delta). delta buyudukce tilt dikey eksende itkiden kat kat
etkili olur (delta=80 deg'de cos = 0.17, yani itki neredeyse islevsiz). Kanat
araci tasirken irtifa dongusunun talebi "tasimayi AZALT" olur (nu_des[4] > 0,
FRD'de +z asagi) ve bunu yapmanin en ucuz yolu tilt'i ILERI almaktir. Kendi
kendini besleyen dongu: tilt ileri -> hiz artar -> kanat tasimasi artar ->
"azalt" talebi buyur -> tilt daha ileri.

RAKIP HIPOTEZ (H2, pitch momenti): burun-yukari talebi. Ama kanat rotorleri
icin dM_y/ddelta = T*(r_z*cos d - r_x*sin d) = T*(0.06*cos d - 0.22*sin d),
delta=43 deg'de -0.106*T -- yani H2 kanat tilt'lerinin GERIYE gitmesini
ongorur, kuyrugun ileri. Olculen ise kanat tilt'lerinin ileri gitmesi.
Iki hipotez ZIT ISARET ongordugu icin ayirt edilebilirler.

NEDEN YENIDEN UCUS GEREKIYOR: Adim 30'un log'u (07_50_14.ulg) duruyor ama
`tiltrotor_indi_status` VARSAYILAN log profilinde YOK, yani nu_des/du hic
kaydedilmemis. Bu betik o topic'i acar (logger_topics.txt, bkz. Adim 18) ve
rejimi decelLoop OLMADAN yeniden uretir.

REJIM decelLoop'suz nasil uretiliyor: Adim 30'un baslangic kosulu "hizli +
fx_sp = 0" idi; burun-yukari frenleme yasasi tilt kacisinin SEBEBI degildi
(rapor tablosu: pitch komutu ~0-4 deg'e kisilmisti ve tilt yine de kacti).
Yani sadece seyire cikip fx_sp'yi 0'a birakmak yeter -- yeni kod, rebuild ve
carpma riski olmadan.

Zaman cizelgesi:
  1. pos_hold ile gercek hover'da otur (SETTLE_S)
  2. pos_hold birak, fx 0 -> FX_FINAL rampa (T_RAMP), sonra tut (ACCEL_HOLD_S)
     -> ~15 m/s seyir, tilt ~40-50 deg (Adim 30'un giris kosulu)
  3. GOZLEM PENCERESI (OBSERVE_S): fx_sp = 0, roll_sp = pitch_sp = 0.
     Hicbir sey komut edilmiyor. Tilt ne yapiyor?

DIKKAT (olcum tuzagi #4, Adim 27): yaw_sp ASLA 0 verilmez -- airframe gercek
yaw +90 deg'de spawn oluyor. Arm sonrasi olculen heading tum kosu boyunca
yaw_sp olarak kullanilir, yoksa 90 derecelik bir donus manevrasi olcume
karisir.

Kullanim:
    python3 run_backtrans_probe.py
Sonra:
    python3 analyze_backtrans_probe.py <ulog>
"""

from __future__ import annotations

import math
import os
import shutil
import sys
import time

import indi_sitl_common as sc

CLIMB_M = 15.0        # m, Adim 29/30'da kacis tirmanisi gorulduu icin bol pay
SETTLE_S = 12.0       # s, pos_hold ile gercek hover
FX_FINAL = 15.0       # N, Adim 30'un ~15 m/s giris kosuluna ulasmak icin
T_RAMP = 12.0         # s
ACCEL_HOLD_S = 10.0   # s, hizin oturmasi icin
OBSERVE_S = 35.0      # s, fx_sp = 0 gozlem penceresi

LOG_TOPICS_SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "logger_topics_shadow.txt")
LOG_TOPICS_DST_DIR = "/home/umran/PX4-Autopilot/build/px4_sitl_default/rootfs/etc/logging"
LOG_TOPICS_DST = os.path.join(LOG_TOPICS_DST_DIR, "logger_topics.txt")
ULOG_DIR = "/home/umran/PX4-Autopilot/build/px4_sitl_default/rootfs/log"


def install_log_profile() -> None:
    """tiltrotor_indi_status'u tam hizda logla. DIKKAT: varsayilan profili
    TAMAMEN degistirir (logged_topics.cpp:560 if/else) -- kosu bitince silinir."""
    os.makedirs(LOG_TOPICS_DST_DIR, exist_ok=True)
    shutil.copyfile(LOG_TOPICS_SRC, LOG_TOPICS_DST)
    print(f"log profili kuruldu: {LOG_TOPICS_DST}")


def remove_log_profile() -> None:
    try:
        os.remove(LOG_TOPICS_DST)
        print(f"log profili SILINDI: {LOG_TOPICS_DST}")
    except FileNotFoundError:
        pass


def newest_ulog() -> str:
    best, best_mtime = "", 0.0
    for root, _dirs, files in os.walk(ULOG_DIR):
        for f in files:
            if f.endswith(".ulg"):
                p = os.path.join(root, f)
                m = os.path.getmtime(p)
                if m > best_mtime:
                    best, best_mtime = p, m
    return best


def main() -> int:
    px4 = sc.Px4Client()
    out_dir = os.path.dirname(os.path.abspath(__file__))
    log_path = os.path.join(out_dir, "px4_backtrans_probe.log")

    install_log_profile()

    print("=== SITL baslatiliyor ===")
    sc.launch_sitl("gz_tiltrotor_indi", log_path)

    marks = {}

    try:
        print("EKF hizalanmasi bekleniyor (commander preflight check)")
        if not sc.wait_until(px4.preflight_check_ok, timeout=20.0, poll_interval=0.5):
            print("UYARI: preflight 20 s icinde temizlenmedi -- yine de arm ediliyor")

        print("arm")
        px4.arm(force=True)
        time.sleep(0.5)

        z0 = px4.local_position().get("z", 0.0)
        z_sp = z0 - CLIMB_M

        # Olcum tuzagi #4: spawn heading +90 deg. Olculen heading'i kilitle.
        # Okuma basarisiz olursa DURDUR -- nan bir yaw_sp tum kontrolcuyu
        # zehirler (2026-07-29'da bir kosu boyle kaybedildi).
        yaw0_deg = float("nan")
        for _ in range(20):
            _, _, yaw0_deg = px4.attitude_euler_deg()
            if math.isfinite(yaw0_deg):
                break
            time.sleep(0.5)

        if not math.isfinite(yaw0_deg):
            raise RuntimeError("vehicle_attitude'dan heading okunamadi -- kosu iptal")

        yaw0 = math.radians(yaw0_deg)
        print(f"arm heading = {yaw0_deg:.2f} deg -> yaw_sp bu degere kilitlendi")

        def send_sp(fx=0.0, pos_hold=False):
            px4.set_setpoint(roll=0.0, pitch=0.0, yaw=yaw0, fx=fx, z_sp=z_sp,
                             leso_enable=(True, True, False), pos_hold=pos_hold)

        # --- 1) gercek hover (pos_hold) ---
        print(f"1) pos_hold ile hover + {CLIMB_M} m tirmanis ({SETTLE_S} s)")
        t0 = time.monotonic()
        while time.monotonic() - t0 < SETTLE_S:
            send_sp(pos_hold=True)
            time.sleep(1.0)

        lp = px4.local_position()
        print(f"   hover: v_h = {math.hypot(lp.get('vx', 0), lp.get('vy', 0)):.2f} m/s, "
              f"z = {lp.get('z', float('nan')):.2f} m")

        # --- 2) hizlanma ---
        print(f"2) pos_hold birakildi, fx 0 -> {FX_FINAL} N ({T_RAMP} s) + tut ({ACCEL_HOLD_S} s)")
        marks["accel_start"] = time.monotonic()
        t0 = time.monotonic()
        while True:
            t = time.monotonic() - t0
            if t >= T_RAMP + ACCEL_HOLD_S:
                break
            send_sp(fx=FX_FINAL * min(1.0, t / T_RAMP))
            time.sleep(0.5)

        lp = px4.local_position()
        st = px4.status()
        u = st.get("u_actual", [float("nan")] * 6)
        v_entry = math.hypot(lp.get("vx", 0.0), lp.get("vy", 0.0))
        tilt_entry = [math.degrees(u[3 + i]) if len(u) >= 6 else float("nan") for i in range(3)]
        print(f"   GIRIS KOSULU: v_h = {v_entry:.2f} m/s, "
              f"tilt = {tilt_entry[0]:.1f}/{tilt_entry[1]:.1f}/{tilt_entry[2]:.1f} deg, "
              f"z = {lp.get('z', float('nan')):.2f} m")

        # --- 3) gozlem penceresi: fx_sp = 0, duz attitude, baska hicbir sey ---
        print(f"3) GOZLEM: fx_sp = 0, roll/pitch = 0 ({OBSERVE_S} s) -- tilt ne yapiyor?")
        marks["observe_start"] = time.monotonic()
        t0 = time.monotonic()
        while True:
            t = time.monotonic() - t0
            if t >= OBSERVE_S:
                break
            send_sp(fx=0.0)

            if int(t) % 5 == 0:
                lp = px4.local_position()
                st = px4.status()
                u = st.get("u_actual", [float("nan")] * 6)
                nu = st.get("nu_des", [float("nan")] * 5)
                tl = [math.degrees(u[3 + i]) if len(u) >= 6 else float("nan") for i in range(3)]
                print(f"   t={t:5.1f}  v_h={math.hypot(lp.get('vx', 0), lp.get('vy', 0)):6.2f}  "
                      f"z={lp.get('z', float('nan')):7.2f}  "
                      f"tilt={tl[0]:5.1f}/{tl[1]:5.1f}/{tl[2]:5.1f}  "
                      f"nu_Fz={nu[4] if len(nu) == 5 else float('nan'):7.2f}  "
                      f"nu_Fx={nu[3] if len(nu) == 5 else float('nan'):7.2f}")
            time.sleep(1.0)

        marks["observe_end"] = time.monotonic()
        lp = px4.local_position()
        st = px4.status()
        u = st.get("u_actual", [float("nan")] * 6)
        tilt_exit = [math.degrees(u[3 + i]) if len(u) >= 6 else float("nan") for i in range(3)]
        print(f"   CIKIS: v_h = {math.hypot(lp.get('vx', 0), lp.get('vy', 0)):.2f} m/s, "
              f"tilt = {tilt_exit[0]:.1f}/{tilt_exit[1]:.1f}/{tilt_exit[2]:.1f} deg, "
              f"z = {lp.get('z', float('nan')):.2f} m")

        print("disarm (log kapansin diye)")
        px4.disarm(force=True)
        time.sleep(2.0)

    finally:
        sc.kill_sitl()
        remove_log_profile()

    ulog = newest_ulog()
    print(f"\nulog: {ulog}")
    print(f"Gozlem penceresi kosu basindan ~{marks.get('observe_start', 0) - marks.get('accel_start', 0):.0f} s sonra basladi")
    print(f"\nSonraki adim:\n  python3 analyze_backtrans_probe.py {ulog}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
