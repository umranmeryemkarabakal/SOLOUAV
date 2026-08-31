#!/usr/bin/env python3
"""sitl-lockup-check kriter kosusu, tam enstrumanli.

Skill'in prosedurunu betiklestirir, iki duzeltmeyle:
  - yaw_sp = ARM ANINDA OLCULEN heading (Adim 27, olcum tuzagi #4: airframe
    gercek yaw +90 deg'de spawn oluyor; yaw_sp = 0 vermek testi 90 derecelik
    bir donus manevrasina cevirir)
  - pos_hold ACIK (Adim 28): yoksa arac ~7 m/s surüklenir ve "hover" testi
    aslinda seyir testi olur.
`tiltrotor_indi_status` log profili kurulur, yoksa sat_flag/u_actual ulog'a
hic yazilmaz ve kilitlenme olcutu olculemez.

Kullanim:
    python3 run_lockup_check.py
"""

from __future__ import annotations

import math
import os
import shutil
import sys
import time

import numpy as np
from pyulog import ULog

import indi_sitl_common as sc

# RUZGARLI KOSUM (2026-08-31, adim 135): INDI_WORLD=windy_tiltrotor
# Varsayilan "default" -- bu depodaki butun esikler SIFIR RUZGARDA kalibre
# edildi, dolayisiyla ruzgar OPT-IN olmali, varsayilan degil.
WORLD = os.environ.get("INDI_WORLD", "default")

CLIMB_M = 6.0
OBSERVE_S = 30.0   # skill: en az 25 s

LOG_TOPICS_SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logger_topics_shadow.txt")
LOG_TOPICS_DST_DIR = "/home/umran/PX4-Autopilot/build/px4_sitl_default/rootfs/etc/logging"
LOG_TOPICS_DST = os.path.join(LOG_TOPICS_DST_DIR, "logger_topics.txt")
ULOG_DIR = "/home/umran/PX4-Autopilot/build/px4_sitl_default/rootfs/log"


def newest_ulog() -> str:
    best, best_m = "", 0.0
    for root, _d, files in os.walk(ULOG_DIR):
        for f in files:
            if f.endswith(".ulg"):
                p = os.path.join(root, f)
                if os.path.getmtime(p) > best_m:
                    best, best_m = p, os.path.getmtime(p)
    return best


def main() -> int:
    px4 = sc.Px4Client()
    out_dir = os.path.dirname(os.path.abspath(__file__))
    os.makedirs(LOG_TOPICS_DST_DIR, exist_ok=True)
    shutil.copyfile(LOG_TOPICS_SRC, LOG_TOPICS_DST)

    print("=== SITL baslatiliyor ===")
    sc.launch_sitl("gz_tiltrotor_indi", os.path.join(out_dir, "px4_lockup_check.log"), world=WORLD)

    try:
        if not sc.wait_until(px4.preflight_check_ok, timeout=20.0, poll_interval=0.5):
            print("UYARI: preflight temizlenmedi")

        px4.arm(force=True)
        time.sleep(0.5)
        z_sp = px4.local_position().get("z", 0.0) - CLIMB_M

        yaw0_deg = float("nan")
        for _ in range(20):
            _, _, yaw0_deg = px4.attitude_euler_deg()
            if math.isfinite(yaw0_deg):
                break
            time.sleep(0.5)
        if not math.isfinite(yaw0_deg):
            raise RuntimeError("heading okunamadi")
        print(f"arm heading = {yaw0_deg:.2f} deg -> yaw_sp")

        t0 = time.monotonic()
        while time.monotonic() - t0 < OBSERVE_S + 6.0:
            px4.set_setpoint(roll=0.0, pitch=0.0, yaw=math.radians(yaw0_deg), fx=0.0,
                             z_sp=z_sp, leso_enable=(True, True, False), pos_hold=True)
            time.sleep(1.0)

        # Kademeli inis (1.0 m -- 1.5 m kademe BIG_M uretiyor, Adim 27), sonra
        # YERDE disarm. Irtifada disarm etmek araci dusurup takla attirir ve
        # EKF'i iraksatir (2026-07-29'da bir kez yapildi, kosuyu gecersiz kildi).
        # CIKIS SARTI IKI TERIMLI (2026-08-02). Eskiden yalnizca `z < -0.4` idi ve
        # bu bir DENGE SINIRININ ustune oturuyordu: arac yerde otururken EKF
        # z ~ -0.93 m okuyor, yani sart hicbir zaman saglanmiyor, dongu sonsuza
        # doner ve arac YERDE armed halde aktif kontrol altinda birakilir.
        # Olculdu (13_11_30.ulg): o fazda itkilerin %50.4'u 0 / 45 N rayinda,
        # arac t=62 s'den itibaren yerde kayiyor ve t=88 s'de roll +-180'e
        # devrilip sirtustu kaliyor. Kot esigine "inis durdu" tespiti ve bir
        # tavan sayaci eklendi. Kriter penceresi bundan ETKILENMEZ; degisen tek
        # sey inisin sonlanmasinin garanti altina alinmasi.
        z = px4.local_position().get("z", z_sp)
        stalled = 0
        for _ in range(40):
            if z >= -0.4:
                break
            z_cmd = min(-0.3, z + 1.0)
            px4.set_setpoint(roll=0.0, pitch=0.0, yaw=math.radians(yaw0_deg), fx=0.0,
                             z_sp=z_cmd, leso_enable=(True, True, False), pos_hold=True)
            time.sleep(1.5)
            z_new = px4.local_position().get("z", z)
            stalled = stalled + 1 if abs(z_new - z) < 0.05 else 0
            z = z_new
            if stalled >= 3:
                print(f"inis durdu (z = {z:.2f} m, 3 kademe boyunca degisim yok) -> yerde kabul")
                break
        time.sleep(3.0)
        print(f"inis tamam, z = {px4.local_position().get('z', float('nan')):.2f} m -> disarm")
        px4.disarm(force=True)
        time.sleep(2.0)

    finally:
        sc.kill_sitl()
        try:
            os.remove(LOG_TOPICS_DST)
        except FileNotFoundError:
            pass

    # --- olcut ---
    f = newest_ulog()
    u = ULog(f, ["tiltrotor_indi_status", "vehicle_angular_velocity", "vehicle_local_position"])
    d = {x.name: x.data for x in u.data_list}
    st = d["tiltrotor_indi_status"]
    ts = np.asarray(st["timestamp"], float)
    keep = np.concatenate(([True], np.diff(ts) > 1e-6))   # olcum tuzagi #3
    ts = ts[keep]
    t0u = ts[0]
    sat = np.column_stack([np.asarray(st["sat_flag[%d]" % i], float)[keep] for i in range(6)])
    ua = np.column_stack([np.asarray(st["u_actual[%d]" % i], float)[keep] for i in range(6)])
    av = d["vehicle_angular_velocity"]
    ta = (np.asarray(av["timestamp"], float) - t0u) / 1e6
    r = np.asarray(av["xyz[2]"], float)
    lp = d["vehicle_local_position"]
    tl = (np.asarray(lp["timestamp"], float) - t0u) / 1e6
    z = np.asarray(lp["z"], float)
    vz = np.asarray(lp["vz"], float)
    vh = np.hypot(np.asarray(lp["vx"], float), np.asarray(lp["vy"], float))

    i0 = int(np.argmax(z < -(CLIMB_M - 0.5)))
    a = tl[i0] + 3.0
    b = a + OBSERVE_S
    k = (ta >= a) & (ta < b)
    m = (tl >= a) & (tl < b)
    kk = ((ts - t0u) / 1e6 >= a) & ((ts - t0u) / 1e6 < b)

    thrust_sat = 100 * sat[kk, :3].mean()
    big_m = int(sat[kk, :3].any(axis=1).sum())
    yaw_turn = math.degrees(np.trapezoid(r[k], ta[k]))
    vz_max = float(np.abs(vz[m]).max())
    alt_rms = float(np.sqrt(((z[m] - np.median(z[m])) ** 2).mean()))

    print(f"\n=== KRITER PENCERESI {a:.1f} - {b:.1f} s ({b - a:.0f} s) ===")
    print(f"  itki sat_flag        : {thrust_sat:.2f}%   BIG_M ornek: {big_m}")
    print(f"  itki araligi         : {ua[kk, :3].min():.2f} - {ua[kk, :3].max():.2f} N  (0 / 45 = kilitlenme)")
    print(f"  tilt araligi         : {math.degrees(ua[kk, 3:].min()):.2f} - {math.degrees(ua[kk, 3:].max()):.2f} deg")
    print(f"  yaw toplam donus     : {yaw_turn:+.2f} deg   (|r| RMS {np.sqrt((r[k]**2).mean()):.4f} rad/s)")
    print(f"  |vz| max             : {vz_max:.3f} m/s   (ALT_VZ_MAX = 2.0)")
    print(f"  irtifa hata RMS      : {alt_rms:.3f} m")
    print(f"  v_h ort              : {vh[m].mean():.2f} m/s  (pos_hold dogrulama)")

    ok = (thrust_sat == 0.0 and big_m == 0 and abs(yaw_turn) < 30.0 and vz_max < 2.0)
    print(f"\n  SONUC: {'GECTI' if ok else 'KALDI'}")
    print(f"  ulog: {f}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
