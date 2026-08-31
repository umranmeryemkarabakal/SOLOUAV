#!/usr/bin/env python3
"""Seyir zarfi supurmesi: fx -> hiz / tilt / KANAT TASIMA PAYI. (Adim 43)

NEDEN
-----
2026-08-03'te olculdu: `FT_FX_CRUISE = 12 N` ile arac 14.4 m/s'de seyrediyor ama
naseller yalnizca 32 deg'de ve **kanat agirligin %41'ini tasiyor** -- yani bu bir
KISMI gecis, tam sabit kanat degil. Kuyruk rotoru da 0.6 deg'de, hala tasiyici.
Hedef tam kanat-tasimali ucus (v ~ 22 m/s, tasima ~ v^2'den kestirildi), ama
`FT_FX_CRUISE`'u TAHMINLE yukseltmek bu projede tam olarak yasak olan sey:
her sabit bir ORTAMIN olcumudur.

Bu probe fx'i kademeli supurur ve her kademede zarfin dort kenarini olcer:
  - ulasilan hiz ve kanat tilti
  - KANAT TASIMA PAYI = (agirlik - sum(T_i cos d_i)) / agirlik
  - itki marji (toplam itki / 3*ROTOR_TMAX) ve doygunluk
  - irtifa tutulabiliyor mu (kanat tasidikca irtifa dongusu "tasimayi azalt"
    demeye baslar -- adim 31/faz 0'in Fz kacisi mekanizmasi, ve burada
    GUCLENIR: kanat ne kadar cok tasirsa o talep o kadar buyur)

PROBE, KONTROL YASASI DEGIL: fx'i tezgah `test_sp` yolundan surer (ft_enable=0),
cunku olculecek sey yasanin kendisi degil ARACIN ZARFI. Yasaya (FT_FX_CRUISE)
ne yazilacagina bu olcumden sonra karar verilir.

GUVENLIK: pitch = 0 (adim 29), ve |dz| > ALT_ABORT olursa supurme durur.

Kullanim:
    export PATH=~/PX4-Autopilot/build/px4_sitl_default/bin:$PATH
    INDI_SITL_GUI=1 INDI_GZ_CAM=side python3 probe_cruise_envelope.py
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

CLIMB_M = 60.0          # m, yuksek irtifa: supurme irtifa payi ister
SETTLE_S = 12.0
FX_STEPS = [12.0, 15.0, 18.0, 21.0, 24.0]
STEP_S = 18.0           # her kademede yerlesme + olcum
RAMP_S = 6.0            # kademeler arasi rampa
ALT_ABORT = 15.0        # m, giris irtifasindan sapma -> supurmeyi durdur

MASS = 5.0
GRAVITY = 9.81
ROTOR_TMAX = 45.0

LOG_TOPICS_SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "logger_topics_shadow.txt")
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
    log_path = os.path.join(out_dir, "px4_cruise_envelope.log")

    os.makedirs(LOG_TOPICS_DST_DIR, exist_ok=True)
    shutil.copyfile(LOG_TOPICS_SRC, LOG_TOPICS_DST)

    print("=== SITL baslatiliyor ===")
    sc.launch_sitl("gz_tiltrotor_indi", log_path)

    marks = []          # (fx, t_start_us, t_end_us)

    try:
        if not sc.wait_until(px4.preflight_check_ok, timeout=20.0, poll_interval=0.5):
            print("UYARI: preflight temizlenmedi")

        px4.arm(force=True)
        time.sleep(0.5)
        z0 = px4.local_position().get("z", 0.0)
        z_sp = z0 - CLIMB_M

        yaw0_deg = float("nan")
        for _ in range(20):
            _, _, yaw0_deg = px4.attitude_euler_deg()
            if math.isfinite(yaw0_deg):
                break
            time.sleep(0.5)
        yaw0 = math.radians(yaw0_deg)
        print(f"arm heading = {yaw0_deg:.2f} deg")

        def send(fx=0.0, pos_hold=False):
            args = ["test_sp", "0.0", "0.0", f"{yaw0}", f"{fx}", f"{z_sp}",
                    "1", "1", "0", "1" if pos_hold else "0", "0", "0"]
            return px4._run("mc_indi_tiltrotor", args)

        print(f"1) kalkis + hover {CLIMB_M:.0f} m")
        t0 = time.monotonic()
        while time.monotonic() - t0 < SETTLE_S + CLIMB_M / 1.5:
            send(pos_hold=True)
            time.sleep(1.0)
        z_entry = px4.local_position().get("z", z_sp)
        print(f"   hover z = {z_entry:.1f} m")

        print("2) fx supurmesi (pos_hold BIRAKILDI)")
        print("   fx(N)   v_h    tilt0/1/2        z      not")
        fx_prev = 0.0
        for fx in FX_STEPS:
            # Kademeler arasi rampa: basamak vermek gecici bir tirmanma uretir.
            t0 = time.monotonic()
            while time.monotonic() - t0 < RAMP_S:
                a = (time.monotonic() - t0) / RAMP_S
                send(fx=fx_prev + (fx - fx_prev) * a)
                time.sleep(0.5)
            fx_prev = fx

            t_a = px4.local_position().get("timestamp", float("nan"))
            t0 = time.monotonic()
            aborted = False
            while time.monotonic() - t0 < STEP_S:
                send(fx=fx)
                z_now = px4.local_position().get("z", z_entry)
                if abs(z_now - z_entry) > ALT_ABORT:
                    aborted = True
                    break
                time.sleep(0.5)
            t_b = px4.local_position().get("timestamp", float("nan"))
            marks.append((fx, t_a, t_b))

            lp = px4.local_position()
            stt = px4.status()
            u = stt.get("u_actual", [float("nan")] * 6)
            vh = math.hypot(lp.get("vx", float("nan")), lp.get("vy", float("nan")))
            tilts = [math.degrees(u[3 + i]) for i in range(3)] if len(u) >= 6 else [float("nan")] * 3
            print(f"   {fx:5.1f}  {vh:6.2f}  {tilts[0]:5.1f}/{tilts[1]:4.1f}/{tilts[2]:4.1f}  "
                  f"{lp.get('z', float('nan')):7.2f}  {'IRTIFA SAPTI -- DURDURULDU' if aborted else ''}")
            if aborted:
                break

        print("3) geri gecis ile hover'a don")
        t0 = time.monotonic()
        while time.monotonic() - t0 < 200.0:
            px4._run("mc_indi_tiltrotor",
                     ["test_sp", "0.0", "0.0", f"{yaw0}", "0.0", f"{z_sp}",
                      "1", "1", "0", "0", "1", "0"])
            lp = px4.local_position()
            if math.hypot(lp.get("vx", 9.0), lp.get("vy", 9.0)) < 1.0:
                print("   -> hover")
                break
            time.sleep(1.0)

        z_now = px4.local_position().get("z", z_sp)
        stalled = 0
        for _ in range(80):
            if z_now >= -0.4:
                break
            px4._run("mc_indi_tiltrotor",
                     ["test_sp", "0.0", "0.0", f"{yaw0}", "0.0", f"{min(-0.3, z_now + 1.0)}",
                      "1", "1", "0", "1", "0", "0"])
            time.sleep(1.5)
            z_new = px4.local_position().get("z", z_now)
            stalled = stalled + 1 if abs(z_new - z_now) < 0.05 else 0
            z_now = z_new
            if stalled >= 3:
                break
        px4.disarm(force=True)
        time.sleep(2.0)

    finally:
        sc.kill_sitl()
        try:
            os.remove(LOG_TOPICS_DST)
        except FileNotFoundError:
            pass

    ulog = newest_ulog()
    print(f"\nulog: {ulog}\n")
    analyze(ulog, marks)
    return 0


def analyze(path: str, marks) -> None:
    u = ULog(path, ["tiltrotor_indi_status", "vehicle_local_position"])
    d = {m.name: m.data for m in u.data_list}
    st = d["tiltrotor_indi_status"]
    ts = np.asarray(st["timestamp"], float)
    keep = np.concatenate(([True], np.diff(ts) > 1e-6))
    t = ts[keep]
    ua = np.column_stack([np.asarray(st["u_actual[%d]" % i], float)[keep] for i in range(6)])
    sat = np.column_stack([np.asarray(st["sat_flag[%d]" % i], float)[keep] for i in range(6)])
    T = ua[:, 0:3]
    dl = ua[:, 3:6]
    Tz = (T * np.cos(dl)).sum(axis=1)

    lp = d["vehicle_local_position"]
    tl = np.asarray(lp["timestamp"], float)
    vh = np.hypot(np.asarray(lp["vx"], float), np.asarray(lp["vy"], float))
    z = np.asarray(lp["z"], float)

    W = MASS * GRAVITY
    print("=== SEYIR ZARFI ===")
    print("  fx(N)  v_h    tilt0  tilt2   itki   dikey  KANAT%  itki marji  doyum%  dz(m)")
    for fx, ta, tb in marks:
        if not (math.isfinite(ta) and math.isfinite(tb)):
            continue
        # Son 1/3'u al: kademenin YERLESMIS kismi (gecici rampayi disla)
        a = ta + 2.0 * (tb - ta) / 3.0
        m = (t >= a) & (t <= tb)
        ml = (tl >= a) & (tl <= tb)
        if not m.any() or not ml.any():
            continue
        wing = W - Tz[m].mean()
        tot = T[m].sum(axis=1).mean()
        print(f"  {fx:5.1f} {vh[ml].mean():6.2f}  {np.degrees(dl[m,0]).mean():5.1f}  "
              f"{np.degrees(dl[m,2]).mean():5.1f}  {tot:6.1f} {Tz[m].mean():6.1f}  "
              f"{100*wing/W:5.0f}   {100*tot/(3*ROTOR_TMAX):8.0f}  "
              f"{100*sat[m,:3].mean():6.2f}  {z[ml].max()-z[ml].min():5.2f}")

    print("\n  KANAT% = (agirlik - rotorlarin dikey bileseni) / agirlik")
    print("  %100 = tam kanat-tasimali ucus (hedef); %41 = 2026-08-03'te olculen hal")


if __name__ == "__main__":
    sys.exit(main())
