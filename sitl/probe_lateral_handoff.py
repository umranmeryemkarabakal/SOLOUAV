#!/usr/bin/env python3
"""Madde (S) rejimini BILEREK kurar: BRAKE sirasinda yanal surukleme.

Adim 39 (2026-08-03).

NEDEN AYRI BIR PROBE GEREKTI
----------------------------
Madde (S) 2026-07-31'de kancali bir ucusta (BT_RELEASE_V = 5.0f) gorulmustu:
BRAKE penceresinde min|v_h| = 3.08 m/s iken govde ileri hizi u = -0.51 ve yanal
hiz v = +3.04 m/s idi -- yani buyukluk esigini tutan sey fren yasasinin
KALDIRAMADIGI eksendi, handoff hic istenmedi ve arac geri yonde 12.8 m/s'ye
kacti. Duzeltmeden sonra ayni kanca TEKRARLANDI ve manevra temiz tamamlandi,
ama yanal surukleme yalnizca 1.28 m/s'ye ulasti: yani o ucus rejime HIC
GIRMEDI, dolayisiyla duzeltmeyi SINAMADI.

Adim 21d'nin kurali: bir ortam ancak hedeflenen mekanizma orada AKTIFSE bir sey
kanitlar. Yanal surukleme SITL'de rastlantisal (adim 38'de yalnizca durum
makinesi 'attitude LOST' ile sifirlanip manevra bes kez bastan basladigi icin
birikmisti), o yuzden burada DETERMINISTIK olarak uretilir.

NASIL URETILIYOR -- ve neden bu meşru bir senaryo
-------------------------------------------------
Arac frenlerken heading'i cevirirsek, mevcut hiz vektoru govde cercevesinde
ILERI'den YANAL'a doner: |v_h| degismez, v_fwd duser. Olculen arizada da
gorulen budur (u = -0.51 iken v = +3.04, yani ~90 deg donmus bir arac).
Yaw setpoint'i BRAKE'e girilince arm heading + 90 deg yapilir; baska hicbir sey
degistirilmez, kanca yok, normal build (BT_RELEASE_V = 10.0f).

NE OLCULUR
----------
  1. HANDOFF gercekten oldu mu, ve OLDUGU ANDA |v_h| esigin USTUNDE miydi?
     Ustundeyse ESKI mantik o anda gecemezdi -- karsi-olgusal, gercek ucus
     verisinden. (MATLAB testi 8 ayni seyi sentetik izde gosteriyor.)
  2. BRAKE + (pos_hold devralmamis) HANDOFF penceresinde govde ileri hizi
     GERI yone kacti mi (madde (S)'nin asil zarari).
  3. Kalinti: handoff istegi ile pos_hold'un gercekten devralmasi arasindaki
     sure -- POS_ENGAGE_V_MAX hala bir BUYUKLUK kapisi oldugu icin sifir
     olmayabilir. Bu, dokumantasyondaki "olculecek" maddesinin ta kendisi.

Kullanim:
    export PATH=~/PX4-Autopilot/build/px4_sitl_default/bin:$PATH
    python3 probe_lateral_handoff.py
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

CLIMB_M = 25.0
SETTLE_S = 12.0
FX_FINAL = 15.0
T_RAMP = 12.0
ACCEL_HOLD_S = 10.0
BT_TIMEOUT_S = 200.0
HOLD_CHECK_S = 20.0

YAW_KICK_DEG = 90.0    # heading'i bu kadar cevir -> ileri hiz yanala doner
KICK_AT_VH = 8.0       # m/s, bu hizin altina inince cevirmeye basla

BT_HANDOFF_V = 3.0     # TiltrotorIndiParams.hpp ile ayni olmali
RUNAWAY_MAX = 1.0      # m/s, analyze_backtrans.py ile ayni olcut

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


def fly() -> None:
    px4 = sc.Px4Client()
    out_dir = os.path.dirname(os.path.abspath(__file__))
    log_path = os.path.join(out_dir, "px4_lateral_handoff.log")

    os.makedirs(LOG_TOPICS_DST_DIR, exist_ok=True)
    shutil.copyfile(LOG_TOPICS_SRC, LOG_TOPICS_DST)

    print("=== SITL baslatiliyor ===")
    sc.launch_sitl("gz_tiltrotor_indi", log_path)

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
            raise RuntimeError("heading okunamadi -- kosu iptal")
        yaw0 = math.radians(yaw0_deg)
        yaw_kick = yaw0 + math.radians(YAW_KICK_DEG)
        print(f"arm heading = {yaw0_deg:.2f} deg,  kick hedefi = {yaw0_deg + YAW_KICK_DEG:.2f} deg")

        def send_sp(fx=0.0, pos_hold=False, bt=False, yaw=None):
            args = ["test_sp", "0.0", "0.0", f"{yaw0 if yaw is None else yaw}",
                    f"{fx}", f"{z_sp}", "1", "1", "0",
                    "1" if pos_hold else "0", "1" if bt else "0"]
            return px4._run("mc_indi_tiltrotor", args)

        print(f"1) hover + {CLIMB_M} m")
        t0 = time.monotonic()
        while time.monotonic() - t0 < SETTLE_S:
            send_sp(pos_hold=True)
            time.sleep(1.0)

        print("2) fx rampasi -> seyir")
        t0 = time.monotonic()
        while time.monotonic() - t0 < T_RAMP + ACCEL_HOLD_S:
            t = time.monotonic() - t0
            send_sp(fx=FX_FINAL * min(1.0, t / T_RAMP))
            time.sleep(0.5)

        print("3) bt_enable = 1; v_h < %.1f olunca heading +%.0f deg cevrilir"
              % (KICK_AT_VH, YAW_KICK_DEG))
        bt_t0 = time.monotonic()
        kicked = False
        last = -99.0
        while time.monotonic() - bt_t0 < BT_TIMEOUT_S:
            t = time.monotonic() - bt_t0
            lp = px4.local_position()
            vh = math.hypot(lp.get("vx", float("nan")), lp.get("vy", float("nan")))
            if (not kicked) and math.isfinite(vh) and vh < KICK_AT_VH:
                kicked = True
                print(f"   -> YAW KICK, t = {t:.1f} s, v_h = {vh:.2f} m/s")
            send_sp(bt=True, yaw=(yaw_kick if kicked else yaw0))
            if t - last >= 5.0:
                _, pitch_now, yaw_now = px4.attitude_euler_deg()
                print(f"   t={t:5.1f}  v_h={vh:6.2f}  yaw={yaw_now:+7.1f}  pitch={pitch_now:+5.1f}")
                last = t
            if math.isfinite(vh) and vh < 1.0:
                print(f"   -> v_h < 1.0 m/s, t = {t:.1f} s")
                break
            time.sleep(0.5)

        t0 = time.monotonic()
        while time.monotonic() - t0 < HOLD_CHECK_S:
            send_sp(bt=True, yaw=yaw_kick if kicked else yaw0)
            time.sleep(1.0)

        # Kademeli inis, sonra YERDE disarm -- havada disarm bu projede
        # belgelenmis bir tuzak (arac dusup ters kaliyor) ve GUI'li kosuda
        # gozlemin son evresini bilgisiz birakiyor. bt birakilir, pos_hold ile
        # inilir; cikis sarti iki terimli (yerde EKF z ~ -0.93 okuyor).
        print("4) kademeli inis, sonra yerde disarm")
        z_now = px4.local_position().get("z", z_sp)
        stalled = 0
        for _ in range(40):
            if z_now >= -0.4:
                break
            z_cmd = min(-0.3, z_now + 1.0)
            px4._run("mc_indi_tiltrotor",
                     ["test_sp", "0.0", "0.0", f"{yaw_kick if kicked else yaw0}",
                      "0.0", f"{z_cmd}", "1", "1", "0", "1", "0"])
            time.sleep(1.5)
            z_new = px4.local_position().get("z", z_now)
            stalled = stalled + 1 if abs(z_new - z_now) < 0.05 else 0
            z_now = z_new
            if stalled >= 3:
                print(f"   inis durdu (z = {z_now:.2f} m) -> yerde kabul")
                break
        time.sleep(3.0)
        px4.disarm(force=True)
        time.sleep(2.0)

    finally:
        sc.kill_sitl()
        try:
            os.remove(LOG_TOPICS_DST)
        except FileNotFoundError:
            pass


def analyze(path: str) -> bool:
    u = ULog(path, ["tiltrotor_indi_status", "vehicle_local_position", "vehicle_attitude"])
    d = {m.name: m.data for m in u.data_list}
    st = d["tiltrotor_indi_status"]
    ts = np.asarray(st["timestamp"], float)
    keep = np.concatenate(([True], np.diff(ts) > 1e-6))
    t_st = ts[keep] / 1e6
    bt = np.asarray(st["bt_state"], float)[keep]
    pha = (np.asarray(st["pos_hold_active"], float)[keep] > 0.5
           if "pos_hold_active" in st else np.zeros_like(bt, dtype=bool))

    lp = d["vehicle_local_position"]
    t = np.asarray(lp["timestamp"], float) / 1e6
    vx = np.asarray(lp["vx"], float)
    vy = np.asarray(lp["vy"], float)
    vh = np.hypot(vx, vy)

    at = d["vehicle_attitude"]
    ta = np.asarray(at["timestamp"], float) / 1e6
    q = np.column_stack([np.asarray(at["q[%d]" % i], float) for i in range(4)])
    yaw = np.arctan2(2 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]),
                     1 - 2 * (q[:, 2] ** 2 + q[:, 3] ** 2))
    psi = np.interp(t, ta, np.unwrap(yaw))
    v_fwd = vx * np.cos(psi) + vy * np.sin(psi)
    v_lat = -vx * np.sin(psi) + vy * np.cos(psi)

    print(f"\n=== MADDE (S) PROBE ANALIZI ===\n  ulog: {path}")

    ho = np.nonzero(bt > 2.5)[0]
    if ho.size == 0:
        print("  HANDOFF'a HIC GIRILMEDI -- madde (S) ariza imzasi (esik saglanmadi)")
        return False

    t_ho = t_st[ho[0]]
    vh_ho = float(np.interp(t_ho, t, vh))
    vf_ho = float(np.interp(t_ho, t, v_fwd))
    vl_ho = float(np.interp(t_ho, t, v_lat))

    print(f"\n  1) HANDOFF t = {t_ho:.1f} s'de gerceklesti")
    print(f"     o anda: |v_h| = {vh_ho:.2f} m/s   v_fwd = {vf_ho:+.2f}   v_yanal = {vl_ho:+.2f}")
    old_would = vh_ho < BT_HANDOFF_V
    print(f"     ESKI (buyukluk) kosulu o anda saglanir miydi? "
          f"{'EVET -- rejim kurulamadi' if old_would else 'HAYIR -- yeni sinyal fark yaratti'}")

    # BRAKE + pos_hold devralmamis HANDOFF: fren yasasinin pitch'e sahip oldugu pencere
    own = (bt >= 1.5) & (~pha)
    if own.any():
        a, b = t_st[own][0], t_st[own][-1]
        m = (t >= a) & (t <= b)
        # OLCUT YENIDEN-HIZLANMADIR, isaretli ileri hiz DEGIL. Bu probe'un
        # kendisi tam olarak bu tuzagi ortaya cikardi (Adim 39): heading burada
        # ~187 deg donuyor, v_fwd DONEN cerceveye izdusum oldugu icin arac
        # yavaslarken bile isaret degistiriyor. Kacisin frame'den bagimsiz
        # tanimi: v_h kendi kosan minimumunun uzerine cikti mi?
        vh_w = vh[m]
        reaccel = float((vh_w - np.minimum.accumulate(vh_w)).max())
        print(f"\n  2) fren yasasinin pitch'e sahip oldugu pencere: {a:.1f} .. {b:.1f} s")
        print(f"     yeniden-hizlanma = {reaccel:.2f} m/s  (<= {RUNAWAY_MAX} olmali) "
              f"-> {'GECTI' if reaccel <= RUNAWAY_MAX else 'KALDI -- GERI KACIS'}")
        print(f"     v_h {vh_w[0]:.2f} -> {vh_w[-1]:.2f} m/s, heading degisimi "
              f"{math.degrees(psi[m][-1] - psi[m][0]):+.1f} deg")
        print(f"     teshis (olcut DEGIL): min v_fwd = {np.nanmin(v_fwd[m]):+.2f} m/s, "
              f"max |v_yanal| = {np.nanmax(np.abs(v_lat[m])):.2f} m/s  "
              f"(rejim kuruldu mu: yanal >= 2.0 anlamli)")

    # Kalinti: handoff istegi -> pos_hold devri
    eng = np.nonzero(pha)[0]
    eng = eng[eng >= ho[0]]
    if eng.size:
        print(f"\n  3) kalinti: HANDOFF -> pos_hold devri {t_st[eng[0]] - t_ho:.2f} s bekledi")
        print(f"     (o sirada |v_h| {float(np.interp(t_ho, t, vh)):.2f} -> "
              f"{float(np.interp(t_st[eng[0]], t, vh)):.2f} m/s; kapi POS_ENGAGE_V_MAX)")
    else:
        print("\n  3) kalinti: pos_hold HIC devralmadi -- kapinin engeli kalici")

    # Yerlesme hizi MANEVRA penceresinin sonundan okunur, log'un sonundan DEGIL:
    # kosu artik kademeli inisle bitiyor (Adim 39) ve log'un son 10 s'i inisi
    # olcer, yerlesmeyi degil. Ilk surumde bu 0.15 yerine 2.01 m/s yaziyordu --
    # olcum penceresini olctugu seyden bagimsiz kurma kuralinin kucuk hali.
    t_bt_end = t_st[bt > 0.5][-1] if (bt > 0.5).any() else t[-1]
    tail = (t >= t_bt_end - 10.0) & (t <= t_bt_end)
    print(f"\n  manevra sonu son 10 s ort |v_h| = {vh[tail].mean():.2f} m/s")
    return True


def main() -> int:
    if "--analyze" not in sys.argv:
        fly()
    path = newest_ulog()
    if not path:
        print("HATA: ulog bulunamadi")
        return 2
    return 0 if analyze(path) else 1


if __name__ == "__main__":
    sys.exit(main())
