#!/usr/bin/env python3
"""Adim 31 / Faz 2 -- OTOMATIK geri gecis testi (madde B5).

Adim 31'in onceki kosulari mekanizmayi ELLE surmustu (tavan konsoldan,
pitch konsoldan). Bu test hicbir sey surmez: yalnizca `bt_enable` bayragini
kaldirir ve durum makinesinin (backTransition(), backtrans_loop.m) isi
bitirmesini bekler.

Beklenen dizi -- her esigin mekanizmasi ayri ayri olculdu (bkz. rapor Adim 31,
esikler Adim 38/39'da guncellendi):
  RETRACT   tavan tabana varir VE (v_h < 10 m/s VEYA tabanda 20 s beklendi):
            kanat tilt tavani 2 deg/s ile 9 deg'e iner   [madde (R)]
  BRAKE     v_fwd < 3 m/s : tavan 20 deg'e YUKSELTILIR (yaw aktuatoru geri) +
            durus trimi + hizla sonen frenleme marji     [madde (S): esik ve
            marj GOVDE ILERI hizina bakar, buyukluge degil]
  HANDOFF   pos_hold istenir ve (buyukluk kapisini gecince) devralir

GECME OLCUTLERI:
  1. v_h < 1.0 m/s'ye iner ve orada kalir
  2. pos_hold gercekten DEVREYE GIRER (px4 log'unda "pos_hold: holding")
  3. irtifa bandi <= 3 m (kacis tirmanisi yok -- Adim 29'un arizasi)
  4. yaw: manevra boyunca toplam donus <= 45 deg (tavan takili kalirsa
     981-2117 deg oluyordu -- Adim 31/Faz 2 on-olcumu)
  5. itki doyumu %0 (aktuator kilitlenmesi yok)
  6. fren penceresinde YENIDEN HIZLANMA yok (Adim 39, madde (S): eski kodda
     3.08 -> 12.8 m/s geri kacis olmustu ve olcut 1-5 bunu gormemisti)

Kullanim:
    python3 run_backtrans_test.py
"""

from __future__ import annotations

import math
import os
import shutil
import sys
import time

import indi_sitl_common as sc

CLIMB_M = 25.0
SETTLE_S = 12.0
FX_FINAL = 15.0
T_RAMP = 12.0
ACCEL_HOLD_S = 10.0
# BT_TIMEOUT_S: 120 IDI. 2026-07-30 (Adim 37) 200'e cikarildi, cunku 120 s bir
# KONTROL arizasi ile YAVASLIGI ayirt edemiyordu: bir kosuda RETRACT 111.4 s
# surdu (arac 9 deg tavanda ~8.0 m/s'ye asimptotik yaklasiyor, ki bu tam
# BT_RELEASE_V'nin kendisi), geriye BRAKE'e yalnizca 8.7 s kaldi ve test
# "KALDI" verdi -- oysa BRAKE o 8.7 s'de 8.00 -> 3.28 m/s yapiyordu, yani
# saglikliydi. Olcut suresi, olculen manevranin en yavas mesru halinden
# kisaysa test kendi penceresini olcer.
BT_TIMEOUT_S = 200.0   # s, geri gecise verilen sure
HOLD_CHECK_S = 20.0    # s, pos_hold devraldiktan sonra izleme

LOG_TOPICS_SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "logger_topics_shadow.txt")
LOG_TOPICS_DST_DIR = "/home/umran/PX4-Autopilot/build/px4_sitl_default/rootfs/etc/logging"
LOG_TOPICS_DST = os.path.join(LOG_TOPICS_DST_DIR, "logger_topics.txt")
ULOG_DIR = "/home/umran/PX4-Autopilot/build/px4_sitl_default/rootfs/log"


def install_log_profile():
    os.makedirs(LOG_TOPICS_DST_DIR, exist_ok=True)
    shutil.copyfile(LOG_TOPICS_SRC, LOG_TOPICS_DST)


def remove_log_profile():
    try:
        os.remove(LOG_TOPICS_DST)
    except FileNotFoundError:
        pass


def newest_ulog() -> str:
    best, best_m = "", 0.0
    for root, _d, files in os.walk(ULOG_DIR):
        for f in files:
            if f.endswith(".ulg"):
                p = os.path.join(root, f)
                m = os.path.getmtime(p)
                if m > best_m:
                    best, best_m = p, m
    return best


def main() -> int:
    px4 = sc.Px4Client()
    out_dir = os.path.dirname(os.path.abspath(__file__))
    log_path = os.path.join(out_dir, "px4_backtrans_test.log")

    install_log_profile()
    print("=== SITL baslatiliyor ===")
    sc.launch_sitl("gz_tiltrotor_indi", log_path)

    yaw_ref = [float("nan")]
    bt_t0 = None

    def snap():
        lp = px4.local_position()
        st = px4.status()
        _, pitch_now, yaw_now = px4.attitude_euler_deg()
        u = st.get("u_actual", [float("nan")] * 6)
        return {
            "vh": math.hypot(lp.get("vx", float("nan")), lp.get("vy", float("nan"))),
            "z": lp.get("z", float("nan")),
            "vz": lp.get("vz", float("nan")),
            "pitch": pitch_now,
            "yaw": yaw_now,
            "tilt": [math.degrees(u[3 + i]) if len(u) >= 6 else float("nan") for i in range(3)],
        }

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
        if not math.isfinite(yaw0_deg):
            raise RuntimeError("heading okunamadi -- kosu iptal")
        yaw0 = math.radians(yaw0_deg)
        yaw_ref[0] = yaw0_deg
        print(f"arm heading = {yaw0_deg:.2f} deg")

        def send_sp(fx=0.0, pos_hold=False, bt=False):
            args = ["test_sp", "0.0", "0.0", f"{yaw0}", f"{fx}", f"{z_sp}",
                    "1", "1", "0", "1" if pos_hold else "0", "1" if bt else "0"]
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

        s = snap()
        print(f"   GIRIS: v_h={s['vh']:.2f} m/s  tilt={s['tilt'][0]:.1f}/{s['tilt'][1]:.1f}  z={s['z']:.2f}")

        print(f"3) bt_enable = 1 -- durum makinesi devrede, elle hicbir sey surulmuyor")
        bt_t0 = time.monotonic()
        z_min = z_max = s["z"]
        last = -99.0
        reached = False
        while time.monotonic() - bt_t0 < BT_TIMEOUT_S:
            t = time.monotonic() - bt_t0
            send_sp(bt=True)
            s = snap()
            if math.isfinite(s["z"]):
                z_min = min(z_min, s["z"]); z_max = max(z_max, s["z"])
            if t - last >= 5.0:
                print(f"   t={t:5.1f}  v_h={s['vh']:6.2f}  vz={s['vz']:+5.2f}  z={s['z']:7.2f}  "
                      f"pitch={s['pitch']:+5.1f}  tilt={s['tilt'][0]:5.1f}/{s['tilt'][1]:4.1f}/{s['tilt'][2]:4.1f}")
                last = t
            if s["vh"] < 1.0:
                reached = True
                print(f"   -> v_h < 1.0 m/s, t = {t:.1f} s")
                break
            time.sleep(0.5)

        if reached:
            print(f"4) pos_hold devraldi mi? {HOLD_CHECK_S:.0f} s izleniyor")
            t0 = time.monotonic()
            while time.monotonic() - t0 < HOLD_CHECK_S:
                send_sp(bt=True)
                s = snap()
                if math.isfinite(s["z"]):
                    z_min = min(z_min, s["z"]); z_max = max(z_max, s["z"])
                time.sleep(1.0)
            s = snap()
            print(f"   son: v_h={s['vh']:.2f} m/s  z={s['z']:.2f}  pitch={s['pitch']:+.1f}")

        print(f"\nirtifa bandi (manevra boyunca): {z_max - z_min:.2f} m")

        # --- kademeli inis, sonra YERDE disarm (2026-08-03, Adim 39) ---
        # Eskiden burada dogrudan `disarm` vardi ve arac 25 m'den DUSUYORDU.
        # Olcutler bundan etkilenmiyordu (pencere bt_state'ten kuruluyor ve
        # burada kapaniyor), ama iki sebeple degistirildi: (1) GUI'li kosuda
        # izlenen son sey aracin dusup takla atmasi oluyordu, yani gozlemin
        # son evresi bilgi tasimiyordu; (2) havada disarm bu projede zaten
        # belgelenmis bir tuzak (2026-07-29: arac dusuyor, ters kaliyor ve
        # sonraki arm "ters ucuyor" gibi okunuyor).
        # bt ONCE birakilir: bt_enable acik kalirsa olcut penceresi inise
        # kadar uzar ve irtifa bandi olcutu (<= 3 m) 25 m'lik inis yuzunden
        # HAKSIZ yere kalir. Inis pos_hold ile yapilir.
        # Desen run_lockup_check.py'den birebir alindi -- CIKIS SARTI IKI
        # TERIMLI olmali: arac yerdeyken EKF z ~ -0.93 m okuyor, yani yalnizca
        # `z >= -0.4` beklemek sonsuz donguye girer ve araci yerde armed
        # birakir (2026-08-02'de olculdu).
        print("5) kademeli inis (1.0 m kademe), sonra yerde disarm")
        z_now = px4.local_position().get("z", z_sp)
        stalled = 0
        for _ in range(40):
            if z_now >= -0.4:
                break
            z_cmd = min(-0.3, z_now + 1.0)
            args = ["test_sp", "0.0", "0.0", f"{yaw0}", "0.0", f"{z_cmd}",
                    "1", "1", "0", "1", "0"]
            px4._run("mc_indi_tiltrotor", args)
            time.sleep(1.5)
            z_new = px4.local_position().get("z", z_now)
            stalled = stalled + 1 if abs(z_new - z_now) < 0.05 else 0
            z_now = z_new
            if stalled >= 3:
                print(f"   inis durdu (z = {z_now:.2f} m) -> yerde kabul")
                break
        time.sleep(3.0)
        print(f"   inis tamam, z = {px4.local_position().get('z', float('nan')):.2f} m -> disarm")
        px4.disarm(force=True)
        time.sleep(2.0)

    finally:
        sc.kill_sitl()
        remove_log_profile()

    # --- olcutler ulog'dan, konsol gozleminden DEGIL (2026-07-30, Adim 37) ---
    # Yukaridaki canli izleme yalnizca uc olcutu gorebiliyordu (v_h, irtifa
    # bandi, pos_hold); yaw toplam donusu ve itki doyumu elle bakiliyordu, yani
    # docstring'de yazan bes olcut hicbir zaman otomatik dogrulanmiyordu.
    ulog = newest_ulog()
    print(f"\nulog: {ulog}")
    print(f"px4 log: {log_path}\n")

    import analyze_backtrans
    import check_output_cuts
    ok = analyze_backtrans.analyze(ulog)
    print()
    check_output_cuts.check(ulog)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
