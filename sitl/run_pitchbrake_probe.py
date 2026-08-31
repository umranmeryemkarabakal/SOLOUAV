#!/usr/bin/env python3
"""Adim 31 / Faz 2 on-olcumu -- gövde pitch'i 7.5 m/s bandinda FRENLIYOR mu,
yoksa Adim 29'daki gibi TIRMANIS komutuna mi donuyor?

Neden bu soru. Faz 1 olctu: tilt tavani 15 -> ~7.5 m/s getiriyor (taban 9 deg,
yaw kabul edilebilir) ama daha asagi inemiyor, cunku delta1 TILT_MIN'de cakili
oldugundan tau_z = -0.25*Fx -- yaw trim torku ile kalan ileri kuvvet AYNI
buyukluk. Ote yandan Adim 29 gövde pitch'iyle frenlemenin ~5-6 m/s ustunde
tehlikeli oldugunu gosterdi (14.5 m/s'de pitch POS_TILT_MAX'te doydu, arac
35 s tirmandi). Yani 7.5 -> 5 m/s bandini iki mekanizma da tek basina
kapatmiyor. Bu kosu o bandi olcer.

Adim 29 ile farki ONEMLI: orada devreye alma 14.5 m/s'de, doymus pitch ile ve
tilt 43-80 deg kacarken oldu. Burada arac 7.5 m/s'de, DUZ, irtifasini tutan ve
tilt'i 9 deg'e kilitli bir durumdan giriyor. 7.5 m/s'de kanat taşıması
(7.5/14.5)^2 = %27. Soru tam olarak bu farkin yeterli olup olmadigidir.

Yontem: tavan 9 deg'de sabit tutulur (tek degisken pitch olsun diye), arac
oturur, sonra pitch_sp adim adim artirilir ve her adimda v_h / vz olculur.
Ikinci ucusta ADIM SIRASI TERS verilir (Adim 20 kurali).

ISARET (position_loop.m): +theta = burun YUKARI = GERI kuvvet.

Kullanim:
    python3 run_pitchbrake_probe.py [pitch_deg_listesi]
    python3 run_pitchbrake_probe.py 2,4,6
    python3 run_pitchbrake_probe.py 6,4,2
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
CEIL_FLOOR = 9.0      # deg, Faz 1'de yaw'in kabul edilebilir kaldigi taban
RETRACT_RATE = 2.0    # deg/s
SETTLE_AT_FLOOR_S = 25.0
PITCH_STEP_S = 20.0   # s, her pitch adimi
POSHOLD_TRY_S = 12.0

LOG_TOPICS_SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "logger_topics_shadow.txt")
LOG_TOPICS_DST_DIR = "/home/umran/PX4-Autopilot/build/px4_sitl_default/rootfs/etc/logging"
LOG_TOPICS_DST = os.path.join(LOG_TOPICS_DST_DIR, "logger_topics.txt")
ULOG_DIR = "/home/umran/PX4-Autopilot/build/px4_sitl_default/rootfs/log"

ABORT_Z_LOW = -4.0     # m NED, yere yakinlik
ABORT_CLIMB_M = 15.0   # m, hedefin bu kadar ustune cikarsa Adim 29 tekrar ediyor demektir


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
    pitches = [float(x) for x in sys.argv[1].split(",")] if len(sys.argv) > 1 else [2.0, 4.0, 6.0]
    # 2. arg "release": pitch adimlarindan ONCE tavani birak (90 deg).
    # Gerekce (ilk kosuda olculdu): taban 9 deg'de delta0 tavana, delta1
    # TILT_MIN'e cakili -> diferansiyel SABIT, yaw'in kontrol otoritesi sifir.
    # Seyirde aero weathervane bunu maskeliyor, yavaslayinca maske kalkiyor ve
    # arac kaciyor (pitch +4'te 981 deg, +6'da 2117 deg donus).
    # Ama tavanin GEREKCESI de yavaska yok: taban fazlarinda nu_des(Fz) ~ 0.00,
    # yani Fz kaynakli tilt kacisi zaten bitmis. Tavan yalnizca hizliyken gerekli.
    release_ceiling = len(sys.argv) > 2 and sys.argv[2] == "release"

    px4 = sc.Px4Client()
    out_dir = os.path.dirname(os.path.abspath(__file__))
    log_path = os.path.join(out_dir, "px4_pitchbrake_probe.log")

    install_log_profile()
    print(f"=== SITL baslatiliyor (pitch adimlari = {pitches} deg) ===")
    sc.launch_sitl("gz_tiltrotor_indi", log_path)

    yaw_ref = [float("nan")]

    def snap():
        lp = px4.local_position()
        st = px4.status()
        _, pitch_now, yaw_now = px4.attitude_euler_deg()
        yaw_err = float("nan")
        if math.isfinite(yaw_now) and math.isfinite(yaw_ref[0]):
            yaw_err = (yaw_now - yaw_ref[0] + 180.0) % 360.0 - 180.0
        u = st.get("u_actual", [float("nan")] * 6)
        sat = st.get("sat_flag", [False] * 6)
        return {
            "vh": math.hypot(lp.get("vx", float("nan")), lp.get("vy", float("nan"))),
            "z": lp.get("z", float("nan")),
            "vz": lp.get("vz", float("nan")),
            "pitch": pitch_now,
            "tilt": [math.degrees(u[3 + i]) if len(u) >= 6 else float("nan") for i in range(3)],
            "T": [u[i] if len(u) >= 6 else float("nan") for i in range(3)],
            "yaw_err": yaw_err,
            "sat": sum(1 for s in sat if s) if isinstance(sat, list) else 0,
        }

    def show(tag, t, s, extra=""):
        print(f"  {tag} t={t:5.1f}  v_h={s['vh']:6.2f}  vz={s['vz']:+6.2f}  z={s['z']:7.2f}  "
              f"pitch={s['pitch']:+5.1f}  tilt={s['tilt'][0]:5.1f}/{s['tilt'][1]:4.1f}  "
              f"T={s['T'][0]:5.1f}/{s['T'][1]:5.1f}/{s['T'][2]:5.1f}  yaw={s['yaw_err']:+6.1f}{extra}")

    results = []
    aborted = False

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

        def send_sp(fx=0.0, pitch_deg=0.0, pos_hold=False):
            px4.set_setpoint(roll=0.0, pitch=math.radians(pitch_deg), yaw=yaw0,
                             fx=fx, z_sp=z_sp, leso_enable=(True, True, False),
                             pos_hold=pos_hold)

        def check_abort(s):
            # Telemetri kopmasi = KOR UCUS. Bir kez yasandi (2026-07-29): px4-listener
            # / tiltceil istemci yolu tumuyle sessizce basarisiz oldu, her okuma nan
            # dondu, tavan komutlari hic gitmedi ve arac 24 m/s'ye hizlanip 44 m
            # tirmandi. Okuma yoksa manevraya devam etmek anlamsiz ve tehlikeli.
            if not (math.isfinite(s["z"]) and math.isfinite(s["vh"])):
                print("   !! IPTAL: telemetri okunamiyor (nan) -- kor ucus, durduruluyor")
                return True
            if s["z"] > ABORT_Z_LOW:
                print(f"   !! IPTAL: yere yakin, z={s['z']:.2f}")
                return True
            if s["z"] < z_sp - ABORT_CLIMB_M:
                print(f"   !! IPTAL: KACIS TIRMANISI, z={s['z']:.2f} < hedef{z_sp:.1f}-{ABORT_CLIMB_M:.0f}")
                return True
            return False

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
        print(f"   GIRIS: v_h={s['vh']:.2f}  tilt={s['tilt'][0]:.1f}/{s['tilt'][1]:.1f}  z={s['z']:.2f}")

        print(f"3) tavan {CEIL_FLOOR} deg'e iniyor ({RETRACT_RATE} deg/s)")
        ceil = max(s["tilt"][0], s["tilt"][1])
        t0 = time.monotonic()
        while ceil > CEIL_FLOOR:
            t = time.monotonic() - t0
            ceil = max(CEIL_FLOOR, max(s["tilt"][0], s["tilt"][1]) - RETRACT_RATE * t)
            px4.tiltceil(ceil)
            send_sp(fx=0.0)
            time.sleep(0.25)

        print(f"4) tavan tabaninda oturma ({SETTLE_AT_FLOOR_S:.0f} s) -- pitch = 0")
        t0 = time.monotonic()
        while time.monotonic() - t0 < SETTLE_AT_FLOOR_S:
            px4.tiltceil(CEIL_FLOOR)
            send_sp(fx=0.0, pitch_deg=0.0)
            time.sleep(0.5)
        s = snap()
        if check_abort(s):
            raise RuntimeError("taban oturmasi sonunda telemetri/irtifa gecersiz -- kosu iptal")
        show("[0]", 0.0, s, "   <- pitch=0 referansi")
        results.append((0.0, s["vh"], s["vz"], s["z"]))
        v_ref = s["vh"]

        # 5) pitch adimlari, tavan sabit
        ceil_now = CEIL_FLOOR
        if release_ceiling:
            print("4b) TAVAN BIRAKILIYOR (90 deg) -- yaw aktuatoru geri veriliyor")
            ceil_now = 90.0
            px4.tiltceil(90.0)

        print(f"5) PITCH ADIMLARI (tavan {ceil_now:.0f} deg), her biri {PITCH_STEP_S:.0f} s")
        for pd in pitches:
            print(f"   --- pitch_sp = +{pd} deg ---")
            t0 = time.monotonic()
            last = -99.0
            while time.monotonic() - t0 < PITCH_STEP_S:
                t = time.monotonic() - t0
                px4.tiltceil(ceil_now)
                send_sp(fx=0.0, pitch_deg=pd)
                s = snap()
                if t - last >= 5.0:
                    show(f"[+{pd:.0f}]", t, s)
                    last = t
                if check_abort(s):
                    aborted = True
                    break
                time.sleep(0.5)
            if aborted:
                break
            s = snap()
            results.append((pd, s["vh"], s["vz"], s["z"]))

        # 6) pitch'i geri al, pos_hold dene
        if not aborted:
            print(f"6) pitch -> 0, pos_hold denemesi")
            t0 = time.monotonic()
            while time.monotonic() - t0 < POSHOLD_TRY_S:
                px4.tiltceil(ceil_now)
                send_sp(pitch_deg=0.0, pos_hold=True)
                time.sleep(1.0)
            s = snap()
            show("[P]", POSHOLD_TRY_S, s)

        px4.disarm(force=True)
        time.sleep(2.0)

    finally:
        sc.kill_sitl()
        remove_log_profile()

    print(f"\n{'pitch_sp':>9}{'v_h':>9}{'vz':>9}{'z':>9}")
    for pd, vh, vz, z in results:
        print(f"{pd:>8.0f}d{vh:>9.2f}{vz:>+9.2f}{z:>9.2f}")
    if len(results) > 1:
        print(f"\npitch=0 referansi v_h = {v_ref:.2f} m/s")
        print("FRENLIYOR ise v_h dusmeli ve vz ~0 kalmali; TIRMANIS ise vz belirgin negatif olur")
    print(f"\nulog: {newest_ulog()}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
