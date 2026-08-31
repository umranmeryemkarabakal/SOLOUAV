#!/usr/bin/env python3
"""Adim 31 / Faz 1 -- tilt tavani mekanizmasinin ilk ucusu.

Faz 0'da olculdu: seyirde tilt'i ILERI suren sey irtifa (Fz) kanali; Fx talebi
atil (agirlik farki 160.000x). Dolayisiyla tilt'i geri cekmenin tek yolu
AGIRLIK degil KISIT -- tahsisat kutusunun ust siniri (`tiltceil`, kanat
rotorleri icin abs_hi).

Bu betik yasa DEGIL, mekanizmanin elle surulmesidir. Faz 2'nin durum makinesi
tam olarak bunu otomatiklestirecek; once tavanin gercekte ne yaptigini gormek
gerekiyor.

ACIK OLAN TEK SORU (Faz 0'in ongordugu kuplaj): tavan tilt'i asagi zorlayinca
irtifa dongusu tercih ettigi "tasima bosaltma" aktuatorunu kaybeder ve geriye
kalan itki yuksek tilt'te zayiftir (cos delta). Yani araç tirmanabilir. Bunu
ancak ucurarak ogrenebiliriz -- bu kosunun asil olcumu budur.

Zaman cizelgesi:
  1. pos_hold ile gercek hover, CLIMB_M tirmanis
  2. pos_hold birak, fx rampasi -> ~15 m/s seyir
  3. NOTRLUK KONTROLU: tavan 90 deg (varsayilan) ile NEUTRAL_S bekle
     -> Faz 0'daki ileri surukleme aynen gorunmeli
  4. GERI CEKME: tavan mevcut kanat tilt'inden RATE deg/s ile TILT_FLOOR'a iner
  5. BEKLEME: tavan tabanda, DWELL_S -- hiz dusuyor mu?
  6. pos_hold denemesi (v_h < 3 m/s ise kabul edilmeli)

Kullanim:
    python3 run_tiltceil_probe.py [rate_deg_s]
"""

from __future__ import annotations

import math
import os
import shutil
import sys
import time

import indi_sitl_common as sc

CLIMB_M = 25.0        # m, geri cekme sirasinda tirmanma ihtimaline karsi pay
SETTLE_S = 12.0
FX_FINAL = 15.0
T_RAMP = 12.0
ACCEL_HOLD_S = 10.0
NEUTRAL_S = 8.0       # s, tavan kapaliyken referans davranis
TILT_FLOOR = 9.0      # deg, hover trim civari (hoverTrim() delta0 ~ 9.4 deg)
DWELL_S = 25.0        # s, her tavan tabaninda (supurmede 3 kez)
POSHOLD_TRY_S = 15.0  # s, sonunda pos_hold denemesi

# Tavanin izlenebilecegi en yuksek hiz (Adim 26):
#   etkin slew = TILT_SLEW_BOX_RATE * TS_BOX / TILT_TAU = 3.0*(1/250)/0.15
#             = 0.080 rad/s = 4.58 deg/s
# Bunun ustunde tavan aktuatoru gecer ve rate artik ayarlanabilir olmaktan cikar.
MAX_TRACKABLE_DEG_S = 4.58

LOG_TOPICS_SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "logger_topics_shadow.txt")
LOG_TOPICS_DST_DIR = "/home/umran/PX4-Autopilot/build/px4_sitl_default/rootfs/etc/logging"
LOG_TOPICS_DST = os.path.join(LOG_TOPICS_DST_DIR, "logger_topics.txt")
ULOG_DIR = "/home/umran/PX4-Autopilot/build/px4_sitl_default/rootfs/log"

ABORT_Z = -4.0        # m NED, bunun uzerine cikarsa (yere yaklasirsa) durdur


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
    rate = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
    # Taban supurmesi. Ikinci ucusta SIRAYI TERS verin (Adim 20 kurali):
    #   python3 run_tiltceil_probe.py 2.0 9,7,5
    #   python3 run_tiltceil_probe.py 2.0 5,7,9
    floors = [float(x) for x in sys.argv[2].split(",")] if len(sys.argv) > 2 else [9.0, 7.0, 5.0]
    # 3. arg "nohold": sondaki pos_hold denemesini atla. Yaw dose-response
    # kosularinda gerekli -- 5 deg tabanda yaw zaten zayifken ek bir manevra
    # istemiyoruz (Ucus A'da o pencerede kendiliginden kacis oldu).
    try_poshold = not (len(sys.argv) > 3 and sys.argv[3] == "nohold")
    if rate > MAX_TRACKABLE_DEG_S:
        print(f"UYARI: {rate} deg/s izlenebilir tavanin ({MAX_TRACKABLE_DEG_S}) ustunde -- "
              f"aktuator tavani gecemez, rate ayarlanabilir olmaktan cikar")

    px4 = sc.Px4Client()
    out_dir = os.path.dirname(os.path.abspath(__file__))
    log_path = os.path.join(out_dir, "px4_tiltceil_probe.log")

    install_log_profile()
    print(f"=== SITL baslatiliyor (geri cekme hizi = {rate} deg/s) ===")
    sc.launch_sitl("gz_tiltrotor_indi", log_path)

    yaw_ref = [float("nan")]   # arm heading, doldurulunca yaw hatasi olculur

    def snap():
        lp = px4.local_position()
        st = px4.status()
        _, _, yaw_now = px4.attitude_euler_deg()
        yaw_err = float("nan")
        if math.isfinite(yaw_now) and math.isfinite(yaw_ref[0]):
            yaw_err = (yaw_now - yaw_ref[0] + 180.0) % 360.0 - 180.0
        u = st.get("u_actual", [float("nan")] * 6)
        nu = st.get("nu_des", [float("nan")] * 5)
        sat = st.get("sat_flag", [False] * 6)
        return {
            "vh": math.hypot(lp.get("vx", float("nan")), lp.get("vy", float("nan"))),
            "z": lp.get("z", float("nan")),
            "vz": lp.get("vz", float("nan")),
            "tilt": [math.degrees(u[3 + i]) if len(u) >= 6 else float("nan") for i in range(3)],
            "T": [u[i] if len(u) >= 6 else float("nan") for i in range(3)],
            "nu_fz": nu[4] if len(nu) == 5 else float("nan"),
            "nu_fx": nu[3] if len(nu) == 5 else float("nan"),
            "sat": sum(1 for s in sat if s) if isinstance(sat, list) else 0,
            "yaw_err": yaw_err,
        }

    def show(tag, t, s, ceil):
        print(f"  {tag} t={t:5.1f} ceil={ceil:5.1f}  v_h={s['vh']:6.2f}  z={s['z']:7.2f}  "
              f"vz={s['vz']:+5.2f}  tilt={s['tilt'][0]:5.1f}/{s['tilt'][1]:5.1f}/{s['tilt'][2]:4.1f}  "
              f"T={s['T'][0]:5.1f}/{s['T'][1]:5.1f}/{s['T'][2]:5.1f}  "
              f"nuFz={s['nu_fz']:+6.2f}  yaw={s['yaw_err']:+5.1f}  sat={s['sat']}")

    try:
        if not sc.wait_until(px4.preflight_check_ok, timeout=20.0, poll_interval=0.5):
            print("UYARI: preflight 20 s icinde temizlenmedi")

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

        def send_sp(fx=0.0, pos_hold=False):
            px4.set_setpoint(roll=0.0, pitch=0.0, yaw=yaw0, fx=fx, z_sp=z_sp,
                             leso_enable=(True, True, False), pos_hold=pos_hold)

        # 1) hover
        print(f"1) hover + {CLIMB_M} m tirmanis")
        t0 = time.monotonic()
        while time.monotonic() - t0 < SETTLE_S:
            send_sp(pos_hold=True)
            time.sleep(1.0)

        # 2) hizlanma
        print(f"2) fx rampasi -> seyir")
        t0 = time.monotonic()
        while True:
            t = time.monotonic() - t0
            if t >= T_RAMP + ACCEL_HOLD_S:
                break
            send_sp(fx=FX_FINAL * min(1.0, t / T_RAMP))
            time.sleep(0.5)

        s = snap()
        print(f"   GIRIS: v_h={s['vh']:.2f} m/s  tilt={s['tilt'][0]:.1f}/{s['tilt'][1]:.1f}/"
              f"{s['tilt'][2]:.1f}  z={s['z']:.2f}")

        # 3) notrluk kontrolu -- tavan varsayilan (90 deg) iken
        print(f"3) NOTRLUK: tavan 90 deg, {NEUTRAL_S} s (Faz 0'daki ileri surukleme gorunmeli)")
        t0 = time.monotonic()
        while time.monotonic() - t0 < NEUTRAL_S:
            send_sp(fx=0.0)
            time.sleep(2.0)
            show("[N]", time.monotonic() - t0, snap(), 90.0)

        s_pre = snap()
        ceil = max(s_pre["tilt"][0], s_pre["tilt"][1])
        print(f"   notr sonu: kanat tilt {s_pre['tilt'][0]:.1f}/{s_pre['tilt'][1]:.1f} "
              f"-> tavan {ceil:.1f} deg'den baslatiliyor")

        # 4) geri cekme rampasi
        print(f"4) GERI CEKME: tavan {ceil:.1f} -> {floors[0]} deg, {rate} deg/s")
        px4.tiltceil(ceil)
        t0 = time.monotonic()
        last_print = -99.0
        aborted = False
        while ceil > floors[0]:
            t = time.monotonic() - t0
            ceil = max(floors[0], (max(s_pre["tilt"][0], s_pre["tilt"][1])) - rate * t)
            px4.tiltceil(ceil)
            send_sp(fx=0.0)
            s = snap()
            if t - last_print >= 3.0:
                show("[R]", t, s, ceil)
                last_print = t
            if s["z"] > ABORT_Z:
                print(f"   !! IPTAL: z={s['z']:.2f} m, yere cok yakin")
                aborted = True
                break
            time.sleep(0.25)

        # 5) TABAN SUPURMESI -- her tabanda terminal hiz olculur.
        # Adim 31 / Faz 1 ilk kosusunda hiz 9 deg tavanda ~7.5 m/s'de platoya
        # oturdu: delta0 = 9 deg'de kalan Fx = T0*sin(9 deg) ~ 2.5 N, ve suruklenme
        # bunu 7.5 m/s'de dengeliyor. Yani terminal hizi tavan TABANI belirliyor.
        # Supurme ucus ICINDE yapilir (Adim 20/23 disiplini: ucuslar arasi
        # karsilastirma marjinal kararli bir araci kendi varyansiyla karistirir).
        if not aborted:
            print(f"5) TABAN SUPURMESI: {floors} deg, her biri {DWELL_S:.0f} s")
            for fl in floors:
                print(f"   --- taban {fl} deg ---")
                t0 = time.monotonic()
                last_print = -99.0
                while time.monotonic() - t0 < DWELL_S:
                    t = time.monotonic() - t0
                    px4.tiltceil(fl)
                    send_sp(fx=0.0)
                    s = snap()
                    if t - last_print >= 6.0:
                        show("[D]", t, s, fl)
                        last_print = t
                    if s["z"] > ABORT_Z:
                        print(f"   !! IPTAL: z={s['z']:.2f} m")
                        aborted = True
                        break
                    time.sleep(0.5)
                if aborted:
                    break
                s = snap()
                print(f"   taban {fl} deg -> terminal v_h = {s['vh']:.2f} m/s, "
                      f"tilt {s['tilt'][0]:.1f}/{s['tilt'][1]:.1f}, yaw hatasi {s['yaw_err']:+.1f} deg")

        # 6) pos_hold denemesi
        if not aborted and try_poshold:
            s = snap()
            print(f"6) pos_hold denemesi (v_h = {s['vh']:.2f} m/s)")
            t0 = time.monotonic()
            while time.monotonic() - t0 < POSHOLD_TRY_S:
                px4.tiltceil(floors[-1])
                send_sp(pos_hold=True)
                time.sleep(1.0)
            s = snap()
            show("[P]", POSHOLD_TRY_S, s, floors[-1])

        print("disarm")
        px4.disarm(force=True)
        time.sleep(2.0)

    finally:
        sc.kill_sitl()
        remove_log_profile()

    print(f"\nulog: {newest_ulog()}")
    print("pos_hold sonucu icin px4 log'una bak:")
    print(f"  grep -E 'pos_hold|tiltceil' {log_path} | tail -20")
    return 0


if __name__ == "__main__":
    sys.exit(main())
