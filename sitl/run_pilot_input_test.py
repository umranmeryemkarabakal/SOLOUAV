#!/usr/bin/env python3
"""B1 pilot yolu -- ILK KEZ CALISTIRILAN kriter kosusu. (Adim 40, 2026-08-03)

Pilot girisi Adim 33'te yazildi ve bugune kadar HIC calistirilmadi: kontrol
listesinde B1 tam olarak bu yuzden 🔴 duruyor. Bu projede calistirilmamis kod
yolu iki kez pahaliya patladi (Adim 34: RATE_ONLY ilk tetiklendiginde arac
dustu; Adim 36: "ulasilamaz" ve "olu" hukumlerinin IKISI de yanlisti), o yuzden
buradaki amac "pilot yolu var mi" degil, HER DALINI bir kez gercekten kosturmak.

Giris yolu: MAVLink MANUAL_CONTROL -> mavlink_receiver -> `manual_control_input`
-> manual_control modulu -> `manual_control_setpoint`. Param DEGISTIRILMEZ:
COM_RC_IN_MODE varsayilani 3 ("RC or Joystick, keep first") joystick'i zaten
kabul ediyor (probe_pilot_link.py ile olculdu). SITL param'lari kalici oldugu
ve bir sonraki kosuya sizdigi icin (RUNBOOK tuzagi) hicbir param'a dokunmamak
bilincli bir tercihtir.

OLCUTLER -- her biri modulun BIR dalina karsilik gelir:
  1. Devralma: `pilot input ACTIVE` bir kez yazilir ve BASAMAK YOKTUR
     (yaw/z hedefleri aracin bulundugu yerden tohumlanir -- kod bunu iddia
     ediyor, olculmemisti).
  2. Cubuklar ORTADA -> pos_hold KENDILIGINDEN devreye girer (madde (N):
     hands-off hover VARSAYILAN olmali, pilotun hatirlamasi gereken bir mod
     degil).
  3. Roll/pitch cubugu -> duruş komutu izlenir VE pos_hold birakilir.
  4. Yaw cubugu -> heading doner, ve LEASH calisir (tutulan cubuk setpoint'i
     aracin donebileceginden hizli goturmemeli -- Adim 13'un ariza modu).
  5. Throttle cubugu -> tirmanma/alcalma, z leash icinde.
  6. LINK KAYBI -> `pilot input LOST`, pozisyon tutulur ve alcalinir.
  7. Butun kosu boyunca havada NaN cikis YOK ve BIG_M = 0.

Kullanim:
    export PATH=~/PX4-Autopilot/build/px4_sitl_default/bin:$PATH
    python3 run_pilot_input_test.py            # INDI_SITL_GUI=1 ile izlenebilir
"""

from __future__ import annotations

import math
import os
import shutil
import sys
import threading
import time

import numpy as np
from pyulog import ULog

import indi_sitl_common as sc

try:
    from pymavlink import mavutil
except ImportError:
    print("HATA: pymavlink yok (pip install pymavlink)")
    sys.exit(2)

MAV_ADDR = "udpin:127.0.0.1:14540"
STREAM_HZ = 50.0

# CLIMB_S 14 -> 26 s (2026-08-03, Adim 41). 14 s ile arac ~13 m'ye cikiyor ve
# throttle-asagi evresinden sonra link kaybi 7.2 m AGL'de olusuyordu -- yani
# BT_MIN_ALT'in (15 m) ALTINDA, ve geri gecis hakli olarak REDDEDIYORDU
# (`back-transition REFUSED: 7.2 m AGL < 15.0 m minimum`). Madde (U)'nun
# hedefledigi durum "hizli VE irtifasi olan bir arac"; testin o durumu
# kurmasi gerekiyor. Dusuk irtifadaki link kaybi ayri bir durumdur ve orada
# dogru cevap zaten "hemen in"dir.
CLIMB_S = 26.0        # throttle yukari (~30 m)
SETTLE_S = 10.0       # cubuklar ortada
STEP_S = 8.0          # her cubuk adimi
DOWN_S = 8.0          # throttle asagi (irtifayi 15 m'nin ALTINA dusurmemeli)
LINKLOSS_S = 45.0     # akis kesildikten sonra izleme (geri gecis 30-40 s surer)

# Modulun sabitleriyle ayni olmali (TiltrotorIndiParams.hpp)
MAN_TILT_MAX_DEG = 15.0
MAN_YAW_LEASH_DEG = 45.0
MAN_STICK_DEAD = 0.05
POS_ENGAGE_V_MAX = 3.0

LOG_TOPICS_SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "logger_topics_shadow.txt")
LOG_TOPICS_DST_DIR = "/home/umran/PX4-Autopilot/build/px4_sitl_default/rootfs/etc/logging"
LOG_TOPICS_DST = os.path.join(LOG_TOPICS_DST_DIR, "logger_topics.txt")
ULOG_DIR = "/home/umran/PX4-Autopilot/build/px4_sitl_default/rootfs/log"


class StickStream:
    """MANUAL_CONTROL + GCS heartbeat akisi, ayri thread. `stop()` cagrilinca
    akis KESILIR -- olcut 6 (link kaybi) tam olarak budur."""

    def __init__(self, conn):
        self.conn = conn
        self.x = self.y = self.r = 0.0
        self.z = 0.0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._t.start()

    def stop(self):
        self._stop.set()
        self._t.join(timeout=2.0)

    def set(self, x=0.0, y=0.0, z=0.0, r=0.0):
        with self._lock:
            self.x, self.y, self.z, self.r = x, y, z, r

    def _run(self):
        period = 1.0 / STREAM_HZ
        last_hb = 0.0
        while not self._stop.is_set():
            now = time.monotonic()
            if now - last_hb > 1.0:
                self.conn.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                last_hb = now
            with self._lock:
                x, y, z, r = self.x, self.y, self.z, self.r
            self.conn.mav.manual_control_send(
                self.conn.target_system,
                int(x * 1000), int(y * 1000), int((z + 1.0) * 500.0), int(r * 1000), 0)
            time.sleep(period)


def newest_ulog() -> str:
    best, best_m = "", 0.0
    for root, _d, files in os.walk(ULOG_DIR):
        for f in files:
            if f.endswith(".ulg"):
                p = os.path.join(root, f)
                if os.path.getmtime(p) > best_m:
                    best, best_m = p, os.path.getmtime(p)
    return best


def phase(px4, stream, label, secs, **sticks):
    """Bir cubuk evresi kosar ve baslangic/bitis durumunu dondurur."""
    stream.set(**sticks)
    t0 = time.monotonic()
    a = snap(px4)
    while time.monotonic() - t0 < secs:
        time.sleep(0.5)
    b = snap(px4)
    print(f"   {label:<22} roll {a['roll']:+6.1f}->{b['roll']:+6.1f}  "
          f"pitch {a['pitch']:+6.1f}->{b['pitch']:+6.1f}  "
          f"yaw {a['yaw']:+7.1f}->{b['yaw']:+7.1f}  z {a['z']:+7.2f}->{b['z']:+7.2f}")
    return a, b


def snap(px4):
    lp = px4.local_position()
    roll, pitch, yaw = px4.attitude_euler_deg()
    return {"roll": roll, "pitch": pitch, "yaw": yaw,
            "z": lp.get("z", float("nan")),
            "vh": math.hypot(lp.get("vx", float("nan")), lp.get("vy", float("nan"))),
            "t": time.monotonic()}


def main() -> int:
    px4 = sc.Px4Client()
    out_dir = os.path.dirname(os.path.abspath(__file__))
    log_path = os.path.join(out_dir, "px4_pilot_input.log")

    os.makedirs(LOG_TOPICS_DST_DIR, exist_ok=True)
    shutil.copyfile(LOG_TOPICS_SRC, LOG_TOPICS_DST)

    print("=== SITL baslatiliyor ===")
    sc.launch_sitl("gz_tiltrotor_indi", log_path)

    conn = None
    stream = None
    marks = {}

    try:
        conn = mavutil.mavlink_connection(MAV_ADDR)
        if conn.wait_heartbeat(timeout=30) is None:
            print("HATA: PX4 heartbeat yok")
            return 1
        print(f"MAVLink baglandi (sys={conn.target_system})")

        if not sc.wait_until(px4.preflight_check_ok, timeout=20.0, poll_interval=0.5):
            print("UYARI: preflight temizlenmedi")

        # Cubuklar ORTADA baslar: devralmanin basamaksiz oldugunu gormek icin
        # aracin once kendi halinde olmasi gerekiyor.
        stream = StickStream(conn)
        stream.set(x=0.0, y=0.0, z=0.0, r=0.0)
        stream.start()
        time.sleep(3.0)

        px4.arm(force=True)
        time.sleep(1.0)
        marks["arm"] = time.time()
        s0 = snap(px4)
        print(f"arm: yaw={s0['yaw']:.1f} deg  z={s0['z']:.2f} m")

        print("\n1) throttle YUKARI -- tirmanma")
        phase(px4, stream, "throttle +0.6", CLIMB_S, z=0.6)

        print("2) cubuklar ORTADA -- pos_hold kendiliginden girmeli")
        marks["centre"] = time.time()
        a, b = phase(px4, stream, "hepsi 0", SETTLE_S)
        print(f"      -> v_h = {b['vh']:.2f} m/s (pos_hold calisiyorsa kucuk olmali)")

        print("3) roll/pitch cubugu -- duruş komutu + pos_hold birakilmali")
        marks["roll"] = time.time()
        phase(px4, stream, "roll +0.5", STEP_S, y=0.5, z=0.0)
        phase(px4, stream, "orta", 4.0)
        marks["pitch"] = time.time()
        phase(px4, stream, "pitch -0.5 (burun yuk.)", STEP_S, x=-0.5)
        phase(px4, stream, "orta", 4.0)

        print("4) yaw cubugu -- heading donmeli, leash tutmali")
        marks["yaw"] = time.time()
        phase(px4, stream, "yaw +0.6", STEP_S, r=0.6)
        phase(px4, stream, "orta", 5.0)

        print("5) throttle ASAGI -- alcalma")
        marks["down"] = time.time()
        phase(px4, stream, "throttle -0.4", DOWN_S, z=-0.4)
        phase(px4, stream, "orta", 4.0)

        print("6) LINK KAYBI -- akis kesiliyor")
        # Kesme ANINI ulog saatinde isaretle. Olcut penceresi bundan kurulur;
        # ilk surum "arm noktasindan uzaklik" olcuyordu, o ise pilotun kendi
        # manevrasini da iceriyor ve link kaybinin bedelini soylemiyor.
        lp0 = px4.local_position()
        t_ll_us = lp0.get("timestamp", float("nan"))
        s_a = snap(px4)
        print(f"   kesme aninda: z={s_a['z']:+.2f} m ({-s_a['z']:.1f} m AGL), "
              f"v_h={s_a['vh']:.2f} m/s   [BT_MIN_ALT = 15 m]")
        marks["linkloss"] = time.time()
        stream.stop()
        stream = None
        t0 = time.monotonic()
        landed = 0

        while time.monotonic() - t0 < LINKLOSS_S:
            time.sleep(1.0)
            # TEMASTA DUR (2026-08-03, Adim 41). Eskiden dongu LINKLOSS_S boyunca
            # donuyordu ve arac yere degdikten sonra ARMED kalip yerde suruniyordu:
            # olculdu, o evrede 3294 BIG_M, tilt 48.6 deg'e, yaw 823 deg'lik bir
            # span'e gidiyor -- hepsi YERDE. GUI'de izleyene "arac kontrolu
            # kaybetti" gibi gorunuyor ve olcut ozetlerini de kirletiyor
            # (havadaki gercek deger 3.7-15.1 deg tilt, 0 BIG_M). Ayni tuzak
            # run_lockup_check.py'de zaten cozulmustu; buraya tasinmamisti.
            zc = px4.local_position().get("z", float("nan"))
            landed = landed + 1 if (math.isfinite(zc) and zc > -1.0) else 0

            if landed >= 3:
                print(f"   yere indi (z = {zc:+.2f} m, 3 ornek) -> izleme bitti")
                break

        s_b = snap(px4)
        print(f"   z {s_a['z']:+.2f} -> {s_b['z']:+.2f} m   v_h {s_a['vh']:.2f} -> {s_b['vh']:.2f} m/s")

        px4.disarm(force=True)
        time.sleep(2.0)

    finally:
        if stream is not None:
            stream.stop()
        if conn is not None:
            conn.close()
        sc.kill_sitl()
        try:
            os.remove(LOG_TOPICS_DST)
        except FileNotFoundError:
            pass

    ulog = newest_ulog()
    print(f"\nulog: {ulog}\npx4 log: {log_path}\n")
    ok = analyze(ulog, log_path, t_ll_us)
    print()
    import check_output_cuts
    check_output_cuts.check(ulog)
    return 0 if ok else 1


def analyze(path: str, px4_log: str, t_ll_us: float = float('nan')) -> bool:
    """Olcutler ulog + px4 konsol log'undan. Konsol log'u burada MESRU bir
    kaynak: olculen seylerin bir kismi (devralma, link kaybi) modulun BIR KEZ
    yazdigi durum gecisleri ve telemetri alani yok."""
    u = ULog(path, ["tiltrotor_indi_status", "vehicle_local_position", "vehicle_attitude"])
    d = {m.name: m.data for m in u.data_list}
    st = d["tiltrotor_indi_status"]
    ts = np.asarray(st["timestamp"], float)
    keep = np.concatenate(([True], np.diff(ts) > 1e-6))
    sat = np.column_stack([np.asarray(st["sat_flag[%d]" % i], float)[keep] for i in range(6)])
    pha = (np.asarray(st["pos_hold_active"], float)[keep] > 0.5
           if "pos_hold_active" in st else None)

    with open(px4_log) as f:
        log = f.read()

    n_active = log.count("pilot input ACTIVE")
    n_lost = log.count("pilot input LOST")
    n_attlost = log.count("attitude LOST")

    print("=== PILOT GIRISI ANALIZI ===")
    c1 = n_active == 1
    print(f"  1) devralma          : 'pilot input ACTIVE' x{n_active} "
          f"(tam 1 olmali) ......... {'GECTI' if c1 else 'KALDI'}")

    c2 = pha is not None and bool(pha.any())
    frac = 100.0 * pha.mean() if pha is not None else float("nan")
    print(f"  2) hands-off pos_hold: kosunun %{frac:.1f}'inde aktif "
          f"................. {'GECTI' if c2 else 'KALDI'}")

    lp = d["vehicle_local_position"]
    tl = np.asarray(lp["timestamp"], float) / 1e6
    z = np.asarray(lp["z"], float)
    at = d["vehicle_attitude"]
    ta = np.asarray(at["timestamp"], float) / 1e6
    q = np.column_stack([np.asarray(at["q[%d]" % i], float) for i in range(4)])
    roll = np.degrees(np.arctan2(2 * (q[:, 0] * q[:, 1] + q[:, 2] * q[:, 3]),
                                 1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2)))
    pitch = np.degrees(np.arcsin(np.clip(2 * (q[:, 0] * q[:, 2] - q[:, 3] * q[:, 1]), -1, 1)))
    yaw = np.unwrap(np.arctan2(2 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]),
                               1 - 2 * (q[:, 2] ** 2 + q[:, 3] ** 2)))

    # OLCUT PENCERESI: yalnizca PILOTUN roll/pitch'e sahip oldugu ornekler.
    # `MAN_TILT_MAX` pilot komutunun tavanidir; pos_hold devredeyken acilari
    # POZISYON DONGUSU secer (kendi tavani POS_TILT_MAX) ve yere temasta konum
    # hatasini duzeltmek icin daha buyuk aci komut eder. Ilk surum butun ucusa
    # bakiyordu ve 22.1 deg ile KALDI verdi -- o deger t=100.8 s'de, z=-1.7 m'de,
    # pos_hold %95 aktifken, yani YERE TEMASTA olusmustu. Pilot sahipken olculen
    # 8.5 deg (0.5 cubuk x 15 deg = 7.5 deg komutunun karsiligi). Ayni hata
    # sinifi madde (S) olcutunde de yapilmisti: bir olcut, olctugu buyuklugun
    # SAHIBI olan yasanin aktif oldugu pencereye kisitlanmali.
    pha_att = np.interp(ta, np.asarray(st["timestamp"], float)[keep] / 1e6,
                        (pha if pha is not None else np.zeros(keep.sum())).astype(float)) >= 0.5
    man_owns = ~pha_att
    r_man = np.abs(roll[man_owns]).max() if man_owns.any() else float("nan")
    p_man = np.abs(pitch[man_owns]).max() if man_owns.any() else float("nan")
    c3 = (r_man <= MAN_TILT_MAX_DEG + 3.0) and (p_man <= MAN_TILT_MAX_DEG + 3.0)
    print(f"  3) duruş komutu      : PILOT sahipken max |roll| {r_man:.1f}, "
          f"max |pitch| {p_man:.1f} deg (<= {MAN_TILT_MAX_DEG:.0f}+3) ... "
          f"{'GECTI' if c3 else 'KALDI'}")
    print(f"       (pos_hold sahipken {np.abs(pitch[pha_att]).max():.1f} deg -- "
          f"o pozisyon dongusunun tavani, bu olcutun konusu degil)")

    yaw_span = math.degrees(yaw.max() - yaw.min())
    c4 = yaw_span > 5.0
    print(f"  4) yaw cubugu        : toplam heading degisimi {yaw_span:.1f} deg "
          f"(>5 = cubuk isliyor) . {'GECTI' if c4 else 'KALDI'}")

    c5 = (z.max() - z.min()) > 2.0
    print(f"  5) throttle          : irtifa araligi {z.max() - z.min():.2f} m "
          f"(>2 = cubuk isliyor) ..... {'GECTI' if c5 else 'KALDI'}")

    # Olcut 6 iki parcali: mesajin YAZILMASI (dal calisti mi) ve dalin SAVININ
    # tutmasi (pozisyon gercekten tutuldu mu). Ilk surum yalnizca birincisine
    # bakiyordu ve ikinci ucusta GECTI verdi -- oysa arac o kosuda link kaybi
    # boyunca 4.8-6.1 m/s ile UCUP GITTI ve ~100 m ileride yere indi. Bir dalin
    # varligini olcmek, savini olcmek degildir (Adim 35'in dersi).
    lost_hold = float("nan")
    if pha is not None and n_lost >= 1:
        # Link kaybi evresi: son 'pilot input LOST' sonrasi, HAVADA olan kisim.
        z_st = np.interp(np.asarray(st["timestamp"], float)[keep] / 1e6, tl, z)
        vh_st = np.interp(np.asarray(st["timestamp"], float)[keep] / 1e6, tl,
                          np.hypot(np.asarray(lp["vx"], float), np.asarray(lp["vy"], float)))
        # Dal aktifken pos_hold'un devrede oldugu ornek orani, yalnizca havada.
        tail_air = (z_st < -3.0)
        n = int(tail_air.sum())
        lost_hold = float(vh_st[tail_air][-int(0.25 * n):].mean()) if n > 8 else float("nan")

    c6 = (n_lost >= 1) and (not math.isfinite(lost_hold) or lost_hold <= POS_ENGAGE_V_MAX)
    print(f"  6) link kaybi        : 'pilot input LOST' x{n_lost}, alcalirken ort v_h "
          f"{lost_hold:.2f} m/s (<= {POS_ENGAGE_V_MAX:.1f}) .. {'GECTI' if c6 else 'KALDI'}")
    if math.isfinite(lost_hold) and lost_hold > POS_ENGAGE_V_MAX:
        print("       -> madde (U): dal 'pozisyon tutuluyor' diyor ama hold "
              "POS_ENGAGE_V_MAX'ten reddediliyor; arac ucup gidiyor")

    # Madde (U)'nun ASIL bedeli: nereye ve ne hizla iniyor. Temas, HAVADAKI SON
    # ornekten SONRAKI ilk yer ornegidir -- `z > -1` in ilk ornegi kalkistan
    # onceki yerdir ve ilk olcumumde tam bu hatayi yaptim (368 m yerine 0 m
    # yazdirdi). Olcum penceresini "once havada olmali" diye kurmak sart.
    x = np.asarray(lp["x"], float)
    y = np.asarray(lp["y"], float)
    vh_all = np.hypot(np.asarray(lp["vx"], float), np.asarray(lp["vy"], float))
    air = np.nonzero(z < -3.0)[0]
    if air.size:
        after = np.nonzero((z > -1.0) & (np.arange(z.size) > air[-1]))[0]
        i_td = int(after[0]) if after.size else int(air[-1])
        # Mesafe LINK KAYBINDAN ITIBAREN olculur, arm noktasindan degil: ilk
        # surum arm noktasini kullaniyordu ve pilotun kendi manevrasini da
        # sayiyordu, yani dalin bedelini degil ucusun toplamini raporluyordu.
        if math.isfinite(t_ll_us):
            i0 = int(np.argmin(np.abs(np.asarray(lp["timestamp"], float) - t_ll_us)))
            d = math.hypot(x[i_td] - x[i0], y[i_td] - y[i0])
            print(f"       link kaybindan temasa: {d:.0f} m yatay yol, "
                  f"{tl[i_td] - tl[i0]:.0f} s, temas v_h {vh_all[i_td]:.2f} m/s")
        print(f"       temas: v_h {vh_all[i_td]:.2f} m/s "
              f"(havadaki son ornek {vh_all[air[-1]]:.2f} m/s)")
        print("       [madde (U) baseline (duzeltme oncesi, dusuk irtifa): "
              "104-110 m link-kaybi yolu, 4.96-5.39 m/s]")

    # OLCUT PENCERESI: HAVADA. Yere temas bu projede zaten olculmus, bilinen bir
    # BIG_M kaynagi (Adim 27: 1.5 m kademeli inis 13 BIG_M, 1.0 m kademe sifir)
    # ve pilot yolunun ozelligi degil. Ilk surum butun armed pencereye bakiyordu
    # ve 283 BIG_M ile KALDI verdi; 283'un TAMAMI t=99.6-101.0 s araliginda,
    # z=-1.7 m'de, yani tekerlek yere degerken olustu. Yine de RAPORLANIR --
    # olcutten cikarmak, gormezden gelmek degildir.
    z_st = np.interp(np.asarray(st["timestamp"], float)[keep] / 1e6, tl, z)
    airborne = z_st < -3.0
    big_air = int(sat[airborne, :3].any(axis=1).sum())
    sat_air = 100.0 * float(sat[airborne, :3].mean())
    big_all = int(sat[:, :3].any(axis=1).sum())
    c7 = (big_air == 0) and (sat_air == 0.0) and (n_attlost == 0)
    print(f"  7) saglik (HAVADA)   : itki doyumu %{sat_air:.2f}, BIG_M {big_air}, "
          f"'attitude LOST' x{n_attlost} .. {'GECTI' if c7 else 'KALDI'}")
    print(f"       (tum kosuda BIG_M {big_all}; farki yere temas evresidir, "
          f"Adim 27'de zaten olculmustu)")

    ok = all([c1, c2, c3, c4, c5, c6, c7])
    print(f"\n  SONUC: {'GECTI' if ok else 'KALDI'}")
    return ok


if __name__ == "__main__":
    sys.exit(main())
