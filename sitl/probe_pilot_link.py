#!/usr/bin/env python3
"""Pilot yolunun ULASILABILIR olup olmadigini olcer -- ucus YOK. (Adim 40)

NEDEN ONCE BU
-------------
`manual_control_setpoint` yolu Adim 33'te yazildi ve BUGUNE KADAR BIR KEZ BILE
CALISTIRILMADI (kontrol listesi B1: "pilot yolu hic calistirilmadi"). Bu projede
calistirilmamis kod yolu iki kez pahaliya patladi: Adim 34'te RATE_ONLY seviyesi
ilk tetiklendiginde arac dustu, Adim 36'da "ulasilamaz" sanilan bir dal aslinda
ulasilabilirdi ve "olu" sanilan fonksiyon olu degildi. O yuzden once UCMADAN,
yalnizca yolun acik olup olmadigi olculur.

Modulun pilot dali IKI kosula birden bagli (MulticopterIndiTiltrotor.cpp:544):
    _vehicle_control_mode.flag_control_manual_enabled   <- COMMANDER verir
    && _manual_control_setpoint_sub.copy(&man) && man.valid && taze
Ikincisi bizim gonderdigimiz seydir; BIRINCISI DEGIL. Commander'in bu airframe'de
manuel bir nav_state'e girip girmedigi bilinmiyor -- `.post` betigi
`flight_mode_manager`'i durduruyor ve Adim 34 commander'in bu airframe'de
"yedek mod yok" davrandigini olctu. Yani asil risk ikinci kosulda degil,
BIRINCISINDE. Bu betik ikisini AYRI AYRI raporlar.

Gonderim yolu: MAVLink MANUAL_CONTROL -> mavlink_receiver ->
`manual_control_input` -> manual_control modulu -> `manual_control_setpoint`.
Param degistirmeye GEREK YOK: COM_RC_IN_MODE varsayilani 3 ("RC or Joystick,
keep first"), yani joystick zaten kabul ediliyor. (Param degistirmemek bilincli:
SITL'de param'lar kalicidir ve bir sonraki kosuya sizar -- RUNBOOK tuzagi.)

NOT: `manual_control_switches` (VTOL anahtari -> bt_enable) bu yoldan GELMEZ;
onu yalnizca `rc_update` yayinlar, yani RC_CHANNELS_OVERRIDE gerekir. Bu betik
onu da ayrica dener ve hangisinin geldigini raporlar.

Kullanim:
    export PATH=~/PX4-Autopilot/build/px4_sitl_default/bin:$PATH
    python3 probe_pilot_link.py
"""

from __future__ import annotations

import os
import sys
import threading
import time

import indi_sitl_common as sc

try:
    from pymavlink import mavutil
except ImportError:
    print("HATA: pymavlink yok (pip install pymavlink)")
    sys.exit(2)

MAV_ADDR = "udpin:127.0.0.1:14540"   # PX4 SITL offboard/API baglantisi
STREAM_HZ = 50.0
PROBE_S = 8.0


class StickStream:
    """MANUAL_CONTROL + GCS heartbeat akisi. Degerler [-1, 1]; z (throttle)
    MAVLink'te [0, 1000] olarak gider ve alicida 2*z-1'e cevrilir, yani
    z_norm = 0 -> 500."""

    def __init__(self, conn):
        self.conn = conn
        self.x = self.y = self.r = 0.0   # pitch, roll, yaw
        self.z = 0.0                      # throttle, [-1, 1]
        self.buttons = 0
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._t.start()

    def stop(self):
        self._stop.set()
        self._t.join(timeout=2.0)

    def set(self, x=0.0, y=0.0, z=0.0, r=0.0):
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
            self.conn.mav.manual_control_send(
                self.conn.target_system,
                int(self.x * 1000), int(self.y * 1000),
                int((self.z + 1.0) * 500.0), int(self.r * 1000),
                self.buttons)
            time.sleep(period)


def main() -> int:
    px4 = sc.Px4Client()
    out_dir = os.path.dirname(os.path.abspath(__file__))
    log_path = os.path.join(out_dir, "px4_pilot_probe.log")

    print("=== SITL baslatiliyor ===")
    sc.launch_sitl("gz_tiltrotor_indi", log_path)

    conn = None
    stream = None
    try:
        print(f"MAVLink baglaniyor: {MAV_ADDR}")
        conn = mavutil.mavlink_connection(MAV_ADDR)
        hb = conn.wait_heartbeat(timeout=30)
        if hb is None:
            print("HATA: PX4'ten heartbeat gelmedi")
            return 1
        print(f"  baglandi: sys={conn.target_system} comp={conn.target_component}")

        # --- 1) AKIS OLMADAN taban durum ---
        print("\n1) akis YOKKEN taban durum")
        report_state(px4, prefix="   ")

        # --- 2) MANUAL_CONTROL akisi ---
        print(f"\n2) MANUAL_CONTROL akisi {STREAM_HZ:.0f} Hz, {PROBE_S:.0f} s")
        stream = StickStream(conn)
        stream.start()
        time.sleep(PROBE_S)
        report_state(px4, prefix="   ")

        # --- 3) Modul gercekten gordu mu? ---
        print("\n3) modulun kendi raporu (`mc_indi_tiltrotor status`)")
        st = px4._run("mc_indi_tiltrotor", ["status"], timeout=5.0)
        for line in st.splitlines():
            if any(k in line for k in ("pilot", "setpoint", "failsafe", "back-transition")):
                print(f"   {line.strip()}")

        # --- 4) RC_CHANNELS_OVERRIDE: switches topic'i geliyor mu? ---
        print("\n4) RC_CHANNELS_OVERRIDE denemesi (manual_control_switches icin)")
        for _ in range(int(3.0 * STREAM_HZ)):
            conn.mav.rc_channels_override_send(
                conn.target_system, conn.target_component,
                1500, 1500, 1100, 1500, 1000, 1000, 1000, 1000)
            time.sleep(1.0 / STREAM_HZ)
        sw = px4._run("listener", ["manual_control_switches"], timeout=5.0)
        print("   manual_control_switches: " +
              ("YOK (never published)" if "never published" in sw or not sw.strip()
               else "VAR"))
        for line in sw.splitlines():
            if "transition_switch" in line or "mode_slot" in line:
                print(f"     {line.strip()}")

    finally:
        if stream is not None:
            stream.stop()
        if conn is not None:
            conn.close()
        sc.kill_sitl()

    print(f"\npx4 log: {log_path}")
    return 0


def report_state(px4, prefix=""):
    """Iki kosulu AYRI AYRI oku -- hangisinin eksik oldugu tek satirda gorunsun."""
    cm = px4._run("listener", ["vehicle_control_mode"], timeout=5.0)
    man = px4._run("listener", ["manual_control_setpoint"], timeout=5.0)

    def field(text, name):
        for line in text.splitlines():
            s = line.strip()
            if s.startswith(name + ":"):
                return s.split(":", 1)[1].strip()
        return "?"

    published = "never published" not in man and man.strip() != ""
    print(f"{prefix}flag_control_manual_enabled = {field(cm, 'flag_control_manual_enabled')}"
          f"   (COMMANDER verir -- asil risk burada)")
    print(f"{prefix}manual_control_setpoint     = "
          f"{'yayinlaniyor' if published else 'YOK (never published)'}"
          + (f", valid={field(man, 'valid')}, source={field(man, 'data_source')}"
             if published else ""))
    print(f"{prefix}nav_state (vehicle_status)  = "
          f"{field(px4._run('listener', ['vehicle_status'], timeout=5.0), 'nav_state')}")


if __name__ == "__main__":
    sys.exit(main())
