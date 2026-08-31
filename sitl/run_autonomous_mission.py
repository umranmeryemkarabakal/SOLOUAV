#!/usr/bin/env python3
"""TAM OTONOM GOREV: tek bayrak, gerisini MODUL yurutur.

NEDEN VAR (2026-08-31, Adim 154 -- madde B0).
`run_mission_test.py` gorevi PC tarafindan surer: her evreyi kendisi
zamanlar, bayraklari kendisi kaldirir, inis profilini kendisi uretirdi.
Bu, SITL'de calisir ama GERCEK KARTTA CALISMAZ -- betik hedefleri
`px4-mc_indi_tiltrotor test_sp` POSIX kabuk istemcisiyle gonderiyor ve o
yol yalnizca POSIX SITL'de var.

Bu betik onun AYNASI degil, KARSITI: yalnizca `mission_enable` bayragini
kaldirir ve IZLER. Tirmanis, ileri gecis, seyir, sabit kanat, geri gecis,
oturma ve inis -- hepsini modulun missionSequencer()'i yurutur.

Yani bu betigin yaptigi her sey, kartta bir RC anahtari ya da basit bir
companion katmani tarafindan da yapilabilir: TEK BOOL.

KULLANIM
    export PATH=~/PX4-Autopilot/build/px4_sitl_default/bin:$PATH
    INDI_SITL_GUI=1 python3 run_autonomous_mission.py
    INDI_WORLD=windy_tiltrotor python3 run_autonomous_mission.py
"""
from __future__ import annotations

import os
import sys
import time

import indi_sitl_common as sc

WORLD = os.environ.get("INDI_WORLD", "default")
TIMEOUT_S = float(os.environ.get("INDI_MSN_TIMEOUT", "420"))

MSN = {0: "IDLE", 1: "CLIMB", 2: "HOVER", 3: "FWD", 4: "CRUISE", 5: "FW",
       6: "FW_CRUISE", 7: "BACK", 8: "SETTLE", 9: "LAND", 10: "DONE"}
LAND = {0: "IDLE", 1: "DESCEND", 2: "FLARE", 3: "TOUCHDOWN"}


def main() -> int:
    out_dir = os.path.dirname(os.path.abspath(__file__))
    log_path = os.path.join(out_dir, "px4_autonomous.log")
    px4 = sc.Px4Client()
    sc.launch_sitl("gz_tiltrotor_indi", log_path, world=WORLD)

    try:
        if not sc.wait_until(px4.preflight_check_ok, timeout=20.0, poll_interval=0.5):
            print("UYARI: preflight temizlenmedi")

        px4.arm(force=True)
        time.sleep(0.5)

        def send_mission():
            # test_sp: roll pitch yaw fx z leso(3) pos_hold bt ft fw land mission
            return px4._run("mc_indi_tiltrotor",
                            ["test_sp", "0.0", "0.0", "0.0", "0.0", "0.0",
                             "1", "1", "0", "0", "0", "0", "0", "0", "1"])

        print(f"GOREV BASLADI -- tek bayrak (mission_enable), dunya: {WORLD}")
        print(f"{'t':>7} {'gorev':>10} {'inis':>10} {'agl':>7} {'v_h':>6} {'tilt':>6}")

        t0 = time.monotonic()
        prev = None
        seen = []

        while time.monotonic() - t0 < TIMEOUT_S:
            send_mission()
            st = sc.parse_named_floats(px4.listener("tiltrotor_indi_status"))
            lp = px4.local_position()
            ms = int(st.get("mission_state", -1))
            ls = int(st.get("land_state", -1))
            ua, du = st.get("u_actual", []), st.get("du", [])
            tilt = 0.0

            if len(ua) > 3 and len(du) > 3:
                tilt = (float(ua[3]) + float(du[3])) * 57.2958

            vh = (lp.get("vx", 0.0) ** 2 + lp.get("vy", 0.0) ** 2) ** 0.5
            agl = -lp.get("z", 0.0)

            if ms != prev:
                t = time.monotonic() - t0
                print(f"{t:7.1f} {MSN.get(ms,'?'):>10} {LAND.get(ls,'?'):>10} "
                      f"{agl:7.2f} {vh:6.2f} {tilt:6.1f}")
                seen.append(MSN.get(ms, "?"))
                prev = ms

            if ms == 10:      # DONE
                break

            time.sleep(1.0)

        t_end = time.monotonic() - t0
        print(f"\nGECILEN EVRELER: {' -> '.join(seen)}")

        # GECME OLCUTU: gorev DONE'a ulasti ve butun evrelerden gecti.
        need = ["CLIMB", "HOVER", "FWD", "CRUISE", "BACK", "SETTLE", "LAND", "DONE"]
        missing = [p for p in need if p not in seen]
        ok = not missing

        print(f"sure: {t_end:.0f} s")

        if missing:
            print(f"SONUC: KALDI -- ulasilmayan evre: {', '.join(missing)}")
        else:
            print("SONUC: GECTI -- tam gorev MODUL tarafindan yurutuldu")

        px4.disarm(force=True)
        time.sleep(2.0)
        return 0 if ok else 1

    finally:
        sc.kill_sitl()


if __name__ == "__main__":
    sys.exit(main())
