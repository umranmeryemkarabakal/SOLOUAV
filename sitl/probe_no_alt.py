#!/usr/bin/env python3
"""FsLevel::NO_ALT gercekten tetiklenebilir mi? -- teshis kosusu (Adim 36).

NEDEN AYRI BIR PROBE. Adim 34 seviye 2'yi iki kez TAHMINLE tetiklemeye calisti
ve ikisi de tutmadi; "muhtemelen olu dal" diye kayda gecti. Bu betik tahmin
etmiyor: EKF2'nin dikey gecerlilik mantigi once kaynaktan okundu, sonra HANGI
yardim kaynaginin ayakta kaldigi `estimator_status_flags`'tan ORNEKLENIYOR.

Kaynaktan cikan kesin kosul (EKF2.cpp:1588-1590 + ekf.h:294-302 +
ekf_helper.cpp:628-647):

    z_valid = isLocalVerticalPositionValid() || isLocalVerticalVelocityValid()
    isLocalVerticalPositionValid() = !vpos_deadreckon_exceeded && !fake_hgt
    isLocalVerticalVelocityValid() = !vvel_deadreckon_exceeded && !fake_hgt

  =>  z_valid FALSE  <=>  fake_hgt  VEYA  vvel_deadreckon_exceeded

ve `vvel_deadreckon_exceeded` ancak (a) dikey HIZ yardimi EKF2_NOAID_TOUT
boyunca kapali VE (b) dikey KONUM dead-reckon'i zaten asilmisken olusuyor.
Yani TEK BIR yuksekilik kaynagi bile ayakta kalirsa seviye 2 imkansiz.

Iki asama, artan siddette -- birincisi adim 34'un denemesi, ikincisi tam kesme:
  A) EKF2_BARO_CTRL=0, EKF2_GPS_CTRL=1  (yalnizca lon/lat; xy SAGLAM kalir --
     izole "z gitti, xy iyi" testi budur)
  B) A + EKF2_GPS_CTRL=0                (tum yardim kesilir; xy de gider, yani
     izole degil, ama dikey dalin GOVDESINI calistirir)

Kullanim:
    python3 probe_no_alt.py
"""

from __future__ import annotations

import math
import os
import sys
import time

import indi_sitl_common as sc

HOVER_ALT = 20.0
CLIMB_STAGE = 5.0
WATCH_S = 22.0        # EKF2_NOAID_TOUT = 5 s + genis pay

FLAGS = ["cs_baro_hgt", "cs_gps_hgt", "cs_rng_hgt", "cs_ev_hgt", "cs_fake_hgt", "cs_fake_pos"]


def snapshot(px4: sc.Px4Client) -> dict:
    st = px4.status()
    lp = px4.local_position()
    fl = px4.estimator_status_flags() if hasattr(px4, "estimator_status_flags") else {}
    return dict(
        fs=int(st.get("failsafe_level", -1)),
        z_valid=lp.get("z_valid"), vz_valid=lp.get("v_z_valid"),
        xy_valid=lp.get("xy_valid"), z=lp.get("z"),
        flags={k: fl.get(k) for k in FLAGS},
    )


def watch(px4: sc.Px4Client, seconds: float, yaw_sp: float, z_sp: float, label: str) -> None:
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        px4.set_setpoint(roll=0.0, pitch=0.0, yaw=yaw_sp, fx=0.0, z_sp=z_sp,
                         leso_enable=(True, True, False), pos_hold=True)
        s = snapshot(px4)
        on = [k.replace("cs_", "") for k, v in s["flags"].items() if v]
        print(f"  [{label}] t={time.monotonic() - t0:5.1f}s  fs={s['fs']}  "
              f"z_valid={s['z_valid']}  vz_valid={s['vz_valid']}  xy_valid={s['xy_valid']}  "
              f"z={s['z']}  gz={px4.gz_truth_z():6.2f}  aiding={on or '-'}")
        time.sleep(2.0)


def main() -> int:
    px4 = sc.Px4Client()
    out_dir = os.path.dirname(os.path.abspath(__file__))

    print("=== SITL baslatiliyor (NO_ALT teshis probe) ===")
    sc.launch_sitl("gz_tiltrotor_indi", os.path.join(out_dir, "px4_no_alt_probe.log"))

    try:
        if not sc.wait_until(px4.preflight_check_ok, timeout=20.0, poll_interval=0.5):
            print("UYARI: preflight temizlenmedi")

        px4.arm(force=True)
        time.sleep(0.5)
        z0 = px4.local_position().get("z", 0.0)

        yaw0 = float("nan")
        for _ in range(20):
            _, _, yaw0 = px4.attitude_euler_deg()
            if math.isfinite(yaw0):
                break
            time.sleep(0.5)
        yaw_sp = math.radians(yaw0)
        print(f"arm heading = {yaw0:.2f} deg")

        z_sp = z0
        while z_sp > z0 - HOVER_ALT + 1e-6:
            z_sp = max(z0 - HOVER_ALT, z_sp - CLIMB_STAGE)
            for _ in range(5):
                px4.set_setpoint(roll=0.0, pitch=0.0, yaw=yaw_sp, fx=0.0, z_sp=z_sp,
                                 leso_enable=(True, True, False), pos_hold=True)
                time.sleep(1.0)

        watch(px4, 6.0, yaw_sp, z_sp, "baseline")

        # ASAMA C: IZOLE dikey kayip denemesi. A ve B (ilk kosu) sunu gosterdi:
        #   A) BARO_CTRL=0 + GPS_CTRL=1  -> z_valid TRUE kaldi ama z DONDU
        #      (-20.04 sabit, arac 19.95 -> 10.15 m aliciliyordu) = sessizce yanlis
        #   B) + GPS_CTRL=0              -> fake_hgt, z_valid ANINDA false, fs=2
        #                                   ama xy de gitti -> commander sonlandirdi
        # C, ikisinin arasini hedefliyor: yukseklik REFERANSINI var olmayan bir
        # kaynaga (range finder) cevirip baro'yu da kapat, GPS lon/lat'i BIRAK.
        # Amac: hicbir yukseklik yardimi yok (=> fake_hgt) ama yatay yardim tam
        # (=> xy_valid). Bu tutarsa seviye 2 IZOLE olarak olculebilir.
        print("\n=== ASAMA C: EKF2_BARO_CTRL=0, EKF2_GPS_CTRL=1, EKF2_HGT_REF=2 (izole dikey) ===")
        px4.param_set("EKF2_BARO_CTRL", 0)
        px4.param_set("EKF2_GPS_CTRL", 1)
        px4.param_set("EKF2_HGT_REF", 2)
        watch(px4, WATCH_S, yaw_sp, z_sp, "asama-C")

        px4.disarm(force=True)
        time.sleep(2.0)

    finally:
        # ZORUNLU. PX4 SITL param'lari `rootfs/parameters.bson`'a KALICI yaziyor,
        # yani bu probe'un enjeksiyonu bir sonraki kosuya sizar ve orada
        # "Preflight Fail: Yaw estimate error" ile SITL hic acilmaz. Bir kez
        # yasandi (2026-07-30) ve nedeni bulunana kadar sonraki testi 3 launch
        # denemesi boyunca bloke etti. Elle kurtarma: rootfs/parameters*.bson sil.
        try:
            px4.param_set("EKF2_BARO_CTRL", 1)
            px4.param_set("EKF2_GPS_CTRL", 7)
            px4.param_set("EKF2_HGT_REF", 0)
            # `param save` + bekleme SART. PX4 param otosave'i GECIKMELI; ilk
            # denemede geri alma yapildi ama hemen ardindan kill_sitl() geldigi
            # icin diske HIC yazilmadi ve bson stage-C degerleriyle kaldi
            # (olculdu 2026-07-30). Yani "geri aldim" demek yetmiyor,
            # KALICILASTIRMAK gerekiyor.
            px4.param_save()
            time.sleep(2.0)
            print("param geri alindi ve kaydedildi")
        except Exception as exc:                      # noqa: BLE001
            print(f"UYARI: param geri alinamadi ({exc}) -- rootfs/parameters*.bson SILIN")
        sc.kill_sitl()

    return 0


if __name__ == "__main__":
    sys.exit(main())
