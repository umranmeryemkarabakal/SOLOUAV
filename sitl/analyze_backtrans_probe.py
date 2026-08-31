#!/usr/bin/env python3
"""Adim 31 / Faz 0 -- run_backtrans_probe.py'nin ulog'unu coz: seyirde
fx_sp = 0 iken tilt'i ILERI suren talep HANGISI?

Yontem: WLS tahsisati her ornek icin CEVRIMDISI YENIDEN COZULUR
(TiltrotorIndiControl.hpp'deki effectivenessMatrix + wlsAllocate birebir
port edilerek, log'lanan u_actual ve nu_des girdi olarak kullanilarak).
Once model dogrulanir (cozulen du, log'lanan du ile karsilastirilir); sonra
nu_des'in tek tek kanallari SIFIRLANIP tilt artiminin nasil degistigine
bakilir. Bu bir korelasyon degil, dogrudan bir ATIF: "Fz talebi olmasaydi
tilt ne yapardi?" sorusunun tam cevabi.

Bu, projenin kendi dersini uygular (Adim 21d): bir ortam ancak hedeflenen
mekanizma orada AKTIFSE bir sey kanitlar -- burada mekanizma tahsisatin
kendisinde, o yuzden tahsisat yeniden cozuluyor.

Kullanim:
    python3 analyze_backtrans_probe.py <ulog> [<ulog2> ...]
"""

from __future__ import annotations

import math
import sys

import numpy as np
from pyulog import ULog

# --- TiltrotorIndiParams.hpp ile SENKRON OLMALI ---
ROTOR_PX = np.array([0.27, 0.27, -0.55])
ROTOR_PY = np.array([0.35, -0.35, 0.00])
ROTOR_PZ = np.array([0.06, 0.06, -0.07])
ROTOR_KM = np.array([-0.06, 0.06, -0.06])
ROTOR_TMAX = 45.0
ROTOR_TMIN = 0.0
ROTOR_TAU_UP = 0.0125
ROTOR_TAU_DOWN = 0.0250
TILT_MIN = 0.0
TILT_MAX = math.pi / 2
TILT_TAU = 0.15
TS_BOX = 1.0 / 250.0
TILT_SLEW_BOX_RATE = 3.00
WS = np.array([200.0, 200.0, 3.0, 0.05, 20.0])   # roll pitch yaw Fx Fz
WU_TILT_HOVER = 3.0
WU_TILT_CRUISE = 1.5
WU_TAIL_PENALTY = 3.0
BIG_M = 1e6

CH = ["taux", "tauy", "tauz", "Fx", "Fz"]


def effectiveness(u):
    """u = [T0 T1 T2 d0 d1 d2] -> (G 5x6, nu0 5)"""
    G = np.zeros((5, 6))
    nu0 = np.zeros(5)
    for i in range(3):
        T, de = u[i], u[3 + i]
        s, c = math.sin(de), math.cos(de)
        dir_ = np.array([s, 0.0, -c])
        ddir = np.array([c, 0.0, s])
        r = np.array([ROTOR_PX[i], ROTOR_PY[i], ROTOR_PZ[i]])
        km = ROTOR_KM[i]

        f = dir_ * T
        m = dir_ * (km * T)
        tau = np.cross(r, f) + m

        dtau_dT = np.cross(r, dir_) + dir_ * km
        dtau_dd = (np.cross(r, ddir) + ddir * km) * T

        nu0[0:3] += tau
        nu0[3] += f[0]
        nu0[4] += f[2]

        G[0:3, i] = dtau_dT
        G[0:3, 3 + i] = dtau_dd
        G[3, i] = dir_[0]
        G[3, 3 + i] = T * ddir[0]
        G[4, i] = dir_[2]
        G[4, 3 + i] = T * ddir[2]
    return G, nu0


def box(u):
    du_min = np.zeros(6)
    du_max = np.zeros(6)
    for i in range(3):
        rate_lo = -ROTOR_TMAX / ROTOR_TAU_UP * TS_BOX * 5.0
        rate_hi = ROTOR_TMAX / ROTOR_TAU_DOWN * TS_BOX * 5.0
        du_min[i] = max(ROTOR_TMIN - u[i], rate_lo)
        du_max[i] = min(ROTOR_TMAX - u[i], rate_hi)
    for i in range(3):
        rate = TILT_SLEW_BOX_RATE * TS_BOX
        du_min[3 + i] = max(TILT_MIN - u[3 + i], -rate)
        du_max[3 + i] = min(TILT_MAX - u[3 + i], rate)
    du_min = np.minimum(du_min, du_max)
    return du_min, du_max


def wls(G, nu_des, du_min, du_max, Wu):
    """wlsAllocate() birebir portu (big-M cezali, 6 iterasyon)."""
    Ws2 = WS ** 2
    Wu_eff = Wu.copy()
    du_pref = np.zeros(6)
    du = np.zeros(6)
    for _ in range(6):
        H = G.T @ (Ws2[:, None] * G) + np.diag(Wu_eff ** 2)
        rhs = G.T @ (Ws2 * nu_des) + (Wu_eff ** 2) * du_pref
        du = np.linalg.solve(H, rhs)
        viol = False
        for i in range(6):
            if du[i] > du_max[i]:
                du_pref[i] = du_max[i]; Wu_eff[i] = BIG_M; viol = True
            elif du[i] < du_min[i]:
                du_pref[i] = du_min[i]; Wu_eff[i] = BIG_M; viol = True
        if not viol:
            break
    return np.clip(du, du_min, du_max)


def wu_from_smooth(w):
    wu = WU_TILT_HOVER + (WU_TILT_CRUISE - WU_TILT_HOVER) * w
    return np.array([1.0, 1.0, 1.0, wu, wu, wu * WU_TAIL_PENALTY])


def load(path):
    u = ULog(path, ["tiltrotor_indi_status", "tiltrotor_indi_setpoint", "vehicle_local_position"])
    d = {x.name: x.data for x in u.data_list}
    if "tiltrotor_indi_status" not in d:
        raise SystemExit(f"{path}: tiltrotor_indi_status yok -- logger_topics profili acik miydi?")
    return d


def observation_window(d):
    """fx_sp'nin son kez >0'dan 0'a dustugu ani bul; pencere oradan log sonuna."""
    sp = d.get("tiltrotor_indi_setpoint")
    st = d["tiltrotor_indi_status"]
    if sp is None:
        return st["timestamp"][0], st["timestamp"][-1]
    t = np.asarray(sp["timestamp"], dtype=float)
    fx = np.asarray(sp["fx_sp"], dtype=float)
    hi = np.where(fx > 1.0)[0]
    if len(hi) == 0:
        return st["timestamp"][0], st["timestamp"][-1]
    t0 = t[hi[-1]]
    # disarm sonrasini disla: son 3 s at
    return t0 + 1.0e6, st["timestamp"][-1] - 3.0e6


def analyze(path):
    d = load(path)
    st = d["tiltrotor_indi_status"]
    t = np.asarray(st["timestamp"], dtype=float)

    # Olcum tuzagi #3 (Adim 18): ~%1.5 yinelenen timestamp var, ayikla.
    keep = np.concatenate(([True], np.diff(t) > 1e-6))
    idx_all = np.where(keep)[0]

    t0, t1 = observation_window(d)
    sel = idx_all[(t[idx_all] >= t0) & (t[idx_all] <= t1)]
    if len(sel) < 50:
        raise SystemExit(f"{path}: gozlem penceresi cok kisa ({len(sel)} ornek)")

    u_act = np.column_stack([st[f"u_actual[{i}]"] for i in range(6)])
    nu = np.column_stack([st[f"nu_des[{i}]"] for i in range(5)])
    du_log = np.column_stack([st[f"du[{i}]"] for i in range(6)])
    smooth = np.asarray(st["gain_schedule_smooth"], dtype=float)

    dur = (t[sel[-1]] - t[sel[0]]) / 1e6
    d0 = math.degrees(u_act[sel[0], 3]); d1 = math.degrees(u_act[sel[0], 4]); d2 = math.degrees(u_act[sel[0], 5])
    e0 = math.degrees(u_act[sel[-1], 3]); e1 = math.degrees(u_act[sel[-1], 4]); e2 = math.degrees(u_act[sel[-1], 5])

    lp = d.get("vehicle_local_position")
    v_txt = ""
    if lp is not None:
        tl = np.asarray(lp["timestamp"], dtype=float)
        vh = np.hypot(np.asarray(lp["vx"], dtype=float), np.asarray(lp["vy"], dtype=float))
        m = (tl >= t[sel[0]]) & (tl <= t[sel[-1]])
        if m.any():
            v_txt = f"  v_h {vh[m][0]:.1f} -> {vh[m][-1]:.1f} m/s"

    print(f"\n{'=' * 78}\n{path}")
    print(f"Gozlem penceresi: {dur:.1f} s, {len(sel)} ornek{v_txt}")
    print(f"tilt d0 {d0:.1f} -> {e0:.1f} deg | d1 {d1:.1f} -> {e1:.1f} | d2 {d2:.1f} -> {e2:.1f}")
    print(f"OLCULEN kanat tilt surukleme hizi: {((e0 - d0) + (e1 - d1)) / 2 / dur:+.3f} deg/s")
    print(f"nu_des ortalamalari: " + "  ".join(
        f"{CH[k]}={np.mean(nu[sel, k]):+.3f}" for k in range(5)))

    # --- her ~10 Hz'de bir ornekle (25 tick) ---
    step = max(1, len(sel) // 350)
    sub = sel[::step]

    ablations = {
        "baseline (tum talepler)": None,
        "Fz TALEBI SIFIR": 4,
        "Fx talebi sifir": 3,
        "tau_y (pitch) sifir": 1,
        "tau_x (roll) sifir": 0,
        "tau_z (yaw) sifir": 2,
    }
    acc = {k: [] for k in ablations}
    acc["yalniz Fz talebi"] = []
    du_model = []

    for j in sub:
        u = u_act[j]
        G, _nu0 = effectiveness(u)
        lo, hi = box(u)
        Wu = wu_from_smooth(smooth[j])

        for name, zero_ch in ablations.items():
            n = nu[j].copy()
            if zero_ch is not None:
                n[zero_ch] = 0.0
            du = wls(G, n, lo, hi, Wu)
            acc[name].append(du[3:5].mean())
            if name == "baseline (tum talepler)":
                du_model.append(du)

        n = np.zeros(5); n[4] = nu[j, 4]
        acc["yalniz Fz talebi"].append(wls(G, n, lo, hi, Wu)[3:5].mean())

    du_model = np.array(du_model)

    # --- model dogrulamasi: cevrimdisi cozum log'lanan du'yu yeniden uretiyor mu? ---
    dl = du_log[sub]
    rms = np.sqrt(np.mean((du_model - dl) ** 2, axis=0))
    print(f"\nMODEL DOGRULAMASI (cevrimdisi WLS vs log'lanan du), RMS:")
    print(f"  itki  {rms[0]:.3e} {rms[1]:.3e} {rms[2]:.3e} N   "
          f"tilt {rms[3]:.3e} {rms[4]:.3e} {rms[5]:.3e} rad")
    tilt_scale = np.sqrt(np.mean(dl[:, 3:6] ** 2))
    print(f"  tilt kanali RMS hata / sinyal RMS = {np.sqrt(np.mean(rms[3:6]**2)) / tilt_scale * 100:.2f}%")

    # --- atif ---
    print(f"\nATIF -- kanat tilt artimi du (ortalama) ve bunun ima ettigi surukleme hizi")
    print(f"  (ddelta = du/TILT_TAU; + = ILERI/cruise yonu)\n")
    print(f"  {'senaryo':<26}{'du (rad)':>12}{'ddelta (deg/s)':>18}")
    print(f"  {'-' * 56}")
    base = np.mean(acc["baseline (tum talepler)"])
    for name in list(ablations) + ["yalniz Fz talebi"]:
        v = np.mean(acc[name])
        print(f"  {name:<26}{v:>12.3e}{math.degrees(v / TILT_TAU):>18.3f}")
    print(f"  {'-' * 56}")

    fz_off = np.mean(acc["Fz TALEBI SIFIR"])
    print(f"\n  Fz talebi kaldirilinca surukleme {math.degrees(base / TILT_TAU):+.3f} -> "
          f"{math.degrees(fz_off / TILT_TAU):+.3f} deg/s "
          f"({'ISARET DEGISTIRDI' if base * fz_off < 0 else 'isaret ayni'})")
    return base, fz_off


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    for p in sys.argv[1:]:
        analyze(p)
    return 0


if __name__ == "__main__":
    sys.exit(main())
