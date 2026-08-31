#!/usr/bin/env python3
"""M5 -- SITL counterpart of run_hover_gust_test.m.

Hovers gz_tiltrotor_indi, then injects a slowly-varying roll disturbance
moment (ramped in at t=4s, held with a 0.3 Hz sinusoidal component,
matching run_hover_gust_test.m's ext_m(1) = ramp*(0.4+0.15*sin(2*pi*0.3*(t-4)))
exactly -- same magnitude, since the real vehicle's inertia in
TiltrotorIndiParams.hpp is the same I_XX=0.2 used in tiltrotor_params.m).
Runs twice (LESO roll+pitch enabled vs disabled), logs p/q and d_hat via
polling, reports RMS p/q in the disturbance window, and plots a comparison
figure analogous to hover_gust_test.png.

Deliberate simplification vs the MATLAB test: MATLAB also injects a wind_ned
step that disturbs pitch through its simplified aero model. No reliable way
to inject a scripted wind *velocity* was found in this Gazebo Harmonic setup
(windy.sdf has no active wind-effects plugin, only a static <wind> tag) --
disturbance injection here is the external torque only. The real Gazebo
LiftDrag aero the vehicle actually flies through is still fully unmodeled
from the controller's perspective (same "hidden aero" property the MATLAB
test exercises), it is just not scripted/ramped on a timeline. See
sitl/README.md.

Usage:
    python3 run_hover_gust_test.py
"""

from __future__ import annotations

import math
import os
import sys
import time

import indi_sitl_common as sc

CLIMB_M = 8.0          # m, altitude setpoint above spawn
SETTLE_S = 14.0         # s, wait for hover to stabilize before starting the gust clock
GUST_START = 4.0        # s, matches run_hover_gust_test.m
GUST_RAMP = 1.0         # s
GUST_BASE = 0.4         # Nm
GUST_AMPL = 0.15        # Nm
GUST_FREQ_HZ = 0.3
SCENARIO_DURATION = 12.0  # s, matches Tsim in run_hover_gust_test.m
POLL_DT = 0.2           # s


def ext_torque_x(t_since_gust: float) -> float:
    if t_since_gust < 0:
        return 0.0
    ramp = min(1.0, t_since_gust / GUST_RAMP)
    return ramp * (GUST_BASE + GUST_AMPL * math.sin(2 * math.pi * GUST_FREQ_HZ * t_since_gust))


def run_one(px4: sc.Px4Client, leso_enable: tuple[bool, bool, bool], label: str, log_path: str) -> dict:
    print(f"\n=== launching SITL for config: {label} ===")
    proc = sc.launch_sitl("gz_tiltrotor_indi", log_path)

    try:
        print("waiting for EKF yaw/tilt alignment (commander preflight check)")
        # launch_sitl() only waits for commander's "Ready for takeoff", not for
        # cs_tilt_align/cs_yaw_align to go true. Arming before those settle
        # means MulticopterIndiTiltrotor::Run() gates all real thrust (NaN
        # actuator output) until they do -- with no real thrust, PX4 never
        # detects takeoff and COM_DISARM_PRFLT (10s) auto-disarms the vehicle.
        # Polling here (disarmed, so no 10s clock running) avoids the race.
        if not sc.wait_until(px4.preflight_check_ok, timeout=20.0, poll_interval=0.5):
            print("WARNING: preflight check did not clear within 20s -- arming anyway")

        print("arm + climb")
        px4.arm(force=True)
        time.sleep(0.3)
        z0 = px4.local_position().get("z", 0.0)
        z_sp = z0 - CLIMB_M

        # vh/drift added 2026-07-29 (step 28): MANDATORY context variables -- the
        # result is only interpretable alongside the horizontal speed plateau,
        # and they are also the proof that pos_hold actually held.
        record = {"t": [], "p": [], "q": [], "r": [], "d_hat_p": [], "d_hat_q": [],
                  "ext_ax": [], "ekf_ok": [], "vh": [], "drift": []}
        pos0 = None

        t_wall0 = time.monotonic()

        # pos_hold=True (2026-07-29, step 28): this is a HOVER disturbance test,
        # and without the horizontal position loop the vehicle accelerates to
        # ~7 m/s, so every previous run of this script actually measured gust
        # rejection in CRUISE (report step 16/28). The loop owns roll/pitch and
        # supplies fx, so those arguments are ignored while it is on.
        def send_sp(fx=0.0):
            px4.set_setpoint(roll=0.0, pitch=0.0, yaw=0.0, fx=fx, z_sp=z_sp,
                             leso_enable=leso_enable, pos_hold=True)

        send_sp()

        # settle phase: keep re-sending the setpoint (test_sp is one-shot;
        # COM_DISARM_PRFLT is 10s so we must not go quiet for too long, and
        # re-sending is harmless/idempotent). Time-bounded, see the scenario
        # loop below for why (subprocess overhead >> POLL_DT).
        t_settle0 = time.monotonic()
        while time.monotonic() - t_settle0 < SETTLE_S:
            send_sp()
            time.sleep(1.0)

        print(f"settled ({SETTLE_S}s), starting gust clock")
        t_scenario0 = time.monotonic()
        i = 0

        # Bounded by real elapsed time, not a fixed iteration count --
        # subprocess-based polling (2-3 process spawns per step: listener x2
        # + gz topic) costs ~1.1-1.3s/step in practice, far more than
        # POLL_DT=0.2s nominal. A fixed-count loop at that overhead ran to
        # ~73s wall time instead of the intended ~14s (discovered on the
        # first run of this script). Looping on elapsed time keeps the
        # scenario duration close to SCENARIO_DURATION regardless of
        # per-step cost -- coarser sampling, but correct duration.
        while True:
            t = time.monotonic() - t_scenario0

            if t >= SCENARIO_DURATION:
                break

            t_gust = t - GUST_START
            tx = ext_torque_x(t_gust)

            if tx > 0:
                px4.set_persistent_torque(tx=tx)
            elif i > 0:
                px4.clear_wrench()

            omega = sc.parse_named_floats(px4.listener("vehicle_angular_velocity"))
            status = px4.status()
            lpos = px4.local_position()
            if pos0 is None:
                pos0 = (lpos.get("x", 0.0), lpos.get("y", 0.0))
            vh = math.hypot(lpos.get("vx", 0.0), lpos.get("vy", 0.0))
            drift = math.hypot(lpos.get("x", 0.0) - pos0[0], lpos.get("y", 0.0) - pos0[1])
            xyz = omega.get("xyz", [float("nan")] * 3)
            d_hat = status.get("d_hat", [float("nan")] * 3)

            record["t"].append(t)
            record["p"].append(xyz[0] if len(xyz) > 0 else float("nan"))
            record["q"].append(xyz[1] if len(xyz) > 1 else float("nan"))
            record["r"].append(xyz[2] if len(xyz) > 2 else float("nan"))
            record["d_hat_p"].append(d_hat[0] if len(d_hat) > 0 else float("nan"))
            record["d_hat_q"].append(d_hat[1] if len(d_hat) > 1 else float("nan"))
            record["ext_ax"].append(tx)
            record["ekf_ok"].append(status.get("ekf_healthy", False))
            record["vh"].append(vh)
            record["drift"].append(drift)

            i += 1

        px4.clear_wrench()
        print("scenario done, disarming")
        px4.disarm(force=True)
        time.sleep(0.5)

    finally:
        sc.kill_sitl()

    return record


def rms(vals: list[float]) -> float:
    vals = [v for v in vals if v == v]  # drop NaN
    if not vals:
        return float("nan")
    return math.sqrt(sum(v * v for v in vals) / len(vals))


def main() -> int:
    global SCENARIO_DURATION

    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=SCENARIO_DURATION,
                         help="scenario duration in seconds after settle (default: matches MATLAB Tsim=12)")
    parser.add_argument("--suffix", type=str, default="", help="suffix for output filenames, e.g. _extended")
    args = parser.parse_args()

    SCENARIO_DURATION = args.duration

    px4 = sc.Px4Client()
    out_dir = os.path.dirname(os.path.abspath(__file__))

    configs = [
        ("LESO acik (roll+pitch)", (True, True, False), os.path.join(out_dir, f"px4_m5_leso_on{args.suffix}.log")),
        ("LESO kapali", (False, False, False), os.path.join(out_dir, f"px4_m5_leso_off{args.suffix}.log")),
    ]

    results = {}
    for label, leso, log_path in configs:
        results[label] = run_one(px4, leso, label, log_path)

    print(f"\n=== RMS p/q in disturbance window (t >= 4s), scenario duration {SCENARIO_DURATION:.0f}s ===")
    for label, rec in results.items():
        idx = [i for i, t in enumerate(rec["t"]) if t >= GUST_START]
        rms_p = rms([rec["p"][i] for i in idx])
        rms_q = rms([rec["q"][i] for i in idx])
        ekf_ok_frac = sum(1 for i in idx if rec["ekf_ok"][i]) / max(1, len(idx))
        vh_w = [rec["vh"][i] for i in idx if rec["vh"][i] == rec["vh"][i]]
        dr_w = [rec["drift"][i] for i in idx if rec["drift"][i] == rec["drift"][i]]
        vh_s = f"{min(vh_w):.2f}-{max(vh_w):.2f}" if vh_w else "n/a"
        dr_s = f"{max(dr_w):.2f}" if dr_w else "n/a"
        print(f"{label:28s}  RMS p = {rms_p:.4f} rad/s   RMS q = {rms_q:.4f} rad/s"
              f"   (ekf healthy {ekf_ok_frac * 100:.0f}% of window)")
        # MANDATORY context (step 16/28): a gust result is meaningless without
        # the horizontal speed it was measured at.
        print(f"{'':28s}  v_h = {vh_s} m/s, max drift = {dr_s} m  [pos_hold ON]")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("\nmatplotlib not available -- skipping plot, raw data printed above only.")
        return 0

    fig, axes = plt.subplots(3, 1, figsize=(11, 8))
    colors = {"LESO acik (roll+pitch)": "#d93333", "LESO kapali": "#3366cc"}

    for label, rec in results.items():
        axes[0].plot(rec["t"], rec["p"], color=colors[label], label=label, linewidth=1.4)
        axes[1].plot(rec["t"], rec["q"], color=colors[label], label=label, linewidth=1.4)

    for ax, name in zip(axes[:2], ["p (roll rate)", "q (pitch rate)"]):
        ax.axvline(GUST_START, color="k", linestyle="--", linewidth=1.0)
        ax.set_ylabel(f"{name} (rad/s)")
        ax.grid(True)
        ax.legend(loc="best")
        ax.set_title(name)

    rec_on = results["LESO acik (roll+pitch)"]
    axes[2].plot(rec_on["t"], rec_on["d_hat_p"], color=colors["LESO acik (roll+pitch)"],
                 label="d_hat roll (LESO)", linewidth=1.4)
    I_XX = 0.2  # kg m^2, TiltrotorIndiParams.hpp / tiltrotor_params.m
    axes[2].plot(rec_on["t"], [ax_ / I_XX for ax_ in rec_on["ext_ax"]],
                 "k--", linewidth=1.0, label="gercek roll bozucu ivmesi (komut edilen)")
    axes[2].set_ylabel("rad/s^2")
    axes[2].set_xlabel("t (s, gust clock)")
    axes[2].grid(True)
    axes[2].legend(loc="best")
    axes[2].set_title("LESO bozucu tahmini (d_hat) vs komut edilen")

    fig.tight_layout()
    out_png = os.path.join(out_dir, f"sitl_hover_gust_test{args.suffix}.png")
    fig.savefig(out_png, dpi=120)
    print(f"\nGrafik kaydedildi: {out_png}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
