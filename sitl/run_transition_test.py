#!/usr/bin/env python3
"""M6 -- SITL counterpart of run_transition_test.m.

Hovers gz_tiltrotor_indi, then ramps a body-x forward-force setpoint
(Fx_sp: 0 -> 10 N over 12 s, matching run_transition_test.m's Fx_final/
t_ramp exactly) with no disturbance. No separate transition state machine
is involved (see the plan/design notes) -- WLS finds tilting the rotors
forward is the only way to produce Fx (a vertical rotor contributes zero
Fx) purely from the allocation math, and gain_schedule.m smoothly
interpolates INDI gains / WLS tilt-preference weights as the measured
mean tilt grows. This script shows that same behaviour running for real
in SITL against the real Gazebo aero/actuator dynamics, instead of the
idealized MATLAB plant.

Usage:
    python3 run_transition_test.py
"""

from __future__ import annotations

import math
import os
import sys
import time

import indi_sitl_common as sc

CLIMB_M = 8.0
SETTLE_S = 14.0
FX_FINAL = 10.0     # N, matches Fx_final in run_transition_test.m
T_RAMP = 12.0        # s, matches t_ramp
SCENARIO_DURATION = 14.0  # s, matches Tsim
MASS = 5.0           # kg, TiltrotorIndiParams.hpp / tiltrotor_params.m

# gain_schedule.m constants, reconstructed here from gain_schedule_smooth (0=hover..1=cruise)
KP_RATE_HOVER = (4.0, 4.0, 2.0)
KP_RATE_CRUISE = (3.0, 3.0, 1.5)
WU_TILT_HOVER = 3.0  # TiltrotorIndiParams.hpp / gain_schedule.m (2026-07-26 T0-lockup fix)
WU_TILT_CRUISE = 1.5
WU_TAIL_PENALTY = 3.0


def fx_sp_at(t: float) -> float:
    return FX_FINAL * min(1.0, t / T_RAMP)


def kp_rate_from_smooth(w: float) -> tuple[float, float, float]:
    return tuple(h + (c - h) * w for h, c in zip(KP_RATE_HOVER, KP_RATE_CRUISE))


def wu_tilt_from_smooth(w: float) -> tuple[float, float]:
    wu = WU_TILT_HOVER + (WU_TILT_CRUISE - WU_TILT_HOVER) * w
    return (wu, wu * WU_TAIL_PENALTY)


def main() -> int:
    px4 = sc.Px4Client()
    out_dir = os.path.dirname(os.path.abspath(__file__))
    log_path = os.path.join(out_dir, "px4_m6_transition.log")

    print("=== launching SITL ===")
    sc.launch_sitl("gz_tiltrotor_indi", log_path)

    record = {"t": [], "delta": [[], [], []], "T": [[], [], []], "vx": [], "fx_sp": [],
              "roll": [], "pitch": [], "yaw": [], "smooth": [], "omega_norm": [], "ekf_ok": []}

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

        # pos_hold during the settle phase, released for the ramp (2026-07-29,
        # step 28). This is the transition a real mission actually flies:
        # station-keeping hover -> release -> accelerate. It is also the only
        # way to settle in TRUE hover; without the position loop the vehicle is
        # already doing ~7 m/s by the time the ramp starts, so the "transition"
        # would begin from cruise (report step 16/28).
        #
        # The HANDOFF is the new thing under test: while pos_hold is on the
        # loop owns roll/pitch and supplies fx (the item (P) trim); when it is
        # released both revert to the values in this message, so a step is
        # possible. attitude/rate transients around t=0 are what to watch.
        def send_sp(fx=0.0, pos_hold=False):
            px4.set_setpoint(roll=0.0, pitch=0.0, yaw=0.0, fx=fx, z_sp=z_sp,
                             leso_enable=(True, True, False), pos_hold=pos_hold)

        send_sp(pos_hold=True)
        t_settle0 = time.monotonic()
        while time.monotonic() - t_settle0 < SETTLE_S:
            send_sp(pos_hold=True)
            time.sleep(1.0)

        pre = px4.local_position()
        print(f"settled in POSITION HOLD ({SETTLE_S}s): "
              f"v_h = {math.hypot(pre.get('vx', 0.0), pre.get('vy', 0.0)):.2f} m/s")
        print(f"releasing pos_hold and starting Fx ramp (0 -> {FX_FINAL} N over {T_RAMP}s)")
        z_start = px4.local_position().get("z", z0)
        t_scenario0 = time.monotonic()

        while True:
            t = time.monotonic() - t_scenario0
            if t >= SCENARIO_DURATION:
                break

            fx = fx_sp_at(t)
            send_sp(fx=fx)

            status = px4.status()
            lpos = px4.local_position()
            roll, pitch, yaw = px4.attitude_euler_deg()
            omega = sc.parse_named_floats(px4.listener("vehicle_angular_velocity"))

            u_actual = status.get("u_actual", [float("nan")] * 6)
            xyz = omega.get("xyz", [float("nan")] * 3)

            record["t"].append(t)
            for i in range(3):
                record["delta"][i].append(math.degrees(u_actual[3 + i]) if len(u_actual) == 6 else float("nan"))
                record["T"][i].append(u_actual[i] if len(u_actual) == 6 else float("nan"))
            record["vx"].append(lpos.get("vx", float("nan")))
            record["fx_sp"].append(fx)
            record["roll"].append(roll)
            record["pitch"].append(pitch)
            record["yaw"].append(yaw)
            record["smooth"].append(status.get("gain_schedule_smooth", float("nan")))
            record["omega_norm"].append(math.sqrt(sum(v * v for v in xyz if v == v)))
            record["ekf_ok"].append(status.get("ekf_healthy", False))

        print("scenario done, disarming")
        z_end = px4.local_position().get("z", z_start)
        px4.disarm(force=True)
        time.sleep(0.5)

    finally:
        sc.kill_sitl()

    avg_tilt_end_deg = sum(record["delta"][i][-1] for i in range(3)) / 3.0
    vx_end = record["vx"][-1]
    alt_change = -(z_end - z_start)  # NED: more negative z = higher
    max_omega = max(record["omega_norm"]) if record["omega_norm"] else float("nan")
    ekf_ok_frac = sum(record["ekf_ok"]) / max(1, len(record["ekf_ok"]))

    print(f"\nSon ortalama tilt: {avg_tilt_end_deg:.1f} deg,  ileri hiz (NED vx): {vx_end:.2f} m/s,"
          f"  irtifa degisimi: {alt_change:.2f} m")
    print(f"Senaryo boyunca max |omega| = {max_omega:.4f} rad/s (kararlilik kontrolu)")
    print(f"EKF healthy: {ekf_ok_frac * 100:.0f}% of scenario")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("\nmatplotlib not available -- skipping plot.")
        return 0

    fig, axes = plt.subplots(3, 2, figsize=(12, 10))
    t = record["t"]

    ax = axes[0][0]
    for i, name in enumerate(["d0 (sag kanat)", "d1 (sol kanat)", "d2 (kuyruk)"]):
        ax.plot(t, record["delta"][i], linewidth=1.3, label=name)
    ax.set_ylabel("tilt (deg)"); ax.legend(loc="best"); ax.grid(True)
    ax.set_title("Rotor tilt acilari (WLS kendiliginden secti)")

    ax = axes[0][1]
    for i, name in enumerate(["T0", "T1", "T2"]):
        ax.plot(t, record["T"][i], linewidth=1.1, label=name)
    ax.set_ylabel("itki (N)"); ax.legend(loc="best"); ax.grid(True)
    ax.set_title("Rotor itkileri")

    ax = axes[1][0]
    ax.plot(t, record["vx"], linewidth=1.3, label="vx (NED, ileri hiz yaklasik degeri)")
    ax.plot(t, [fx / MASS for fx in record["fx_sp"]], "k--", label="Fx_sp/m (referans ivme)")
    ax.set_ylabel("m/s"); ax.legend(loc="best"); ax.grid(True)
    ax.set_title("Ileri hiz (NED vx, govde-ekseni degil -- bkz. dosya notu)")

    ax = axes[1][1]
    ax.plot(t, record["roll"], linewidth=1.1, label="roll")
    ax.plot(t, record["pitch"], linewidth=1.1, label="pitch")
    ax.plot(t, record["yaw"], linewidth=1.1, label="yaw")
    ax.set_ylabel("deg"); ax.legend(loc="best"); ax.grid(True)
    ax.set_title("Attitude (referans = 0)")

    ax = axes[2][0]
    kp_rate = [kp_rate_from_smooth(w) for w in record["smooth"]]
    ax.plot(t, [k[0] for k in kp_rate], linewidth=1.3, label="Kp_rate,roll")
    ax.plot(t, [k[2] for k in kp_rate], linewidth=1.3, label="Kp_rate,yaw")
    ax2 = ax.twinx()
    avg_tilt_deg = [sum(record["delta"][i][j] for i in range(3)) / 3.0 for j in range(len(t))]
    ax2.plot(t, avg_tilt_deg, "k--", linewidth=1.3, label="ortalama tilt (deg)")
    ax.set_ylabel("Kp_rate"); ax2.set_ylabel("tilt (deg)"); ax.set_xlabel("t (s)")
    ax.grid(True); ax.legend(loc="upper left"); ax2.legend(loc="upper right")
    ax.set_title("Gain-scheduling: Kp_rate vs ortalama tilt")

    ax = axes[2][1]
    wu = [wu_tilt_from_smooth(w) for w in record["smooth"]]
    ax.plot(t, [w[0] for w in wu], linewidth=1.3, label="Wu_tilt,kanat")
    ax.plot(t, [w[1] for w in wu], linewidth=1.3, label="Wu_tilt,kuyruk")
    ax.set_ylabel("Wu agirligi"); ax.set_xlabel("t (s)"); ax.legend(loc="best"); ax.grid(True)
    ax.set_title("Gain-scheduling: WLS tilt tercih agirligi")

    fig.tight_layout()
    out_png = os.path.join(out_dir, "sitl_transition_test.png")
    fig.savefig(out_png, dpi=120)
    print(f"\nGrafik kaydedildi: {out_png}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
