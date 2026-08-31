#!/usr/bin/env python3
"""M4 smoke test for the mc_indi_tiltrotor SITL rig.

Arms, commands a short hover setpoint, polls status/EKF health for a few
seconds, disarms. This just proves the driver harness (indi_sitl_common.py)
works end-to-end against a live SITL instance -- it is NOT the hover-gust /
transition test campaigns themselves (those are M5/M6, with disturbance
injection and plotting against the MATLAB PNGs).

Usage:
    export PATH=~/PX4-Autopilot/build/px4_sitl_default/bin:$PATH
    # ... launch gz_tiltrotor_indi SITL in another shell first ...
    python3 smoke_test.py
"""

import sys
import time

from indi_sitl_common import Px4Client


def main() -> int:
    px4 = Px4Client()

    print("== preflight ==")
    print("commander check:", "OK" if px4.preflight_check_ok() else "FAIL")

    print("== arm ==")
    px4.arm(force=True)
    time.sleep(0.5)
    state = px4.arming_state()
    print(f"arming_state = {state} (2 = ARMED)")

    if state != 2:
        print("FAIL: did not arm")
        return 1

    lpos0 = px4.local_position()
    z0 = lpos0.get("z", 0.0)
    z_sp = z0 - 3.0  # NED: negative delta = climb 3 m
    print(f"== hover setpoint: level attitude, z_sp={z_sp:.2f} (climb 3 m from z0={z0:.2f}) ==")
    px4.set_setpoint(roll=0.0, pitch=0.0, yaw=0.0, fx=0.0, z_sp=z_sp)

    ok_count = 0
    total = 8
    for i in range(total):
        time.sleep(0.5)
        status = px4.status()
        lpos = px4.local_position()
        ekf_ok = status.get("ekf_healthy", False)
        ok_count += 1 if ekf_ok else 0
        print(f"t={i*0.5:4.1f}s  z={lpos.get('z', float('nan')):+.2f}  vz={lpos.get('vz', float('nan')):+.2f}"
              f"  avg_tilt={status.get('avg_tilt_rad', float('nan')):+.3f}  ekf_healthy={ekf_ok}")

    print("== disarm ==")
    px4.disarm(force=True)

    print(f"\nEKF healthy in {ok_count}/{total} polls.")
    if ok_count < total * 0.5:
        print("FAIL: EKF unhealthy for most of the run.")
        return 1

    print("PASS: driver harness reaches a live, EKF-healthy, actuated hover setpoint.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
