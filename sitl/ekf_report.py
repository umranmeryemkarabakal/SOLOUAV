#!/usr/bin/env python3
"""M7 -- EKF2 health report over the M4/M5/M6 SITL test flights.

process_logdata_ekf.py (Tools/ecl_ekf/) refuses to analyse these logs
("InAirDetector: always on ground" / "no airtime detected") because its
takeoff/landing heuristic depends on modules we deliberately stopped for
this airframe (mc_pos_control et al, see the 4023_gz_tiltrotor_indi.post
rc script) -- land_detector never gets the inputs it normally expects, so
it never flags "in air" even though the vehicle physically flew. This
script bypasses that heuristic and reports directly over the whole armed
window instead, using estimator_status / estimator_status_flags exactly as
described in the M7 plan (commander check / ekf2 status cross-check is
manual, see sitl/README.md).

Usage:
    python3 ekf_report.py <file1.ulg> [file2.ulg ...]
"""

from __future__ import annotations

import sys

from pyulog import ULog

# Sticky/structural flags: expected true for the whole armed window once set.
STICKY_OK_FLAGS = ["cs_tilt_align", "cs_yaw_align"]

# Flags that indicate the filter is coasting on assumed/fake data rather than
# real measurements -- bad if seen while armed and expected to be flying.
COASTING_FLAGS = ["cs_fake_pos", "cs_fake_hgt", "cs_inertial_dead_reckoning", "cs_wind_dead_reckoning"]

# Fusion-fault pulses (see MulticopterIndiTiltrotor.cpp ekfHealthy() comment --
# these can flicker transiently even in healthy flight; reported here as a
# time-fraction, not a hard pass/fail).
FS_BAD_PREFIX = "fs_bad_"

TEST_RATIOS = ["mag_test_ratio", "vel_test_ratio", "pos_test_ratio", "hgt_test_ratio",
               "tas_test_ratio", "hagl_test_ratio", "beta_test_ratio"]

PRE_FLT_FAIL = ["pre_flt_fail_innov_heading", "pre_flt_fail_innov_vel_horiz",
                "pre_flt_fail_innov_vel_vert", "pre_flt_fail_innov_height",
                "pre_flt_fail_mag_field_disturbed"]


def frac_true(values) -> float:
    if len(values) == 0:
        return float("nan")
    return sum(1 for v in values if v) / len(values)


def report_one(path: str) -> dict:
    u = ULog(path)
    duration_s = u.last_timestamp / 1e6 if u.last_timestamp else float("nan")

    flags = u.get_dataset("estimator_status_flags")
    status = u.get_dataset("estimator_status")

    result = {"path": path, "duration_s": duration_s}

    for f in STICKY_OK_FLAGS:
        result[f] = frac_true(flags.data[f])

    for f in COASTING_FLAGS:
        result[f] = frac_true(flags.data[f])

    fs_bad_fields = [k for k in flags.data.keys() if k.startswith(FS_BAD_PREFIX)]
    fs_bad_any = [any(flags.data[k][i] for k in fs_bad_fields) for i in range(len(flags.data[fs_bad_fields[0]]))]
    result["fs_bad_any_frac"] = frac_true(fs_bad_any)

    for f in PRE_FLT_FAIL:
        if f in status.data:
            result[f] = frac_true(status.data[f])

    for f in TEST_RATIOS:
        if f in status.data:
            vals = status.data[f]
            result[f"{f}_max"] = max(vals) if len(vals) else float("nan")

    return result


def print_report(r: dict) -> None:
    name = r["path"].split("/")[-1]
    print(f"\n=== {name} ({r['duration_s']:.1f}s logged) ===")
    print("Sticky alignment flags (should be ~100% once armed+flying):")
    for f in STICKY_OK_FLAGS:
        print(f"  {f:30s} {r[f] * 100:5.1f}%")
    print("Coasting-on-fake-data flags (should be ~0%):")
    for f in COASTING_FLAGS:
        flag = " <-- NONZERO" if r[f] > 0.01 else ""
        print(f"  {f:30s} {r[f] * 100:5.1f}%{flag}")
    print(f"fs_bad_* (any fusion-fault pulse set): {r['fs_bad_any_frac'] * 100:5.1f}% of samples")
    print("Preflight-fail flags:")
    for f in PRE_FLT_FAIL:
        if f in r:
            flag = " <-- NONZERO" if r[f] > 0.01 else ""
            print(f"  {f:35s} {r[f] * 100:5.1f}%{flag}")
    print("Innovation test ratios (max over flight, <1 = pass):")
    for f in TEST_RATIOS:
        key = f"{f}_max"
        if key in r:
            flag = " <-- FAIL" if r[key] >= 1.0 else ""
            print(f"  {f:20s} max={r[key]:.3f}{flag}")


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 1

    all_results = []
    for path in sys.argv[1:]:
        try:
            r = report_one(path)
        except Exception as e:
            print(f"\n=== {path}: FAILED TO ANALYSE ({e}) ===")
            continue
        all_results.append(r)
        print_report(r)

    print(f"\n\n=== SUMMARY across {len(all_results)} flights ===")
    any_bad = False
    for r in all_results:
        name = r["path"].split("/")[-1]
        problems = []
        if r["cs_tilt_align"] < 0.99 or r["cs_yaw_align"] < 0.99:
            problems.append("alignment not sustained")
        for f in COASTING_FLAGS:
            if r[f] > 0.01:
                problems.append(f"{f}={r[f] * 100:.0f}%")
        for f in PRE_FLT_FAIL:
            if r.get(f, 0) > 0.01:
                problems.append(f"{f}={r[f] * 100:.0f}%")
        for f in TEST_RATIOS:
            key = f"{f}_max"
            if r.get(key, 0) >= 1.0:
                problems.append(f"{f}_max={r[key]:.2f}")
        status_str = "CLEAN" if not problems else "ISSUES: " + ", ".join(problems)
        if problems:
            any_bad = True
        print(f"  {name:20s} {status_str}")

    print("\nAll clean." if not any_bad else "\nSome flights show issues -- see above.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
