/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "MulticopterIndiTiltrotor.hpp"

// GOLGE CODEGEN (Adim 131). MATLAB Coder'in urettigi WLS tahsisati. UCUSU
// SURMEZ: her tikte C++ wlsAllocate'in yaninda kosar ve farki loglar.
// NEDEN: Adim 130 uretilen kodu gercek ucus verisiyle karsilastirdi, ama
// CEVRIMDISI (kaydedilmis girdileri tekrar oynatarak). Bu, ayni olcumu
// GERCEK ucus dongusunde yapar -- 250 Hz'de, kapali cevrimin kendi urettigi
// girdilerle, ve gercek zaman butcesi altinda.
extern "C" {
#include "codegen/sf_wls_alloc.h"
}

#include <cstdlib>
#include <drivers/drv_hrt.h>
#include <mathlib/math/Limits.hpp>
#include <uORB/topics/vehicle_attitude.h>

using namespace tiltrotor_indi;

// --- TEST HOOK (2026-07-28, step 23): runtime override for TILT_SLEW_BOX_RATE ---
//
// Lets the allocator's tilt slew box be swept WITHIN a single flight. This is not
// convenience, it is required by this project's own method lesson: the airframe is
// MARGINALLY STABLE in yaw at low speed (step 20 -- the identical configuration
// decayed after ~35 s in one flight and oscillated undecayed for 112 s in another),
// so comparing separate flights confounds the sweep with flight-to-flight variance.
// Changing the value mid-flight keeps everything else fixed.
//
// Defaults to TILT_SLEW_BOX_RATE, so behaviour is unchanged unless explicitly set
// via the `slewbox` custom command. The active value is also recoverable from any
// log without extra fields: the tilt box is exactly rate * TS_BOX, so
// |du(3..5)| p99 reads it back directly.
//
// Held as an integer in millirad/s: px4::atomic is built on __atomic_load_n,
// which GCC does not accept for float. Reading back as x/1000.0f is exact for the
// default (1250/1000 = 1.25f), so neutrality is preserved bit-for-bit.
static px4::atomic<int32_t> _slew_box_rate_mrs{(int32_t)(TILT_SLEW_BOX_RATE * 1000.0f)};

// --- TILT JERK LIMIT (2026-08-25, step 94/95) ---
//
// Adim 94 measured, from Gazebo's OWN joint_state (real tilt angle/rate, not
// PX4's commanded value), that swinging the tilt RATE from one box extreme to
// the other in a single tick -- which `slewbox`'s box alone does not forbid,
// and which was measured happening tick-to-tick during the pos_hold+climb
// pitch/yaw instability (Adim 90-93) -- produces a D'Alembert reaction torque
// (from angularly accelerating the real motor+rotor mass, I_tilt*ddelta_dot)
// comparable to or LARGER than the entire commanded pitch moment (RMS ratio
// 1.13-1.16x). Neither tiltrotor_plant_deriv.m nor this allocator's own
// Jacobian model that term at all -- Gazebo's full multibody physics applies
// it regardless, so the controller is fighting a real torque it does not
// know exists, which can plausibly self-reinforce into the observed
// limit-cycle (the controller's own correction attempt IS the disturbance).
//
// This does NOT reduce the tilt RATE box (`slewbox` above) -- doing so was
// tried for a different reason in step 18-27 and lowering it re-opened the
// allocator-starvation failure mode ((Q)) that step 27 fixed by RAISING it.
// Instead this bounds how fast the RATE ITSELF may change tick-to-tick (one
// derivative higher), which specifically kills the "+box one tick, -box the
// next" pattern without shrinking the box's total authority for a smooth,
// sustained rate demand reached gradually over several ticks.
//
// DEPLOYED 0.3 rad/s (2026-08-25/26, step 96). Landed neutral (2x slewbox)
// first per the step-22 discipline, then characterised SAFELY in MATLAB
// (`run_poshold_climb_jerk_sweep.m`, after step 96 added the same missing
// I_tilt*ddelta_dot reaction physics to tiltrotor_plant_deriv.m -- the
// augmented plant's own baseline, at TAU_DIFF matched to SITL's 4ms tick,
// reproduces a comparable pitch p2p/yaw-rate severity to the real SITL
// instability) BEFORE any further live SITL sweep, because a raw live sweep
// (step 95) was NOT safe here: 0.8 rad/s fully damped pitch but broke
// position-hold (0.36m -> 866m drift over 90s); 1.5 rad/s was WORSE than
// no limit at all (r to -3.9 rad/s). The MATLAB sweep found 0.25-0.30 as a
// broad, non-catastrophic sweet spot on pitch p2p / yaw-rate RMS / drift
// simultaneously. Both ends of that range were then confirmed live in SITL
// (>=45-150s each, clean world verified): 0.30 measured BETTER than 0.25 in
// the real system (yaw settles to a few deg and STAYS there, position holds
// within <=0.34 m of its settle point, pitch bounded 5-11 deg -- no longer
// growing/divergent, though not fully damped either) -- trust the live
// measurement over the model's predicted optimum where they disagree.
// Full data: WLS_LOCKUP_INVESTIGATION_REPORT.md step 95-96.
// 0.30 -> 0.45 rad/s (2026-08-30, adim 134). CANLI SUPURME, tek ucusta,
// hover'da pitch salinimi olculerek:
//   0.20 -> std 3.24, tepe-tepe 13.71 deg
//   0.30 -> std 2.99, tepe-tepe 11.34 deg   <- eski varsayilan
//   0.45 -> std 0.90, tepe-tepe  3.75 deg   <- SECILEN
//   0.60 -> std 0.93, tepe-tepe  4.61 deg
//   1.00 -> std 1.33, tepe-tepe  6.39 deg
// Ikinci, ince supurme 0.40/0.45/0.50/0.55 = 5.22/3.51/3.97/4.56 verdi, yani
// optimum 0.45-0.50 bandinda ve kenarlari daha kotu -- tek noktali bir tesadüf
// degil, bir MINIMUM.
// TEKRARLANABILIRLIK, durustce: 0.45 ayni ucusta ikinci kez olculdugunde 6.65
// deg verdi (ilk 3.51). Yani dagilim var ve tek bir kosum kesin degil; ama
// 0.30'un iki olcumu de (11.34) bandin TAMAMEN disinda kaldi.
// NEDEN 0.30 SECILMISTI (adim 96b): o supurme yalnizca 0.25 ve 0.30'u
// denemisti ve ikisi arasinda 0.30 iyiydi -- 0.45 HIC denenmemisti. MATLAB'in
// ince taramasi 0.25'i onerdigi icin arama araligi bastan dar tutulmustu.
static px4::atomic<int32_t> _tilt_jerk_limit_mrs{(int32_t)(0.45f * 1000.0f)};

// --- WING TILT CEILING (2026-07-29, step 31 / back-transition phase 1) ---
//
// An UPPER BOUND on the wing rotors' tilt, applied to the allocator's absolute
// box (abs_hi) -- not an objective term, not a preference.
//
// Why a constraint and not a weight. Step 31 re-solved the allocator offline
// per sample and attributed the cruise tilt runaway: zeroing the Fz demand
// REVERSES the drift (+0.64 -> -0.45 deg/s) while zeroing the Fx demand changes
// almost nothing. The driver is the ALTITUDE channel -- at 15 m/s the wing
// carries part of the load, the altitude loop asks for less lift
// (nu_des(Fz) = +2.9 N), and since dFz/ddelta = T*sin(delta) is large while
// dFz/dT = -cos(delta) -> 0 as tilt grows, tilting FORWARD is the cheapest way
// to dump lift. So to retract the tilt by weighting you would have to outbid
// WS_FZ = 20 with WS_FX = 0.05 -- a 160,000x gap in the objective -- and
// raising Ws_Fx to that magnitude is exactly what wrecked yaw in step 7.
// A box bound is the one thing the weights cannot argue with. Same class as
// step 19's lesson: in an incremental allocator you cannot impose an actuator
// configuration by preference, only by changing the objective or the constraint.
//
// Why the WING rotors only (i = 0, 1). Measured in both step-31 probe flights:
// the tail tilt stays at 0.5-0.7 deg throughout and contributes no Fx, so
// constraining it buys nothing -- while it is the STRONGEST pitch actuator of
// the three (dM_y/ddelta = T*(-0.07*cos d + 0.65*sin d), vs -0.106*T for a wing
// rotor at 43 deg). Leaving it free preserves pitch authority during the
// retraction.
//
// Defaults to TILT_MAX, i.e. exactly the previous abs_hi -- behaviourally
// NEUTRAL until commanded, per the step-22 discipline of landing a mechanism
// before landing a behaviour change. Held as millidegrees for the same reason
// as _slew_box_rate_mrs (px4::atomic cannot hold a float).
static px4::atomic<int32_t> _tilt_ceiling_mdeg{(int32_t)(TILT_MAX * 180.0f / (float)M_PI * 1000.0f)};

// --- FORWARD TRANSITION TILT CEILING (2026-08-26, step 97) ---
//
// A SEPARATE ceiling from _tilt_ceiling_mdeg/_bt_tilt_ceil above, applied ONLY
// while `_ft_state != FtState::IDLE` (i.e. only during the forward transition
// itself -- NOT hover, NOT back-transition, NOT fixed-wing GLIDE/ACTIVE, which
// bypasses the WLS box entirely). Deliberately its OWN atomic/state-gate,
// NOT folded into _tilt_ceiling_mdeg or _bt_tilt_ceil -- those are shared with
// back-transition's own, separately-tuned ceiling schedule, and step 97 was
// asked to land the forward-transition fix WITHOUT touching or risking that
// (back-transition tuning is its own, separate future topic).
//
// Why: step 97 found that FtState::RAMP's WLS solve lets the two wing rotors'
// tilt diverge (a real, needed ~9-10 deg differential for yaw/roll trim -- NOT
// a bug) while BOTH climb toward TILT_MAX together. The rotor running ahead
// hits the 90 deg hard ceiling FIRST; at that instant it can no longer track
// the OTHER rotor's continued climb, the differential trim collapses, AND
// wing-rotor Fz authority is exhausted simultaneously (both near-fully tilted
// forward) -- the WLS's only remaining lever is thrust magnitude, which pins
// all three rotors at 100% for 10+ seconds (measured, WLS_LOCKUP_INVESTIGATION_REPORT.md
// step 97) -- a full loss of differential control authority, i.e. uncontrolled
// tumbling with altitude climbing from 30 m to 300+ m in under a minute.
//
// Fix: cap FtState's own tilt well BELOW 90 deg, so the leading rotor never
// runs out of room -- when it hits ITS ceiling, the OTHER rotor still has
// plenty of slack in BOTH directions to absorb the yaw/roll trim alone
// (measured, same test: asymmetry actually WIDENED to 20-25 deg with the
// ceiling at 45 deg, yet roll stayed exactly 0.00-0.03 deg -- the asymmetry's
// SIZE was never the problem, both rotors simultaneously running out of room
// was). The final, genuinely-high-tilt stretch (up to 90 deg) is left to
// fixedWingTransition()'s GLIDE state, which is structurally immune to this
// failure mode: it drives BOTH wing rotors from ONE shared scalar (`_fw_tilt`),
// so no differential -- and therefore no differential collapse -- can occur.
//
// KALICI VARSAYILAN: 45 deg (2026-08-26, step 99). SITL-validated live at
// 45 deg (`ftceil 45`) first -- see the report for the full climb ->
// FtState::RAMP -> force_fw -> GLIDE -> ACTIVE trace, clean throughout --
// THEN, because a manual `ftceil 45` was easy to forget between test runs
// and the very next un-commanded run reproduced step 97's original tumble
// verbatim (both rotors converge to 90 deg together at t=~109s, all three
// motors pin to 1.0, roll to +-180 deg, altitude 30m -> 1300m+ in under a
// minute -- WLS_LOCKUP_INVESTIGATION_REPORT.md step 99), made the DEFAULT
// itself 45 deg so the fix applies with no console command needed. Still
// overridable live via `ftceil <deg>` for further tuning/testing.
// Milli-degrees for the same px4::atomic-cannot-hold-a-float reason as
// _tilt_ceiling_mdeg.
static px4::atomic<int32_t> _ft_tilt_ceiling_mdeg{45000};

// TEST HOOK ONLY (`force_fw` console command), same shape as _tilt_ceiling_mdeg
// above: custom_command() is static and cannot reach the instance's _fw_state
// directly. Bypasses the fixed-wing entry gate's speed/altitude/FtState::CRUISE
// requirements -- NOT a production trigger, and NOT wired to anything the
// setpoint topic or a pilot can reach. Exists to validate fixedWingTransition()/
// fixedWingControlLaw() in isolation from the separate (pre-existing,
// unrelated) question of whether a given SITL run's forward transition
// actually reaches FtState::CRUISE at cruise speed -- see Adim 62.
static px4::atomic<bool> _fw_force_arm{false};

// TEST HOOK ONLY (`fw_hdg` console command), same shape/reason as _fw_force_arm
// above. _fw_yaw_entry (the bank-to-turn hdg_sp fixedWingControlLaw() steers
// roll toward) is otherwise only ever written once, bumplessly, on the
// IDLE->GLIDE edge -- there is no live way to command a heading change during
// FwState::ACTIVE to exercise the loop MATLAB already validated under a moving
// hdg_sp (Adim 75-76: 20 deg heading step, clean). Millidegrees for the same
// px4::atomic-cannot-hold-a-float reason as _slew_box_rate_mrs/_tilt_ceiling_mdeg
// above; INT32_MIN is the "no override pending" sentinel (a real heading is
// always in [-180, 180] deg = [-180000, 180000] mdeg, nowhere near it).
static px4::atomic<int32_t> _fw_hdg_override_mdeg{INT32_MIN};

MulticopterIndiTiltrotor::MulticopterIndiTiltrotor() :
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	resetState();
}

MulticopterIndiTiltrotor::~MulticopterIndiTiltrotor()
{
	perf_free(_loop_perf);
}

bool
MulticopterIndiTiltrotor::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

int
MulticopterIndiTiltrotor::print_status()
{
	perf_print_counter(_loop_perf);
	static const char *const fs_name[] = {"NONE", "NO_POS", "NO_ALT"};
	PX4_INFO("armed: %d, ekf ok: %d, u_actual seeded: %d, failsafe: %s",
		 _vehicle_control_mode.flag_armed, ekfHealthy(hrt_absolute_time()),
		 _u_actual_seeded, fs_name[(int)_fs_level]);
	PX4_INFO("pilot input: %s%s", _man_active ? "ACTIVE" : "none (bench setpoint path)",
		 _man_lost ? " -- LINK LOST, holding+descending" : "");
	return 0;
}

void
MulticopterIndiTiltrotor::resetState()
{
	_u_actual_seeded = false;
	_u_actual.setZero();

	for (int i = 0; i < 3; i++) {
		_leso[i] = LesoAxis{};
	}

	_d_hat.setZero();
	_prev_u_leso.setZero();
	_leso_accum = 0.0f;

	_alt_integral_vz = 0.0f;
	_alt_accum = 0.0f;
	_fz_sp = -MASS * GRAVITY;

	// Outer-loop state (blocker B1, step 32). These were NOT reset before, so a
	// disarm -> arm cycle re-entered flight carrying the previous run's position
	// target, integrator and back-transition state -- e.g. arming after an
	// aborted back-transition would resume in BRAKE with a 20 deg tilt ceiling
	// and a nose-up command. Arming must start from a known configuration.
	_pos_hold_active = false;
	_pos_sp.setZero();
	_pos_integral_v.setZero();
	_pos_att_sp.setZero();
	_pos_fx_trim = 0.0f;
	_pos_hold_refused = false;

	_bt_accum = 0.0f;
	_bt_state = BtState::IDLE;
	_bt_tilt_ceil = TILT_MAX;
	_bt_tilt_floor = TILT_MIN;
	_bt_floor_dwell = 0.0f;
	_bt_pitch_sp = 0.0f;
	_bt_req_pos_hold = false;
	_bt_refused = false;
	_bt_handoff_wait = 0.0f;

	_ft_state = FtState::IDLE;
	_ft_fx_cmd = 0.0f;
	_ft_z_entry = 0.0f;
	_ft_t_ramp = 0.0f;
	_ft_release_hold = false;
	_ft_req_abort = false;
	_ft_refused = false;

	_cruise_fx_I = 0.0f;
	_cruise_pitch_I = 0.0f;
	_cruise_fx_cmd = 0.0f;
	_cruise_pitch_sp = 0.0f;

	_fs_level = FsLevel::NONE;
	_fs_level_reported = FsLevel::NONE;

	_man_active = false;
	_man_lost = false;
	_man_yaw_sp = 0.0f;
	_man_z_sp = 0.0f;

	_setpoint_valid = false;

	// Temas mandali (Adim 118): arm dongusu temiz baslamali. _z_datum BILEREK
	// burada DEGIL -- o bir ZEMIN referansi, ucus durumu degil, ve disarm'da
	// her tick yeniden yakalaniyor.
	_land_contact_dwell = 0.0f;
	_land_contact_latch = false;
}

void
MulticopterIndiTiltrotor::captureGroundDatum()
{
	// STEP 116 -- the land_diff gate's altitude source. Measurement first:
	// across 23 mission runs the reported altitude `-lpos.z` carried a constant
	// per-run offset of -0.67 .. +1.77 m (mean +0.41, std 0.69), read directly
	// from the first 3 s of each log while the vehicle SAT ON THE GROUND. It is
	// not in-flight drift: the pre-takeoff value matched the post-landing
	// resting value within 0.3 m. The offset is the EKF local origin's own
	// altitude error, frozen at estimator init.
	//
	// Consequence, measured (ULog 11_26_27): the vehicle touched down at a TRUE
	// 0.64 m while the signal read 2.41 m, so `(-lpos.z) < LAND_DIFF_ALT` (2.0 m)
	// never opened, the step-112 clamp never armed, and the wing difference ran
	// to full scale (45 N). Step 113 recorded that run as an IN-AIR lockup at
	// 2.36 m -- there was no such thing; the altitude datum was wrong.
	//
	// WHY DISARMED AND NOT `vehicle_land_detected`: step 110 measured that this
	// airframe's detector is 14 s LATE at touchdown, because the module keeps
	// commanding ~0.61 normalized thrust on the ground so `has_low_throttle`
	// never latches. That kills it as a gate for the windup. It is NOT what is
	// used here: `flag_armed == false` needs no low-throttle inference, cannot
	// be true while flying, and the datum is only ever needed BEFORE takeoff,
	// where the detector was also measured correct (zero false positives).
	//
	// WHY NOT dist_bottom: `dist_bottom_valid` is 0 in every log -- this airframe
	// carries no rangefinder. On hardware that grows one it is the better source
	// and should be preferred here; until one is measured, adding that branch
	// would be adding an untested path.
	//
	// Re-datums on every disarm, so a flight that lands somewhere else leaves
	// the correct reference behind for the next one.
	vehicle_local_position_s lpos{};

	if (!_vehicle_local_position_sub.copy(&lpos) || !lpos.z_valid) {
		return;
	}

	_z_datum = lpos.z;
	_z_datum_reported = false;
}

void
MulticopterIndiTiltrotor::publishDisarmed()
{
	const hrt_abstime now = hrt_absolute_time();

	actuator_motors_s motors{};
	motors.timestamp_sample = now;

	for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; i++) { motors.control[i] = NAN; }

	motors.timestamp = hrt_absolute_time();
	_actuator_motors_pub.publish(motors);

	actuator_servos_s servos{};
	servos.timestamp_sample = now;

	for (int i = 0; i < actuator_servos_s::NUM_CONTROLS; i++) { servos.control[i] = NAN; }

	servos.timestamp = hrt_absolute_time();
	_actuator_servos_pub.publish(servos);
}

bool
MulticopterIndiTiltrotor::ekfHealthy(hrt_abstime now)
{
	estimator_status_flags_s flags;

	if (!_estimator_status_flags_sub.copy(&flags)) {
		return false;
	}

	// estimator_status_flags republishes roughly once a second (state-flags
	// topic, not a per-tick stream, confirmed in SITL) -- a tight freshness
	// window here just means this check spends half its time "stale" against
	// a healthy estimator. The real per-tick liveness check is the
	// vehicle_attitude/vehicle_angular_velocity freshness check in Run().
	//
	// Written as an addition, NOT `(now - flags.timestamp) > 3_s` -- both are
	// uint64 and `now` is a gyro SAMPLE time, so a topic published from a later
	// sample makes the subtraction wrap to ~1.8e19. That exact bug cost this
	// module NaN-to-all-motors 7-13x per flight before step 32 caught it on
	// vehicle_attitude; step 39 (2026-08-03) found the same form still here and
	// at three other sites and made all of them additive. Measured in SITL: this
	// one never wraps today (0 of 39217 ticks) because the gz gyro's
	// publish-minus-sample gap is ~0 us -- but that gap is an artefact of
	// lockstep simulation. Real sensor pipelines have real latency, so this is a
	// bug that SITL structurally cannot show and hardware would.
	if (flags.timestamp + 3_s < now) {
		return false;
	}

	// cs_tilt_align/cs_yaw_align are sticky (become true once and stay true) --
	// safe to gate hard actuation on. The fs_bad_* fusion-fault bits are
	// per-timestep numerical-fault pulses that flicker true/false even during
	// nominal flight (observed in SITL: alternated every other control tick);
	// gating actuation on them causes the controller to chatter between real
	// output and the disarmed-NaN pattern every ~4 ms, which is worse than
	// either state alone. They belong in the post-flight EKF report (M7,
	// Tools/ecl_ekf), not in this per-tick hard gate.
	return flags.cs_tilt_align && flags.cs_yaw_align;
}

// Split out from ekfHealthy() for blocker B1 (step 32). The two alignment bits
// gate DIFFERENT loops and deserve different answers:
//   cs_tilt_align false -> the roll/pitch reference is meaningless, and with no
//                          roll/pitch reference this airframe has no control law
//                          at all (step 35) -> output is cut, see Run()
//   cs_yaw_align  false -> only the HEADING reference is meaningless. Chasing a
//                          yaw setpoint against an unaligned estimate is how you
//                          command a turn nobody asked for, so the yaw target is
//                          pinned to the current heading and that axis is left
//                          with rate damping only -- roll/pitch are untouched,
//                          since nothing is wrong with them.
// ekfHealthy() keeps its old meaning (both bits) for the status/telemetry field,
// so logs stay comparable with every flight already in the report.
bool
MulticopterIndiTiltrotor::ekfYawAligned(hrt_abstime now)
{
	estimator_status_flags_s flags;

	if (!_estimator_status_flags_sub.copy(&flags)) {
		return false;
	}

	// Additive for the same reason as ekfHealthy() above (step 39).
	if (flags.timestamp + 3_s < now) {
		return false;
	}

	return flags.cs_yaw_align;
}

// Commanded/derived termination. **The comment that used to sit here claimed
// "both of its causes are commanded, not inferred". Step 34 measured that and it
// is wrong: only manual_lockdown is a human.** Both of the others are functions
// of commander's own automatic nav_state:
//
//   Commander.cpp:1881             armed.lockdown = (nav_state == TERMINATION)
//                                                   || HIL || throw_launch
//   ModeUtil/control_mode.cpp:119  flag_control_termination_enabled
//                                                 = (nav_state == TERMINATION)
//
// and commander enters TERMINATION by itself when the attitude estimate goes
// invalid, because FailsafeBase::modeCanRun() then fails and this airframe has
// no fallback mode (its .post script stops flight_mode_manager and
// mc_pos_control). That is what defeated FsLevel::RATE_ONLY within 50 ms and
// dropped the vehicle 35 m in step 34.
//
// Knowing that, the module no longer tries to out-vote it: attitude loss is
// handled as a hard prerequisite in Run() (step 35), for the reason recorded in
// TiltrotorIndiParams.hpp -- rate damping alone was measured NOT to be
// survivable, so there is nothing to preserve by disagreeing.
//
// Degraded OUTER-loop inputs (xy, z/vz) still degrade rather than cut; that part
// of the graded failsafe stands and level 1 is measured working.
bool
MulticopterIndiTiltrotor::terminationCommanded()
{
	actuator_armed_s armed;

	if (_actuator_armed_sub.copy(&armed) && (armed.lockdown || armed.manual_lockdown)) {
		return true;
	}

	return _vehicle_control_mode.flag_control_termination_enabled;
}

void
MulticopterIndiTiltrotor::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
	}

	vehicle_angular_velocity_s angular_velocity;

	if (!_vehicle_angular_velocity_sub.update(&angular_velocity)) {
		perf_end(_loop_perf);
		return;
	}

	const hrt_abstime now = angular_velocity.timestamp_sample;
	const float dt = math::constrain((now - _last_run) * 1e-6f, 0.000125f, 0.02f);
	_last_run = now;

	_vehicle_control_mode_sub.update(&_vehicle_control_mode);

	if (_setpoint_sub.updated()) {
		tiltrotor_indi_setpoint_s sp;

		if (_setpoint_sub.copy(&sp)) {
			_setpoint = sp;
			_setpoint_valid = true;
		}
	}

	if (!_vehicle_control_mode.flag_armed) {
		captureGroundDatum();
		resetState();
		publishDisarmed();
		perf_end(_loop_perf);
		return;
	}

	// One line per arming: the datum the land_diff gate will use for the whole
	// flight. Cheap, and it is the only way to check offline whether the gate
	// COULD have armed -- no status field was added for it (step 116).
	if (!_z_datum_reported) {
		_z_datum_reported = true;

		if (PX4_ISFINITE(_z_datum)) {
			PX4_INFO("ground datum: z = %.2f m (agl = datum - z)", (double)_z_datum);

		} else {
			PX4_WARN("ground datum MISSING -- land_diff gate stays closed this flight");
		}
	}

	// --- commanded termination: the only in-flight path left that cuts output ---
	if (terminationCommanded()) {
		resetState();
		publishDisarmed();

		tiltrotor_indi_status_s status{};
		status.timestamp = hrt_absolute_time();
		status.ekf_healthy = ekfHealthy(now);
		status.shadow_u_actual_valid = _u_actual_seeded;
		// resetState() ran just above, so these are IDLE/TILT_MAX -- written out
		// rather than left zero-filled, because a logged ceiling of 0 rad would
		// read as "wing tilt clamped shut", the opposite of the truth.
		status.bt_state = (uint8_t)_bt_state;
		status.bt_tilt_ceil = _bt_tilt_ceil;
		status.bt_tilt_floor = _bt_tilt_floor;
		status.pos_hold_active = _pos_hold_active;
		_status_pub.publish(status);

		perf_end(_loop_perf);
		return;
	}

	// --- input validity, per consumer (blocker B1, step 32) ---
	// Each flag gates exactly the loop that needs it. Deliberately NOT a ladder:
	// losing xy while z is fine is a different failure from losing z while xy is
	// fine, and folding them into one severity would switch off a loop whose
	// input is still valid. _fs_level below is only the worst-of SUMMARY, for
	// logging and telemetry.
	const bool ekf_ok = ekfHealthy(now);
	const bool yaw_ref_ok = ekfYawAligned(now);

	// Additive freshness (step 39) -- see ekfHealthy(). This site is the one that
	// matters most: `tilt_aligned` gates att_ok, and since step 35 a false att_ok
	// CUTS THE OUTPUT and resets the back-transition state machine. A wrap here
	// is therefore indistinguishable from a real attitude loss, which is exactly
	// the signature of open item (T) (`attitude LOST (tilt=0)` mid-flight).
	// This does NOT close (T): measured, the subtraction never wraps in SITL
	// (0 of 39217 ticks), so it cannot be what fired there. It removes a THIRD
	// candidate mechanism that step 38 had not considered -- and the one that
	// would appear first on hardware, where sample-to-publish latency is real.
	estimator_status_flags_s ekf_flags{};
	const bool tilt_aligned = _estimator_status_flags_sub.copy(&ekf_flags)
				  && (ekf_flags.timestamp + 3_s > now) && ekf_flags.cs_tilt_align;

	// UNSIGNED-UNDERFLOW BUG, found and fixed here (2026-07-29, step 32).
	// The old test was `(now - att.timestamp) < 50_ms` on two uint64s, with
	// `now = angular_velocity.timestamp_sample`. vehicle_attitude is published
	// from a LATER sample than the gyro tick currently being processed, so
	// att.timestamp is routinely 4 ms AHEAD of now -- measured, age = -4000 us
	// -- and the subtraction wraps to ~1.8e19, which fails the test. The old
	// code answered that by calling publishDisarmed(): **NaN to every motor and
	// servo, mid-hover, ~7-13 times per flight.** It survived only because a
	// single 4 ms tick of NaN is absorbed downstream, and it was invisible
	// because nothing logged it.
	//
	// Written as an addition on the left so it cannot wrap, and so a sample
	// NEWER than now reads as what it is -- the freshest possible -- rather
	// than as infinitely stale.
	vehicle_attitude_s att;
	const bool att_copied = _vehicle_attitude_sub.copy(&att);
	const bool att_fresh = att_copied && (att.timestamp + FS_ATT_TIMEOUT_US > now);
	const bool att_ok = att_fresh && tilt_aligned;

	// --- attitude is a HARD PREREQUISITE, not a failsafe level (step 35) ---
	// This was FsLevel::RATE_ONLY: drop the outer attitude P loop, leave
	// omega_sp = 0 so the inner INDI loop damps rotation, and descend open-loop
	// -- justified in the code as "strictly better than motors off". MATLAB
	// measured it (run_rate_only_test.m, no commander there to pre-empt it) and
	// the claim is false: all three scenarios inverted and hit the ground, the
	// cleanest one rolling to 179.6 deg and arriving at 11.7 m/s against a
	// 26.2 m/s free-fall reference. The full table and the mechanism are in
	// TiltrotorIndiParams.hpp. So there is no degraded flight to preserve here.
	//
	// Nothing is lost by cutting: commander independently reaches
	// NAVIGATION_STATE_TERMINATION on this same signal about 50 ms later and
	// raises actuator_armed.lockdown, which terminationCommanded() above already
	// answers with the same NaN. This check only makes the module say so itself,
	// instead of publishing one tick of a flight mode it cannot fly.
	if (!att_ok) {
		if (!_att_loss_reported) {
			PX4_ERR("attitude LOST (cp=%d fresh=%d tilt=%d) -- no control law without it, cutting output",
				(int)att_copied, (int)att_fresh, (int)tilt_aligned);
			_att_loss_reported = true;
		}

		// resetState() for the same reason the termination path calls it: this
		// return skips the LESO update and the INDI increment, so _u_actual,
		// _prev_u_leso and the integrators would FREEZE. If attitude ever came
		// back, the incremental law would resume against a shadow state that no
		// longer describes the vehicle -- the stale-shadow hazard this project
		// has already paid for. Resetting means a resume re-seeds from
		// hoverTrim() instead.
		resetState();
		publishDisarmed();

		tiltrotor_indi_status_s status{};
		status.timestamp = hrt_absolute_time();
		status.ekf_healthy = ekfHealthy(now);
		status.shadow_u_actual_valid = _u_actual_seeded;
		// resetState() ran just above, so these are IDLE/TILT_MAX -- written out
		// rather than left zero-filled, because a logged ceiling of 0 rad would
		// read as "wing tilt clamped shut", the opposite of the truth.
		status.bt_state = (uint8_t)_bt_state;
		status.bt_tilt_ceil = _bt_tilt_ceil;
		status.bt_tilt_floor = _bt_tilt_floor;
		status.pos_hold_active = _pos_hold_active;
		_status_pub.publish(status);

		// resetState() deliberately does NOT clear _att_loss_reported -- it runs
		// every tick here, and clearing it would turn a one-shot error into 250 Hz
		// of console spam.
		perf_end(_loop_perf);
		return;
	}

	_att_loss_reported = false;

	vehicle_local_position_s lpos{};
	// FRESHNESS, which this check never had: uORB::Subscription::copy() keeps
	// returning the last sample forever, so before step 32 a *stopped* estimator
	// read as perfectly healthy here while a *flickering* one cut the motors.
	// Same underflow-safe form as the attitude check above -- vehicle_local_position
	// is published from a later sample than this tick's gyro too (measured on the
	// same run: one level-2 report with alt=0 xy=0 from exactly that wrap).
	const bool lpos_fresh = _vehicle_local_position_sub.copy(&lpos)
				&& (lpos.timestamp + FS_LPOS_TIMEOUT_US > now);
	// Both vertical bits are tested even though EKF2 currently makes them
	// IDENTICAL -- EKF2.cpp:1588-1590 assigns z_valid and v_z_valid the same
	// `isLocalVerticalPositionValid() || isLocalVerticalVelocityValid()` OR, under
	// a TODO saying consumers mishandle them differing. Depending on that
	// coincidence would be depending on someone else's TODO, so the test stays
	// honest; there is just no longer a separate `vz_ok` branch behind it (step 36).
	const bool alt_ok = lpos_fresh && lpos.v_z_valid && lpos.z_valid;
	const bool xy_ok = lpos_fresh && lpos.xy_valid && lpos.v_xy_valid;

	// The position loop and the back-transition both need a heading to rotate
	// their NED errors into body frame (positionLoop's psi argument). Attitude
	// is guaranteed valid past the hard-prerequisite check above, so the
	// horizontal estimate is all that is left to test.
	const bool pos_ok = xy_ok;

	// Only the OUTER loops are graded. Losing xy while z is fine is a different
	// failure from losing z while xy is fine, so this is deliberately not a
	// ladder of severity; _fs_level is the worst-of SUMMARY, for logging and
	// telemetry. Attitude no longer appears here -- it is not degradable.
	_fs_level = !alt_ok ? FsLevel::NO_ALT
		    : !xy_ok  ? FsLevel::NO_POS
		    : FsLevel::NONE;

	if (_fs_level != _fs_level_reported) {
		if (_fs_level == FsLevel::NONE) {
			PX4_INFO("failsafe CLEARED -- all estimates valid again");

		} else {
			// Decomposed on purpose: the first version printed only one flag and
			// cost a whole run to work out WHICH term was false.
			PX4_WARN("failsafe %d (alt=%d xy=%d yaw_ref=%d) -- degrading, NOT cutting",
				 (int)_fs_level, (int)alt_ok, (int)xy_ok, (int)yaw_ref_ok);
		}

		_fs_level_reported = _fs_level;
	}

	if (!_u_actual_seeded) {
		// No servo/ESC position feedback exists in PX4 (see TiltrotorIndiControl.hpp
		// header note) -- seed the shadow model with the analytic hover trim
		// (hoverTrim(), ported from hover_trim.m) instead of a naive equal-thrust
		// zero-tilt guess. The naive seed caused a ~90 deg yaw excursion in the
		// first ~1s after arming in SITL testing (2xCCW+1xCW reaction-torque
		// imbalance, uncorrected because zero tilt gives zero yaw authority) --
		// MATLAB's test scripts never hit this because they start pre-trimmed
		// (x0(14:19) = hover_trim(p)), they never seed from a cold guess.
		_u_actual = hoverTrim();
		_u_actual_seeded = true;
	}

	// att_ok is guaranteed here (the hard-prerequisite check above returns
	// otherwise), so this is a fresh, tilt-aligned attitude -- not the
	// zero-initialised identity quaternion that a failed copy would leave.
	const matrix::Eulerf euler{matrix::Quatf{att.q}};
	const matrix::Vector3f att_now(euler.phi(), euler.theta(), euler.psi());
	const matrix::Vector3f omega(angular_velocity.xyz);
	const matrix::Vector3f omega_dot_meas(angular_velocity.xyz_derivative);

	float roll_sp = 0.0f, pitch_sp = 0.0f, yaw_sp = 0.0f, fx_sp = 0.0f;
	// No setpoint yet -> hold the current altitude (vz-damping only). Under
	// !alt_ok, lpos.z is stale or invalid, so this seed is meaningless -- but
	// the altitude loop is switched off on that path anyway (see below).
	float z_sp = lpos.z;
	bool leso_enable[3] = {true, true, false}; // matches the validated MATLAB hover-gust config

	// Requests that the two setpoint SOURCES (bench topic, pilot) both express.
	// Held as locals so the blocks below have exactly one thing to read.
	bool req_pos_hold = false;
	bool req_bt = false;
	bool req_ft = false;
	bool req_fw = false;

	if (_setpoint_valid) {
		roll_sp = _setpoint.roll_sp;
		pitch_sp = _setpoint.pitch_sp;
		yaw_sp = _setpoint.yaw_sp;
		fx_sp = _setpoint.fx_sp;
		z_sp = _setpoint.z_sp;
		leso_enable[0] = _setpoint.leso_enable_roll;
		leso_enable[1] = _setpoint.leso_enable_pitch;
		leso_enable[2] = _setpoint.leso_enable_yaw;
		req_pos_hold = _setpoint.pos_hold_enable;
		req_bt = _setpoint.bt_enable;
		req_ft = _setpoint.ft_enable;
		req_fw = _setpoint.fw_enable;
	}

	// --- pilot input (blocker B1, step 33) ---
	// Overrides the bench topic when a human is actually flying. See the note in
	// TiltrotorIndiParams.hpp for why the sticks are read here rather than
	// through a FlightTask, and why this is inert on every existing SITL test
	// (no RC/joystick -> manual_control_setpoint is never published).
	manual_control_setpoint_s man{};
	const bool man_ok = _vehicle_control_mode.flag_control_manual_enabled
			    && _manual_control_setpoint_sub.copy(&man)
			    && man.valid
			    // Additive (step 39, see ekfHealthy()). This site is the most
			    // exposed of the four: manual_control_setpoint.timestamp is
			    // stamped with hrt_absolute_time() when the manual_control
			    // module republishes, i.e. REAL time, while `now` is a gyro
			    // sample time -- so "topic newer than now" is the normal case
			    // here, not an edge case. With the wrapping form, a stick input
			    // that arrives between the sample and this tick reads as
			    // infinitely stale and the pilot is silently ignored. On a path
			    // whose entire purpose is letting a human intervene, failing
			    // closed and quiet is the worst available behaviour.
			    && (man.timestamp + MAN_TIMEOUT_US > now)
			    && PX4_ISFINITE(man.roll) && PX4_ISFINITE(man.pitch)
			    && PX4_ISFINITE(man.yaw) && PX4_ISFINITE(man.throttle);

	if (man_ok) {
		if (!_man_active) {
			// Handover. Seed the integrated targets from where the vehicle
			// actually IS, so taking the sticks never commands a step.
			_man_yaw_sp = att_now(2);
			_man_z_sp = alt_ok ? lpos.z : 0.0f;
			_man_active = true;
			_man_lost = false;
			PX4_INFO("pilot input ACTIVE (source %d)", (int)man.data_source);
		}

		// Roll/pitch: direct angle command. PX4 stick convention is "+pitch stick
		// = move forward = nose DOWN", while this controller's pitch_sp is
		// +nose-up (see positionLoop's sign note), hence the negation.
		const float roll_stick = math::constrain(man.roll, -1.0f, 1.0f);
		const float pitch_stick = math::constrain(man.pitch, -1.0f, 1.0f);
		roll_sp = roll_stick * MAN_TILT_MAX;
		pitch_sp = -pitch_stick * MAN_TILT_MAX;

		// Yaw: rate command integrated into the heading target, then LEASHED to
		// the current heading. Without the leash a held stick walks the setpoint
		// away faster than this airframe can turn (yaw authority ~0.05 Nm per
		// step) and the wrapped error diverges -- step 13's failure exactly.
		const float yaw_stick = fabsf(man.yaw) < MAN_STICK_DEAD ? 0.0f : math::constrain(man.yaw, -1.0f, 1.0f);
		_man_yaw_sp += yaw_stick * MAN_YAW_RATE * dt;
		float yaw_lead = atan2f(sinf(_man_yaw_sp - att_now(2)), cosf(_man_yaw_sp - att_now(2)));
		yaw_lead = math::constrain(yaw_lead, -MAN_YAW_LEASH, MAN_YAW_LEASH);
		_man_yaw_sp = att_now(2) + yaw_lead;
		yaw_sp = _man_yaw_sp;

		// Throttle: climb-rate command with a centre deadband, leashed the same
		// way. NED, so up (+throttle) is -z.
		const float thr_stick = fabsf(man.throttle) < MAN_STICK_DEAD ? 0.0f : math::constrain(man.throttle, -1.0f,
				1.0f);
		_man_z_sp -= thr_stick * MAN_VZ_MAX * dt;

		if (alt_ok) {
			_man_z_sp = math::constrain(_man_z_sp, lpos.z - MAN_Z_LEASH, lpos.z + MAN_Z_LEASH);
		}

		z_sp = _man_z_sp;

		// Sticks centred in roll/pitch -> hold position. The hold captures its
		// target on the rising edge, so releasing the sticks pins the vehicle
		// where it was let go. This is what makes hands-off hover the DEFAULT
		// rather than a mode the pilot has to remember to select -- and item (N)
		// is why that matters here: with no hold the airframe accelerates itself
		// forward structurally (235 m in 25 s, step 15).
		req_pos_hold = (fabsf(roll_stick) < MAN_STICK_DEAD) && (fabsf(pitch_stick) < MAN_STICK_DEAD);
		fx_sp = 0.0f;

		// Back-transition on the VTOL transition switch, which gives the pilot
		// both halves that blocker B5 called out as missing: they can START the
		// manoeuvre and they can CANCEL it (flip back to FORWARD_FLIGHT), at any
		// point in its 30-40 s run.
		manual_control_switches_s sw;

		if (_manual_control_switches_sub.copy(&sw)) {
			req_bt = (sw.transition_switch == manual_control_switches_s::SWITCH_POS_OFF);
		}

	} else if (_man_active) {
		// Control-link loss while a human was flying. The estimates are all still
		// good -- there is simply nobody at the controls -- so this is NOT a
		// failsafe LEVEL. Hold position and descend, which on this airframe means
		// engaging the loop that stops it flying away (item (N)).
		const float v_h_now = matrix::Vector2f(lpos.vx, lpos.vy).norm();

		if (!_man_lost) {
			// Report what this branch can ACTUALLY deliver, not what it intends.
			// The old message said "holding position and descending" flatly; step
			// 40 measured the vehicle flying away at 4.8-6.1 m/s for the whole
			// descent and touching down 338-368 m from the arm point, because the
			// hold below is only a REQUEST and POS_ENGAGE_V_MAX refuses it above
			// 3 m/s. A log line asserting an outcome the code cannot guarantee is
			// how step 35's RATE_ONLY comment survived for months.
			if (v_h_now > POS_ENGAGE_V_MAX) {
				PX4_WARN("pilot input LOST at %.1f m/s -- too fast for the hold (limit %.1f), "
					 "handing over to the back-transition (item (U))",
					 (double)v_h_now, (double)POS_ENGAGE_V_MAX);

			} else {
				PX4_WARN("pilot input LOST at %.1f m/s -- holding position and descending",
					 (double)v_h_now);
			}

			_man_lost = true;
		}

		req_pos_hold = true;

		// ITEM (U) FIX (step 41, 2026-08-03). This used to be a flat
		// `req_bt = false`, which switched OFF the only mechanism on this
		// airframe that can turn cruise speed back into hover -- on the one path
		// where, by definition, nobody can intervene. The hold requested above is
		// only a REQUEST: POS_ENGAGE_V_MAX refuses it above 3 m/s, and item (N)'s
		// structural forward acceleration is exactly what puts the vehicle there.
		// So the branch promised station keeping and delivered a 340-370 m
		// fly-away. Step 34(d) had already written that station keeping must be
		// handed to bt_enable on recovery; this is the same wiring, on the more
		// dangerous path.
		//
		// LATCHED on `_bt_state != IDLE` rather than on speed alone: without the
		// latch req_bt would drop the instant speed dipped under the gate, which
		// resets the machine to IDLE and RELEASES the ceiling, then re-arms it a
		// tick later -- the manoeuvre would restart from scratch instead of
		// finishing. Releasing on `_pos_hold_active` is the right end condition
		// because that is the thing the whole manoeuvre exists to reach.
		//
		// The altitude channel keeps descending underneath all of this: the two
		// are independent demands and the descent must NOT wait for the
		// deceleration -- there is no pilot coming back.
		req_bt = !_pos_hold_active
			 && ((v_h_now > POS_ENGAGE_V_MAX) || (_bt_state != BtState::IDLE));
		roll_sp = 0.0f;
		pitch_sp = 0.0f;
		yaw_sp = att_now(2);
		fx_sp = 0.0f;

		if (alt_ok) {
			_man_z_sp += FS_DESCENT_VZ * dt;
			_man_z_sp = math::constrain(_man_z_sp, lpos.z - MAN_Z_LEASH, lpos.z + MAN_Z_LEASH);
			z_sp = _man_z_sp;
		}
	}

	// Degraded: do not push a body-x force when you no longer know where you are.
	// The one-sided tilt range means a standing fx_sp keeps accelerating the
	// vehicle forward (report item (N)) with nothing left to arrest it.
	if (_fs_level != FsLevel::NONE) {
		fx_sp = 0.0f;
	}

	// Heading reference untrusted -> hold the CURRENT heading, so the outer loop
	// contributes no yaw error and the axis is left with pure rate damping.
	// Yaw is this airframe's weakest axis (~0.05 Nm per step, step 12g); handing
	// it a bad reference is how step 13's runaway started.
	if (!yaw_ref_ok) {
		yaw_sp = att_now(2);
	}

	// --- 0a0) forward transition (forwardtrans_loop.m), item (V), step 42 ---
	// Runs BEFORE the back-transition block on purpose: an FT abort has to be able
	// to request the BT in the SAME tick. That ordering is the whole abort path --
	// step 30 measured that zeroing fx does NOT slow this airframe down, so the
	// only real abort is "hand it to the machine that can".
	//
	// The two are mutually exclusive by construction: FT refuses to start while a
	// BT is running, and an FT abort ends the FT. Without that, `bt_enable` and
	// `ft_enable` could both be true and would fight over fx_sp and pitch.
	const float ft_v_h = matrix::Vector2f(lpos.vx, lpos.vy).norm();
	bool ft_enable = req_ft && pos_ok && alt_ok && (_bt_state == BtState::IDLE);

	if (req_ft && _ft_state == FtState::IDLE && ft_enable && -lpos.z < FT_MIN_ALT) {
		// Refuse to START low -- the manoeuvre trades altitude for speed and its
		// abort path is a 30-40 s back-transition. Same shape as BT_MIN_ALT, and
		// the same measured reason it exists there (step 41: a link-loss handover
		// at 7.2 m was correctly refused because there is no room to manoeuvre).
		ft_enable = false;

		if (!_ft_refused) {
			PX4_WARN("forward transition REFUSED: %.1f m AGL < %.1f m minimum",
				 (double)(-lpos.z), (double)FT_MIN_ALT);
			_ft_refused = true;
		}

	} else if (!req_ft) {
		_ft_refused = false;
	}

	if (_bt_accum + dt >= ALT_TS - 1e-6f) {
		const FtState ft_prev = _ft_state;
		const bool sat_thrust = _sat_thrust_prev;   // previous tick -- see the member note
		uint8_t ft_warn = 0;
		forwardTransition(ft_enable, ft_v_h, lpos.z, sat_thrust, _ft_state, _ft_fx_cmd,
				  _ft_z_entry, _ft_t_ramp, _ft_pitch_sp, _ft_release_hold, _ft_req_abort,
				  ft_warn);

		if (_ft_state != ft_prev) {
			PX4_INFO("forward transition: state %d -> %d (v_h=%.1f m/s, fx=%.1f N, tilt=%.1f deg)",
				 (int)ft_prev, (int)_ft_state, (double)ft_v_h, (double)_ft_fx_cmd,
				 (double)(fmaxf(_u_actual(3), _u_actual(4)) * 180.0f / (float)M_PI));
		}

		// The detectors still run with FT_ALLOW_ABORT off -- losing the MEASUREMENT
		// was never the intent, only the action. Logged on the rising edge so a
		// sustained deviation does not flood the console.
		if (ft_warn != 0 && _ft_warn_prev == 0) {
			PX4_WARN("forward transition WARNING (%s) -- CONTINUING (abort disabled): dz=%.1f m, t_ramp=%.1f s, v_h=%.1f m/s",
				 (ft_warn == 1) ? "altitude band" : "timeout",
				 (double)(lpos.z - _ft_z_entry), (double)_ft_t_ramp, (double)ft_v_h);
		}

		_ft_warn_prev = ft_warn;

		if (_ft_req_abort) {
			// Report WHICH safety term fired -- the altitude band is aero-dependent
			// and the dwell is not, so on hardware they mean different things.
			PX4_ERR("forward transition ABORTED (%s) -- requesting back-transition",
				(fabsf(lpos.z - _ft_z_entry) > FT_ALT_BAND) ? "altitude band" : "timeout");
			_ft_state = FtState::IDLE;
			_ft_fx_cmd = 0.0f;
			_ft_t_ramp = 0.0f;
			req_bt = true;
		}
	}

	if (_ft_state != FtState::IDLE) {
		// The FT owns the forward force and the pitch, and it must hold pos_hold
		// OFF: while the hold is active the position loop owns roll/pitch AND
		// supplies fx_trim (items (N)/(P)), so it would simply overwrite this.
		fx_sp = _ft_fx_cmd;
		pitch_sp = _ft_pitch_sp;
		roll_sp = 0.0f;

		if (_ft_release_hold) {
			req_pos_hold = false;
		}
	}

	// --- 0a-1) fixed-wing terminal mode (Adim 59-61 port) ---
	// Only armed from a COMPLETED forward transition, and mutually exclusive with
	// an in-progress back-transition -- same shape as ft_enable's own gate. GLIDE
	// cuts every rotor for a few seconds, so entry additionally needs speed and
	// altitude margin (checked once, on the IDLE->GLIDE edge only -- like
	// FT_MIN_ALT, an already-running manoeuvre is left alone rather than aborted).
	// SIGNED body forward speed (item S -- see the note on the same computation
	// in the back-transition block below, which reuses this value rather than
	// recomputing it).
	const float bt_v_fwd = lpos.vx * cosf(att_now(2)) + lpos.vy * sinf(att_now(2));
	bool fw_enable = FW_ENABLE && req_fw && pos_ok && alt_ok
			 && (_ft_state == FtState::CRUISE) && (_bt_state == BtState::IDLE);

	// TEST HOOK (`force_fw` console command, see _fw_force_arm's declaration):
	// bypasses the ft_state/speed/altitude checks below entirely. Baseline
	// pos_ok/alt_ok/BtState::IDLE sanity is still required. Deliberately does
	// NOT touch req_fw, so the refusal-latch block right below (keyed on
	// req_fw) stays inert while this is active.
	if (_fw_force_arm.load()) {
		fw_enable = FW_ENABLE && pos_ok && alt_ok && (_bt_state == BtState::IDLE);
	}

	if (req_fw && _fw_state == FwState::IDLE && fw_enable) {
		if (bt_v_fwd < FW_TRIGGER_V) {
			fw_enable = false;

			if (!_fw_refused) {
				PX4_WARN("fixed-wing mode REFUSED: %.1f m/s < %.1f m/s trigger speed",
					 (double)bt_v_fwd, (double)FW_TRIGGER_V);
				_fw_refused = true;
			}

		} else if (-lpos.z < FW_MIN_ALT) {
			fw_enable = false;

			if (!_fw_refused) {
				PX4_WARN("fixed-wing mode REFUSED: %.1f m AGL < %.1f m minimum (GLIDE cuts every rotor)",
					 (double)(-lpos.z), (double)FW_MIN_ALT);
				_fw_refused = true;
			}

		} else if (fabsf(att_now(0)) > FW_MAX_ROLL || fabsf(omega(2)) > FW_MAX_YAW_RATE) {
			// Coordinated-flight check (Adim 67, see FW_MAX_ROLL's header note):
			// a fast-enough, high-enough moment is not necessarily a SETTLED one.
			// Refuse to hand the vehicle to GLIDE's brief, lightly-damped law
			// while it is already banked or yawing -- exactly the state Adim 66
			// armed into and could not recover from.
			fw_enable = false;

			if (!_fw_refused) {
				PX4_WARN("fixed-wing mode REFUSED: not coordinated (roll=%.1f deg, yaw rate=%.1f deg/s, limits %.1f/%.1f)",
					 (double)(att_now(0) * 180.0f / (float)M_PI),
					 (double)(omega(2) * 180.0f / (float)M_PI),
					 (double)(FW_MAX_ROLL * 180.0f / (float)M_PI),
					 (double)(FW_MAX_YAW_RATE * 180.0f / (float)M_PI));
				_fw_refused = true;
			}
		}

	} else if (!req_fw) {
		_fw_refused = false;
	}

	// Called every tick, NOT decimated -- see fixedWingTransition()'s header for
	// why the tilt ramp needs no synchronisation with the allocator/LESO cadence.
	{
		const FwState fw_prev = _fw_state;

		if (_fw_state == FwState::IDLE && fw_enable) {
			// Seed the ramp from the CURRENT wing tilt (shadow model), mirroring
			// forwardTransition()'s own IDLE->RAMP edge seeding z_entry from z.
			_fw_tilt = fmaxf(_u_actual(3), _u_actual(4));
			// Bumpless heading/speed references (Adim 68): capture whatever the
			// vehicle is ACTUALLY doing right now rather than snapping to a fixed
			// external target -- same principle as z_entry above and
			// _man_yaw_sp's handover seeding. FW mode then holds/tracks FROM
			// here, not from a possibly-stale or arbitrarily-different setpoint.
			_fw_yaw_entry = att_now(2);
			_fw_v_fwd_filt = bt_v_fwd;
		}

		// FW RETURN request (step 103): `bt_enable` requested while ACTIVE moves
		// FwState to RETURN -- GLIDE's own proven mechanism (every rotor cut,
		// surfaces alone holding attitude, tilt free-slewing) run in reverse,
		// down to TILT_MIN, before IDLE (and therefore WLS/INDI) resumes. See
		// fixedWingTransition()'s RETURN case and the FwState comment in
		// TiltrotorIndiParams.hpp for why (step 102's first cut, which handed
		// control straight to WLS/INDI at tilt=90 deg/v~16 m/s, tumbled inside
		// one tick -- WLS_LOCKUP_INVESTIGATION_REPORT.md).
		fixedWingTransition(fw_enable, req_bt, dt, _fw_state, _fw_tilt);

		if (_fw_state != fw_prev) {
			PX4_INFO("fixed-wing mode: state %d -> %d (v_fwd=%.1f m/s, tilt=%.1f deg)",
				 (int)fw_prev, (int)_fw_state, (double)bt_v_fwd,
				 (double)(_fw_tilt * 180.0f / (float)M_PI));
		}
	}

	// --- 0a) back-transition state machine (backtrans_loop.m), decimated at ALT_TS ---
	// Runs BEFORE the pos_hold block because its HANDOFF state requests the hold.
	// Entry needs a valid horizontal estimate (the machine switches on v_h) and
	// altitude margin -- the manoeuvre held altitude to <=0.86 m in every measured
	// flight, but a back-transition with no room underneath is not something to
	// discover in the air.
	// pos_ok now carries the xy_valid/v_xy_valid test AND the freshness and
	// attitude requirements it was missing (blocker B1). A back-transition
	// switches on v_h and ends by handing over to a loop that needs a heading,
	// so it cannot run on a degraded estimate -- and unlike the BT_MIN_ALT
	// refusal below, this one must also ABORT a manoeuvre already in progress,
	// because continuing it would be steering on numbers known to be wrong.
	const float bt_v_h = matrix::Vector2f(lpos.vx, lpos.vy).norm();
	// SIGNED body forward speed (bt_v_fwd, computed above in the fixed-wing-mode
	// block): the horizontal velocity projected on the heading. BRAKE/HANDOFF
	// switch on THIS, not on the magnitude above -- the brake law commands pitch,
	// so the forward axis is the only axis it controls, and a magnitude held up
	// by lateral drift ran the vehicle away backward (item S, step 39; see
	// backTransition()). RETRACT keeps the magnitude on purpose.
	bool bt_enable = req_bt && pos_ok;

	if (!pos_ok && _bt_state != BtState::IDLE) {
		PX4_WARN("back-transition ABORTED at state %d: horizontal estimate lost", (int)_bt_state);
		_bt_state = BtState::IDLE;
		_bt_tilt_ceil = TILT_MAX;
		_bt_tilt_floor = TILT_MIN;
		_bt_floor_dwell = 0.0f;
		_bt_pitch_sp = 0.0f;
		_bt_req_pos_hold = false;
		_bt_handoff_wait = 0.0f;
	}

	if (bt_enable && _bt_state == BtState::IDLE && -lpos.z < BT_MIN_ALT) {
		// Refuse to START low; an already-running manoeuvre is left alone, since
		// abandoning it mid-way is its own hazard (same reasoning as pos_hold).
		bt_enable = false;

		if (!_bt_refused) {
			PX4_WARN("back-transition REFUSED: %.1f m AGL < %.1f m minimum",
				 (double)(-lpos.z), (double)BT_MIN_ALT);
			_bt_refused = true;
		}

	} else if (!bt_enable) {
		_bt_refused = false;
	}

	_bt_accum += dt;

	if (_bt_accum >= ALT_TS - 1e-6f) {
		const BtState bt_prev = _bt_state;
		const float delta_wing_max = fmaxf(_u_actual(3), _u_actual(4));
		backTransition(bt_enable, bt_v_h, bt_v_fwd, delta_wing_max, _bt_state, _bt_tilt_ceil,
			       _bt_tilt_floor, _bt_floor_dwell, _bt_pitch_sp, _bt_req_pos_hold);

		if (_bt_state != bt_prev) {
			// Report WHICH exit term fired. RETRACT -> BRAKE on the dwell means the
			// vehicle never got under BT_RELEASE_V, i.e. the terminal speed at the
			// floor is at or above the threshold -- item (R) on this airframe. That
			// is a measurement worth having in the log, not an anomaly: it is the
			// backstop doing its job, and on hardware it is the signal that
			// BT_RELEASE_V needs re-deriving against the real aerodynamics.
			const bool by_dwell = (bt_prev == BtState::RETRACT
					       && _bt_state == BtState::BRAKE
					       && bt_v_h >= BT_RELEASE_V);
			// Both speeds are printed because the phases switch on DIFFERENT
			// ones (item S): RETRACT on the magnitude, BRAKE/HANDOFF on the
			// signed forward component. Reading a transition without both is
			// how item (S) stayed invisible in three flights' logs.
			PX4_INFO("back-transition: state %d -> %d (v_h=%.1f, v_fwd=%+.1f m/s, ceil=%.1f deg)%s",
				 (int)bt_prev, (int)_bt_state, (double)bt_v_h, (double)bt_v_fwd,
				 (double)(_bt_tilt_ceil * 180.0f / (float)M_PI),
				 by_dwell ? " [via FLOOR_DWELL backstop]" : "");

			if (by_dwell) {
				// BRAKE does not touch the timer, so this still holds the dwell
				// that expired.
				PX4_WARN("back-transition: RETRACT never reached BT_RELEASE_V (%.1f m/s) in %.1f s at the floor",
					 (double)BT_RELEASE_V, (double)_bt_floor_dwell);
			}
		}

		_bt_accum -= ALT_TS;
	}

	// --- 0) outer loops (altitude_loop.m + position_loop.m), decimated at ALT_TS ---
	// Horizontal position hold is only usable when the EKF's horizontal state is
	// valid; without it the loop would integrate garbage into a tilt command.
	// The back-transition's HANDOFF state requests it too -- that handoff is the
	// whole point of the manoeuvre (blocker B5).
	const bool pos_hold = (req_pos_hold || _bt_req_pos_hold) && pos_ok;

	// Item (S) residual, measured rather than assumed: the handoff is requested on
	// the forward axis but the hold engages on a magnitude, so lateral drift can
	// keep them apart. Time the gap.
	if (_bt_req_pos_hold && !_pos_hold_active) {
		_bt_handoff_wait += dt;

	} else if (!_bt_req_pos_hold) {
		_bt_handoff_wait = 0.0f;
	}

	if (pos_hold && !_pos_hold_active) {
		// Rising edge. REFUSE to engage above POS_ENGAGE_V_MAX: this loop is
		// hover-only (see TiltrotorIndiParams.hpp -- engaging it at 14.5 m/s
		// saturated pitch, stuck the vehicle in a 9.6 m/s steady climb and
		// crashed it). Capture the current position as the target, so a test
		// does not need to know its own coordinates.
		// RUZGAR DUZELTMESI (2026-08-31, adim 136). Kapi eskiden yer hizinin
		// BUYUKLUGUNU (v_h) okuyordu. OLCULEN ARIZA: 6 m/s ruzgarda geri gecis
		// bitti (v_fwd = 0.02 m/s, yani manevra isini TAM yapti) ama ruzgar
		// araci 7.1 m/s yer hiziyla surukluyordu; kapi bunu "cok hizli" sayip
		// pos_hold'u ornekLERIN %100'unde REDDETTI ve BT_HANDOFF 17.5 s yerine
		// 206.9 s surdu -- arac havada asili kaldi.
		//
		// Kapinin GERCEK gerekcesi (TiltrotorIndiParams.hpp POS_ENGAGE_V_MAX):
		// "kanat ~5-6 m/s ustunde baskin olur, burun yukari komutu FRENLEME
		// degil TIRMANIS komutuna doner". Bu bir AERODINAMIK esiktir, yani
		// HAVA hizina baglidir -- yer hizina degil. Ruzgarsizken ikisi ayni;
		// ruzgarda ayrisirlar ve kapi yanlis olcuyu okur.
		//
		// v_fwd (govde ileri hizi) tam olarak kanadin gordugu bilesendir ve
		// ayni duzeltme geri gecis esiginde de yapilmisti (madde (S), adim 39:
		// "buyukluk esigi, giderilemeyen bir yanal bilesen yuzunden sonsuza
		// kadar saglanmayabilir"). Ayni yapisal desen, ayni cozum.
		//
		// YANAL BILESEN BASIBOS BIRAKILMIYOR: yanal hiz da ayri ve DAHA GENIS
		// bir sinirla kapilanir. Kanat yanal eksende tasima uretmez, yani
		// oradaki tehlike bambaskadir ve esigi de ayni olmak zorunda degildir.
		const float v_fwd_eng = lpos.vx * cosf(att_now(2)) + lpos.vy * sinf(att_now(2));
		const float v_lat_eng = -lpos.vx * sinf(att_now(2)) + lpos.vy * cosf(att_now(2));
		const float v_h = matrix::Vector2f(lpos.vx, lpos.vy).norm();

		if (fabsf(v_fwd_eng) > POS_ENGAGE_V_MAX || fabsf(v_lat_eng) > POS_ENGAGE_V_LAT_MAX) {
			// Too fast to hold -- REFUSE (step 29). An automatic deceleration
			// phase was TRIED AND REVERTED here (step 30): see decelLoop() for
			// what was measured and why braking with pitch cannot work on this
			// airframe. Until a real back-transition exists, the vehicle has no
			// automatic way home from cruise and the pilot must know that.
			if (!_pos_hold_refused) {
				PX4_WARN("pos_hold REFUSED: v_fwd %.1f (lim %.1f), v_lat %.1f (lim %.1f), v_h %.1f",
					 (double)v_fwd_eng, (double)POS_ENGAGE_V_MAX,
					 (double)v_lat_eng, (double)POS_ENGAGE_V_LAT_MAX, (double)v_h);
				_pos_hold_refused = true;
			}

		} else {
			_pos_sp = matrix::Vector2f(lpos.x, lpos.y);
			_pos_integral_v.setZero();
			_pos_hold_active = true;
			_pos_hold_refused = false;
			PX4_INFO("pos_hold: holding x=%.2f y=%.2f", (double)_pos_sp(0), (double)_pos_sp(1));

			if (_bt_req_pos_hold && _bt_handoff_wait > 0.5f) {
				// The item (S) residual actually cost something on this flight.
				PX4_WARN("back-transition: handoff waited %.1f s for the magnitude gate (v_h=%.2f m/s)",
					 (double)_bt_handoff_wait, (double)v_h);
			}
		}

	} else if (!pos_hold && _pos_hold_active) {
		_pos_hold_refused = false;
		_pos_hold_active = false;
		_pos_att_sp.setZero();
	}

	_alt_accum += dt;

	if (_alt_accum >= ALT_TS - 1e-6f) {
		// Vertical channel, in descending order of what is still measurable
		// (blocker B1). Only the first branch is the flown configuration.
		if (alt_ok) {
			// YER ETKISI DUZELTMESI (2026-08-31, Adim 152).
			//
			// OLCULEN ARIZA: arac son 1-2 metreyi INEMIYOR, yer etkisinde
			// asili kaliyor. Sebep tahsisat degil (komut edilen ve gerceklesen
			// itki birebir ayni, 0.1 N), irtifa dongusune giren HIZ.
			//
			// lpos.vz'nin yere yakin KALICI bir sapmasi var. Olculdu (ULog
			// 15_22_05, vz eksi EKF'in kendi konum turevi z_deriv):
			//   20-45 m: -0.12 | 10-20 m: -0.02 | 5-10 m: +0.15
			//    2-5 m : +0.32 |  1-2 m : +0.40 |  0-1 m: +0.38
			// Sapma yere yaklastikca buyuyor -- ders kitabi yer etkisi imzasi
			// (rotor asagi akisi baro'yu basincliyor). Gercek kartta da olur.
			//
			// NEDEN TAM OLARAK KILITLIYOR: altitude_loop.m'de
			//   vz_sp = Kp_z * (z_sp - z) = 0.6 * 0.665 = 0.399 m/s
			//   err_vz = vz_sp - vz = 0.399 - 0.442 = -0.04
			// Sapma, komut edilen alcalma hizini BIREBIR iptal ediyor: dongu
			// "zaten iniyorum" sanip itkiyi kismiyor. Gercek inis 0.058 m/s.
			//
			// z_deriv (EKF'in konum turevi) bu sapmayi TASIMIYOR ve GURULTUSU
			// AYNI (olculdu, ardisik fark RMS orani 0.9-1.1x uc kosumda).
			//
			// ANI KAYNAK DEGISIMI YAPILMIYOR: 2 m'de vz->z_deriv gecisi
			// Kp_vz * 0.4 = 1.6 m/s^2, yani 8 N'lik bir BASAMAK olurdu.
			// Yerine LAND_DIFF_ALT..0 arasinda dogrusal harman: yukarida saf
			// vz (fuzyonlanmis kestirim, dogru olan o), yerde saf z_deriv.
			float vz_used = lpos.vz;
			const float agl_alt = _z_datum - lpos.z;

			if (PX4_ISFINITE(agl_alt) && PX4_ISFINITE(lpos.z_deriv)) {
				const float w = math::constrain(
						(LAND_DIFF_ALT - agl_alt) / LAND_DIFF_ALT, 0.f, 1.f);
				vz_used = (1.f - w) * lpos.vz + w * lpos.z_deriv;
			}

			_fz_sp = altitudeLoop(z_sp, lpos.z, vz_used, _alt_integral_vz);

		} else {
			// Nothing vertical left to measure. Open-loop, just under hover
			// weight: the vehicle sinks slowly instead of holding an altitude
			// it cannot observe or climbing away on an unknown mass. This is the
			// whole of FsLevel::NO_ALT, and it is the MEASURED one -- step 35's
			// MATLAB run held 0.71 m/s mean descent over 49.5 s on exactly this
			// policy (attitude level, no vertical feedback at all).
			//
			// A THIRD branch used to sit between these two (step 36 removed it):
			// `else if (vz_ok)` -> altitudeLoopVz(FS_DESCENT_VZ, ...), i.e. "z is
			// gone but vz survives, so close the loop on the rate". It was
			// unreachable by construction -- EKF2.cpp:1588-1590 derives z_valid
			// and v_z_valid from the SAME OR, so the module can never see one
			// without the other -- and making it reachable would have meant
			// inventing a distinction the estimator explicitly declines to make
			// (its TODO says consumers mishandle them differing). Removed rather
			// than kept-with-rationale, unlike decelLoop(): decelLoop records a
			// measured NEGATIVE result worth not repeating, whereas this branch
			// was only ever an untested assumption -- and step 35 is what an
			// untested failsafe assumption costs.
			_alt_integral_vz = 0.0f;
			_fz_sp = -MASS * GRAVITY * FS_FZ_OPENLOOP;
		}

		if (_pos_hold_active) {
			const float delta_bar_now = (_u_actual(3) + _u_actual(4) + _u_actual(5)) / 3.0f;
			_pos_att_sp = positionLoop(_pos_sp, matrix::Vector2f(lpos.x, lpos.y),
						   matrix::Vector2f(lpos.vx, lpos.vy), att_now(2),
						   delta_bar_now, _pos_integral_v, _pos_fx_trim);

		}

		// --- cruise energy management (this airframe's TECS, steps 47/49/53) ---
		// Placed HERE, inside the altitude decimator, because the pitch trim's input
		// is _fz_sp: reading it from the FT block above would feed the loop the
		// PREVIOUS tick's value.
		//
		// Neither loop needs a state machine of its own. The speed loop CLOSES only
		// in FT CRUISE and TRACKS the ramp otherwise (bumpless handover -- an fx
		// step is a tilt transient on this airframe), and the pitch trim's own
		// 13 m/s gate makes it exactly zero in every other regime, including all of
		// the RAMP below that speed.
		if (_ft_state != FtState::IDLE) {
			// bt_v_fwd is the SIGNED body forward speed (item (S)); both loops read
			// the axis they control, never the magnitude.
			// FT_CRUISE_V_SP (step 99), NOT TECS_V_SP -- see its declaration:
			// FtState::CRUISE gets its own, lower speed target so an FT run
			// that never calls force_fw cannot overshoot into the ~17-20 m/s
			// regime where the wing-tilt differential trim saturates.
			_cruise_fx_cmd = cruiseSpeedLoop(_ft_state == FtState::CRUISE, bt_v_fwd,
							 _ft_fx_cmd, _cruise_fx_I, FT_CRUISE_V_SP);

			// The pitch trim needs alt_ok, and not for the usual reason: with the
			// altitude loop open (FsLevel::NO_ALT) _fz_sp is not a measurement at
			// all but the fixed FS_FZ_OPENLOOP constant, so the error this law
			// integrates would be a pure artefact of the failsafe -- it would read
			// a -47.6 N constant as "the actuators are overloaded" and walk the
			// nose up to the limit. A trim loop must not run on a signal that
			// carries no information.
			_cruise_pitch_sp = alt_ok ? cruisePitchLoop(bt_v_fwd, _fz_sp, _cruise_pitch_I) : 0.0f;

			if (!alt_ok) {
				_cruise_pitch_I = 0.0f;
			}

		} else {
			_cruise_fx_I = 0.0f;
			_cruise_pitch_I = 0.0f;
			_cruise_fx_cmd = 0.0f;
			_cruise_pitch_sp = 0.0f;
		}

		_alt_accum -= ALT_TS;
	}

	// The cruise layer owns fx and pitch while a forward transition is running.
	// It sits BETWEEN the FT block and the BT/pos_hold blocks on purpose: it
	// REPLACES the FT's open-loop fx (bumplessly, so the handover at RAMP ->
	// CRUISE is exactly continuous) and ADDS the trim to the FT's pitch, while the
	// back-transition's brake and the position hold still override both below --
	// the priority order is unchanged from step 42.
	// _ft_pitch_sp is structurally zero (step 29); it is written out rather than
	// dropped so that the relationship stays visible if that ever changes.
	if (_ft_state != FtState::IDLE) {
		fx_sp = _cruise_fx_cmd;
		pitch_sp = _ft_pitch_sp + _cruise_pitch_sp;
	}

	// The back-transition owns pitch while braking (BRAKE/HANDOFF). It is applied
	// BEFORE the pos_hold block on purpose: once the hold actually engages, the
	// position loop takes roll/pitch over and the brake law steps aside -- which
	// is exactly the flown handoff.
	if (_bt_state == BtState::BRAKE || _bt_state == BtState::HANDOFF) {
		roll_sp = 0.0f;
		pitch_sp = _bt_pitch_sp;
		fx_sp = 0.0f;
	}

	// The horizontal loop OWNS roll/pitch while it is active (report item (N)),
	// and supplies the Fx trim that keeps the tilt channels off their floor
	// (report item (P) -- structurally dependent on (N), see positionLoop()).
	if (_pos_hold_active) {
		roll_sp = _pos_att_sp(0);
		pitch_sp = _pos_att_sp(1);
		fx_sp = _pos_fx_trim;
	}

	// Dynamic pressure -- needed by both branches below (the WLS surface columns
	// AND fixedWingControlLaw()'s own internal qbar use body-forward speed
	// instead, but this one still feeds status.qbar either way).
	// GROUND speed, not a pitot: in still air they are identical, and SITL has no
	// wind, so this is exact here and a documented approximation on hardware --
	// deliberately preferred over adding a pitot dependency (and its blocked-tube
	// failure mode) to the very first flight of this path. A real airspeed source
	// must replace it before hardware; until then the error IS the wind speed.
	const float v_air = matrix::Vector3f(lpos.vx, lpos.vy, lpos.vz).norm();
	const float qbar = 0.5f * RHO_AIR * v_air * v_air;
	const float delta_bar = (_u_actual(3) + _u_actual(4) + _u_actual(5)) / 3.0f;

	matrix::Vector<float, N_ACT> u_cmd;
	bool sat_flag[N_ACT] = {};
	matrix::Vector<float, 5> nu_des; nu_des.setZero();
	GainSchedule sched{};
	matrix::Vector3f omega_sp{0.f, 0.f, 0.f};

	if (_fw_state != FwState::IDLE) {
		// === FIXED-WING MODE (Adim 59-61 port): bypass gain-scheduling, the
		// outer attitude P loop, the INDI rate loop, LESO and the WLS allocator
		// entirely -- see fixedWingControlLaw()'s header in TiltrotorIndiControl.hpp
		// for why (TECS_PITCH_FZ_SP's "zero thrust is not a target, it is a
		// cliff" note two files up is the SAME conclusion, independently
		// measured, for why the WLS path cannot be reused here). ===
		u_cmd.setZero();
		u_cmd(3) = _fw_tilt; u_cmd(4) = _fw_tilt;
		// Tail tilt parked VERTICAL (TILT_MIN), not forward -- Adim 69: kept
		// ready throughout GLIDE (where T2 stays 0, unchanged) so its thrust
		// is immediately usable as a pitch backstop the instant ACTIVE begins,
		// with the SAME up-thrust-at-a-rearward-moment-arm mechanism this
		// airframe's hover mode already uses for pitch (see FW_TAIL_PITCH_LIMIT's
		// header note). u_cmd(2) (tail thrust) stays 0 from setZero() above
		// unless ACTIVE's backstop engages below.
		u_cmd(5) = TILT_MIN;

		float surf[N_SURF] = {0.f, 0.f, 0.f, 0.f, 0.f};

		if (_fw_state == FwState::GLIDE || _fw_state == FwState::RETURN) {
			// Unpowered glide (T0=T1=T2=0, already zeroed above): the Adim 58
			// aileron-sign-fixed law, PD on roll since Adim 67 (see
			// FW_GLIDE_KD_ROLL's header note -- the coordinated-flight gate
			// keeps entry small, but a brief unpowered phase still deserves
			// its own damping). Sign per effectivenessMatrix()'s own
			// SURF_K/SURF_CPY: tau_x = -1.2*qbar*a_ail for a differential
			// elevon, so a_ail = +Kp*phi (NOT -Kp*phi) corrects a positive
			// roll error; the rate term carries the same sign (same
			// derivation as fixedWingControlLaw()'s a_ail below).
			// Rudder removed (Adim 77) -- same as ACTIVE, see FW_MAX_BANK's
			// header note in TiltrotorIndiParams.hpp; the brief GLIDE phase
			// just holds wings level (roll_sp implicitly 0) and lets ACTIVE's
			// bank-to-turn own heading once it takes over.
			//
			// RETURN (step 103) reuses this SAME law unchanged, tilt just
			// slews the other way (see fixedWingTransition()) -- it needs the
			// identical thing GLIDE does: hold attitude with surfaces alone
			// while unpowered and mid-tilt, nothing about that job depends on
			// which direction tilt is moving.
			const float a_ail =  FW_GLIDE_KP_ROLL  * att_now(0) + FW_GLIDE_KD_ROLL * omega(0);
			const float a_ele = -FW_GLIDE_KP_PITCH * att_now(1);
			surf[0] = a_ail; surf[1] = -a_ail;
			surf[2] = a_ele; surf[3] =  a_ele;
			surf[4] = 0.0f;

		} else { // FwState::ACTIVE
			// hdg_sp is _fw_yaw_entry (bumplessly captured on the IDLE->GLIDE
			// edge, Adim 68) -- fixedWingControlLaw() now derives roll (bank)
			// from THIS internally (Adim 75's bank-to-turn), so there is no
			// separate external roll_sp input any more.
			//
			// TEST HOOK (`fw_hdg` console command, see _fw_hdg_override_mdeg's
			// declaration): a live heading-change request, applied here so it
			// only ever touches _fw_yaw_entry in ACTIVE, never GLIDE's own
			// wings-level hold.
			const int32_t hdg_override_mdeg = _fw_hdg_override_mdeg.load();

			if (hdg_override_mdeg != INT32_MIN) {
				_fw_yaw_entry = (float)hdg_override_mdeg / 1000.0f * (float)M_PI / 180.0f;
				_fw_hdg_override_mdeg.store(INT32_MIN);
			}

			float T_wing = 0.0f, T_tail = 0.0f;
			fixedWingControlLaw(_fw_yaw_entry, z_sp, FW_V_SP, att_now, omega,
					     lpos.z, bt_v_fwd, dt, _fw_v_I, _fw_alt_I, _fw_v_fwd_filt,
					     T_wing, T_tail, surf);
			u_cmd(0) = T_wing; u_cmd(1) = T_wing; u_cmd(2) = T_tail;
		}

		for (int j = 0; j < N_SURF; j++) { u_cmd(6 + j) = surf[j]; }

	} else {

	// --- 1) gain-scheduling from the shadow tilt state (gain_schedule.m) ---
	sched = computeGainSchedule(delta_bar);

	// --- 2) outer attitude P loop ---
	// Unconditional. It used to be skipped under FsLevel::RATE_ONLY, leaving
	// omega_sp = 0 and the inner loop as pure rate damping; step 35 measured that
	// configuration and it does not fly, so attitude loss cuts the output further
	// up instead of arriving here (see the hard-prerequisite check in Run()).
	const matrix::Vector3f att_sp(roll_sp, pitch_sp, yaw_sp);
	matrix::Vector3f e_att = att_sp - att_now;
	e_att(2) = atan2f(sinf(e_att(2)), cosf(e_att(2))); // yaw error wrap

	omega_sp = sched.kp_att.emult(e_att);

	for (int i = 0; i < 3; i++) {
		omega_sp(i) = math::constrain(omega_sp(i), -RATE_SP_LIMIT[i], RATE_SP_LIMIT[i]);
	}

	// --- 3) inner INDI rate loop: desired angular acceleration ---
	const matrix::Vector3f e_omega = omega_sp - omega;
	const matrix::Vector3f omega_dot_des = sched.kp_rate.emult(e_omega);

	// --- 4) LESO update, decimated at LESO_TS, active axes only ---
	_leso_accum += dt;

	if (_leso_accum >= LESO_TS - 1e-6f) {
		for (int ax = 0; ax < 3; ax++) {
			if (leso_enable[ax]) {
				_leso[ax].update(omega(ax), _prev_u_leso(ax), LESO_TS);
				_d_hat(ax) = _leso[ax].z2;
			}
		}

		_leso_accum -= LESO_TS;
	}

	matrix::Vector3f d_hat_active;

	for (int ax = 0; ax < 3; ax++) {
		d_hat_active(ax) = leso_enable[ax] ? _d_hat(ax) : 0.0f;
	}

	const matrix::Vector3f omega_dot_des_adj = omega_dot_des - d_hat_active;
	_prev_u_leso = omega_dot_des_adj; // ESO "input" must be the post-compensation value, see leso_axis_update.m note

	// --- 5) INDI incremental control law ---
	const matrix::Vector3f domega_dot = omega_dot_des_adj - omega_dot_meas;
	const matrix::Vector3f dtau(I_XX * domega_dot(0), I_YY * domega_dot(1), I_ZZ * domega_dot(2));

	// --- 6) WLS control allocation ---
	matrix::Matrix<float, 5, N_ACT> G;
	matrix::Vector<float, 5> nu0;

	effectivenessMatrix(_u_actual, qbar, G, nu0);

	nu_des(0) = dtau(0); nu_des(1) = dtau(1); nu_des(2) = dtau(2);
	nu_des(3) = fx_sp - nu0(3);
	nu_des(4) = _fz_sp - nu0(4);

	// TEMASTA MOMENT ARTIMI KESILIR (Adim 119) -- MATLAB birebir portu.
	// Adim 118 olctu ki tek tek CIKIS kanallarini kirpmak sarmayi tahsisatin
	// icinde KOVALIYOR: kanat itki farki sifirlandiginda ayni sarma TILT
	// kanalina gocuyor (T0 = T1 tam esitken d0/d1 ~4.6 deg/s rampa yapti,
	// tilt farki 7.6 -> 38.1 deg, T2 sifir rayinda, yaw kacti). Kusur cikista
	// degil ARTIMDA: yerde omega_dot_meas ~ 0 kaldigi icin dtau hic sonmuyor
	// ve WLS onu hangi aktuator ucuzsa oraya biriktiriyor.
	//
	// YALNIZCA MOMENT: temasta arac AGIRLIK MERKEZI etrafinda degil TEMAS
	// NOKTASI etrafinda doner, yani etkinlik matrisinin moment satirlari o
	// rejimde gecersizdir (gerekce zaten LAND_DIFF_MAX notunda). Kuvvet
	// satirlari gecerlidir; Fz'yi de kesmek irtifa dongusunun yerde itkiyi
	// azaltmasini engeller ve Adim 110'un kok gozlemini kaliciLastirirdi.
	if (_land_contact_latch) {
		nu_des(0) = 0.0f; nu_des(1) = 0.0f; nu_des(2) = 0.0f;
	}

	matrix::Vector<float, N_ACT> du_min, du_max;

	for (int i = 0; i < 3; i++) {
		const float abs_lo = ROTOR_TMIN - _u_actual(i);
		const float abs_hi = ROTOR_TMAX - _u_actual(i);
		// TS_BOX (not the measured dt) on purpose -- see TiltrotorIndiParams.hpp.
		// Note this is NOT numerically neutral, unlike the tilt box below:
		//   lower  -ROTOR_TMAX/ROTOR_TAU_UP  *TS*5 = -45 N (old) / -72 N (TS_BOX)
		//   upper   ROTOR_TMAX/ROTOR_TAU_DOWN*TS*5 = +22.5 N (old) / +36 N (TS_BOX)
		// The lower bound never mattered (abs_lo = -_u_actual >= -45 always wins),
		// but the old UPPER bound of 22.5 N/tick did sit below abs_hi at hover
		// (45 - 18 = 27 N), so it was a live constraint that the shorter nominal
		// period had under-sized -- same root cause as the tilt box. It was never
		// actually reached though: measured thrust sat_flag = 0.0% of samples both
		// before and after, so this relaxes an inactive constraint.
		const float rate_lo = -ROTOR_TMAX / ROTOR_TAU_UP * TS_BOX * 5.0f;
		const float rate_hi = ROTOR_TMAX / ROTOR_TAU_DOWN * TS_BOX * 5.0f;
		du_min(i) = fmaxf(abs_lo, rate_lo);
		du_max(i) = fminf(abs_hi, rate_hi);
	}

	// Wing tilt ceiling. Three independent sources, whichever is lower wins:
	//   - the back-transition state machine (_bt_tilt_ceil, TILT_MAX when idle)
	//   - the `tiltceil` console hook (_tilt_ceiling_mdeg, 90 deg by default),
	//     kept so the mechanism can still be swept by hand inside one flight
	//     the way slewbox is -- that is how BT_CEIL_FLOOR was chosen.
	//   - the `ftceil` console hook (_ft_tilt_ceiling_mdeg, step 97), but ONLY
	//     while `_ft_state != FtState::IDLE` -- see its declaration above for
	//     why (forward-transition tilt-divergence lockup/tumbling). Gated on
	//     _ft_state, not folded into _tilt_ceiling_mdeg, so it cannot affect
	//     hover, back-transition, or fixed-wing GLIDE/ACTIVE (which does not
	//     use this box at all).
	// All default to TILT_MAX, making this identical to the previous
	// `TILT_MAX - _u_actual(3 + i)`.
	const float ft_tilt_ceil = (_ft_state != FtState::IDLE)
				    ? (float)_ft_tilt_ceiling_mdeg.load() / 1000.0f * (float)M_PI / 180.0f
				    : TILT_MAX;
	const float tilt_ceil = fminf(_bt_tilt_ceil,
				      fminf(TILT_MAX,
					    fminf(ft_tilt_ceil,
						  (float)_tilt_ceiling_mdeg.load() / 1000.0f * (float)M_PI / 180.0f)));

	for (int i = 0; i < 3; i++) {
		// i == 2 is the tail rotor: deliberately NOT ceiling/floor-limited by the
		// BACK-TRANSITION machine (step 31 for the ceiling; the floor, step 101,
		// follows the same reasoning -- it exists to protect the WING rotors'
		// edgewise loading against the direction of travel, which the tail rotor
		// is not doing).
		// IT DOES HAVE ITS OWN, PHYSICAL CEILING THOUGH (step 133): the 0.10 m
		// disc at 90 deg sweeps through the tail boom. TAIL_TILT_MAX = 20 deg,
		// which is 8x the largest tail tilt measured in a full mission (2.5 deg).
		const float tilt_lo = (i < 2 ? _bt_tilt_floor : TILT_MIN);
		const float abs_lo = tilt_lo - _u_actual(3 + i);
		const float abs_hi = (i < 2 ? tilt_ceil : TAIL_TILT_MAX) - _u_actual(3 + i);
		// Allocator box, NOT the physical servo limit (that is TILT_RATE_MAX, used
		// only by the shadow model below). Default 1.25 * (1/250) = 0.005 rad/tick,
		// which equals the previous 2.0 * (1/400) to within 1 ULP in float
		// (0.005 vs 0.0050000004, i.e. 4.7e-10 rad) -- deliberately neutral.
		// See TiltrotorIndiParams.hpp for why the split was needed, and the
		// `slewbox` test hook above for sweeping it inside a single flight.
		const float box_rate = (float)_slew_box_rate_mrs.load() / 1000.0f;
		const float rate_lo = -box_rate * TS_BOX;
		const float rate_hi = box_rate * TS_BOX;
		du_min(3 + i) = fmaxf(abs_lo, rate_lo);
		du_max(3 + i) = fminf(abs_hi, rate_hi);

		// TEST HOOK (`tiltjerk` console command, see _tilt_jerk_limit_mrs's
		// declaration above): one derivative higher than the rate box -- bounds
		// how far this tick's du may sit from LAST tick's du, so the rate
		// itself cannot flip from one box extreme to the other in a single
		// tick. Neutral (does not further constrain du_min/du_max) whenever
		// the jerk limit is wide enough to already contain the full box swing.
		{
			const float jerk_rate = (float)_tilt_jerk_limit_mrs.load() / 1000.0f;
			const float jerk_max = jerk_rate * TS_BOX;
			du_min(3 + i) = fmaxf(du_min(3 + i), _prev_du_tilt(i) - jerk_max);
			du_max(3 + i) = fminf(du_max(3 + i), _prev_du_tilt(i) + jerk_max);
		}

		// The ceiling must be enforced THROUGH the slew limit, never instead of
		// it. Once the ceiling sits below the current tilt, abs_hi goes negative
		// and can fall below rate_lo, i.e. the box becomes EMPTY. The generic
		// `du_min = fminf(du_min, du_max)` guard below would then resolve that by
		// dragging du_min down onto abs_hi -- commanding the whole remaining
		// overshoot in a SINGLE tick (e.g. -0.1 rad against a 0.012 rad box, 8x
		// the slew limit). That guard was written when this could not happen:
		// with abs_hi = TILT_MAX - u >= 0 the tilt box always contained 0.
		// Resolve the other way instead -- collapse to rate_lo, i.e. retract at
		// exactly the allocator slew rate until the tilt is back under the
		// ceiling. This keeps the retraction rate-limited and observable.
		if (du_max(3 + i) < du_min(3 + i)) {
			du_max(3 + i) = du_min(3 + i);
		}
	}

	// Surface box (item (V), step 45). Two terms, same shape as the tilt box:
	// the absolute joint limit and the servo slew over TS_BOX. Using TS_BOX
	// rather than the measured dt is deliberate and for the same reason as the
	// tilt -- step 21 measured that sizing a box with a nominal period the module
	// does not actually run at silently shrinks the real authority to 62%.
	for (int j = 0; j < N_SURF; j++) {
		const float rate = SURF_RATE_MAX * TS_BOX;
		du_min(6 + j) = fmaxf(-SURF_MAX[j] - _u_actual(6 + j), -rate);
		du_max(6 + j) = fminf(SURF_MAX[j] - _u_actual(6 + j),  rate);

		if (du_max(6 + j) < du_min(6 + j)) { du_max(6 + j) = du_min(6 + j); }
	}

	for (int i = 0; i < N_ACT; i++) {
		du_min(i) = fminf(du_min(i), du_max(i)); // numerical safety, see wls_allocate.m
	}

	matrix::Vector<float, 5> Ws;
	Ws(0) = WS_ROLL; Ws(1) = WS_PITCH; Ws(2) = WS_YAW; Ws(3) = WS_FX; Ws(4) = WS_FZ;

	matrix::Vector<float, N_ACT> Wu;
	Wu(0) = 1.0f; Wu(1) = 1.0f; Wu(2) = 1.0f;
	Wu(3) = sched.wu_tilt[0]; Wu(4) = sched.wu_tilt[1]; Wu(5) = sched.wu_tilt[2];

	// Surfaces: a CONSTANT weight, because their |G| already scales with qbar --
	// so the penalty per unit torque falls as speed rises and the handover is the
	// crossing of a ratio, not a mode switch. See WU_SURF for the derivation.
	for (int j = 0; j < N_SURF; j++) { Wu(6 + j) = WU_SURF; }

	const matrix::Vector<float, N_ACT> du = wlsAllocate(G, nu_des, du_min, du_max, Ws, Wu, sat_flag, _wls_scratch);

	// --- GOLGE: uretilen kod ayni girdilerle kosar (Adim 131) ---
	// Yalnizca ILK ALTI aktuator: uretilen fonksiyon 6-aktuatorlu (uçan
	// yapilandirmada SURF_ENABLE=false, Adim 125). C++ 11 ile cozer ama yuzey
	// sutunlari sifirdir, yani ilk altinin ayni cikmasi BEKLENIR.
	// double: uretilen kod cift duyarlik kullanir (Cortex-M7'de FPU var).
	{
		double dtau_d[3] = { (double)nu_des(0), (double)nu_des(1), (double)nu_des(2) };
		// F_sp geri cozulur: nu_des(3:4) = F_sp - nu0(3:4)
		double fsp_d[2]  = { (double)(nu_des(3) + nu0(3)), (double)(nu_des(4) + nu0(4)) };
		double u6_d[6], ucmd_d[6], sat_d[6], st_out[5];
		for (int i = 0; i < 6; i++) { u6_d[i] = (double)_u_actual(i); }
		// agl/roll/pdot: C++ tarafindaki AYNI degerler. Kapi kapaliysa
		// (agl gecersiz) uretilen kod da kapatir -- ayni dal.
		const double agl_d  = PX4_ISFINITE(_z_datum) ? (double)(_z_datum - lpos.z)
				      : (double)NAN;
		const double roll_d = (double)att_now(0);
		const double pdot_d = (double)omega_dot_meas(0);
		sf_wls_alloc(dtau_d, fsp_d, u6_d, agl_d, roll_d, pdot_d,
			     _cg_state, ucmd_d, sat_d, st_out);
		for (int i = 0; i < 5; i++) { _cg_state[i] = st_out[i]; }

		float dmax = 0.0f;
		for (int i = 0; i < 6; i++) {
			const float du_cg = (float)(ucmd_d[i] - u6_d[i]);
			dmax = fmaxf(dmax, fabsf(du_cg - du(i)));
		}
		_cg_du_diff = dmax;
	}

	// Carried to the NEXT tick for `tiltjerk`'s du-vs-du box above.
	for (int i = 0; i < 3; i++) { _prev_du_tilt(i) = du(3 + i); }

	// Carried to the NEXT tick for the forward transition, which runs before this
	// point and therefore cannot see the current solve (item (V), step 42).
	_sat_thrust_prev = sat_flag[0] || sat_flag[1] || sat_flag[2];

	// YWdbg (2026-07-27, step 12): yaw-axis closed-loop attribution. Separates
	// "the allocator cannot deliver the demanded yaw torque" (ach2 << nu2) from
	// "the demand itself is wrong/self-defeating" (ach2 ~ nu2 but the vehicle
	// keeps spinning -- e.g. the outer attitude loop commanding +-RATE_SP_LIMIT
	// as the heading wraps). Critical fields first: PX4_INFO truncates long
	// lines (step 11 lost a Wu1=1000000 to truncation that way).
	{
		static int yw_cnt = 0;

		if ((yw_cnt++ % 250) == 0) {
			const matrix::Vector<float, 5> ach = G * du;
			PX4_INFO("YWdbg nu2=%.3f ach2=%.3f r=%.2f wsp2=%.2f ddelta=[%.4f %.4f %.4f]",
				 (double)nu_des(2), (double)ach(2), (double)omega(2), (double)omega_sp(2),
				 (double)du(3), (double)du(4), (double)du(5));
		}
	}

	u_cmd = _u_actual + du;

	} // else (INDI/LESO/WLS path)

	// Final safety clamp -- shared by both branches. In the WLS path this is
	// mostly redundant (du is already box-constrained); in FIXED-WING mode it is
	// the ONLY clamp applied to the law's raw PD/feed-forward outputs.
	for (int i = 0; i < 3; i++) { u_cmd(i) = math::constrain(u_cmd(i), ROTOR_TMIN, ROTOR_TMAX); }

	// YERE YAKIN KANAT ITKI FARKI SINIRI -- MATLAB indi_attitude_controller.m'in
	// birebir portu; gerekce TiltrotorIndiParams.hpp LAND_DIFF_MAX notunda.
	// ORTALAMA korunur (dikey kanal etkilenmez), yalnizca FARK sinirlanir.
	//
	// alt_ok SART: bayat/gecersiz irtifayla kapiyi acmak, mekanizmayi ait
	// olmadigi bir rejimde devreye sokar. Gecersizse mekanizma kapali kalir,
	// yani davranis degisiklikten onceki haldir -- guvenli taraf budur.
	//
	// Bu blok FIXED-WING yolunu da paylasan son kirpmanin icinde duruyor ama
	// onu ETKILEYEMEZ: FW girisi FW_MIN_ALT = 30 m istiyor, bu kapi ise 2 m.
	//
	// IRTIFA KAYNAGI (Adim 116): `-lpos.z` DEGIL. O, EKF yerel orijininden
	// yukseklik; olculen datum ofseti +1.77 m'ye kadar cikiyor, yani 2.0 m'lik
	// esikle AYNI MERTEBEDE ve kapi rastgele armaniyordu. Tam gerekce
	// captureGroundDatum()'da. MATLAB referansinin `agl` argumaninin anlami
	// zaten buydu -- burasi portu o sozlesmeye geri getiriyor.
	// Datum yoksa agl NAN olur ve kapi KAPALI kalir: alt_ok'in zaten aldigi
	// guvenli taraf. Bu, degisiklikten onceki davranis DEGIL (o zaman kapi
	// yanlis sinyalle acilabiliyordu) -- bilerek boyle.
	const float agl = _z_datum - lpos.z;

	if (LAND_DIFF_ENABLE && alt_ok && PX4_ISFINITE(agl) && agl < LAND_DIFF_ALT) {
		const float t_mean = 0.5f * (u_cmd(0) + u_cmd(1));
		// TEMAS: sinir 0'a iner, kanat farki TAMAMEN silinir. 10 N'lik sinir tek
		// basina YETMEDI (ULog 11_04_57): fark sinira DOYUP kaldi (ort +9.23 N,
		// 3490 ornekte 5 isaret degisimi = olu birikim), kalici yaw momenti
		// araci dondurdu ve egik rotorlarin yatay bileseni daire cizdirdi --
		// 2.9 m surukleme, temasta 3.77 m/s yanal hiz ve -134 deg/s ile vurus.
		// Temasta arac AGIRLIK MERKEZI etrafinda degil TEMAS NOKTASI etrafinda
		// doner, yani effectiveness matrisin dayandigi model gecersizdir ve geri
		// besleme duzeltmez, besler. Dogru davranis momenti KESMEKTIR.
		// Ikinci temas olcutu (Adim 118): duz iniste roll ~0.2 derecede donuyor,
		// yani ustteki roll dali hic atesLenmiyor ve fark 10 N'de DOYUP kaliyor.
		// Mandal, "buyuk fark komut edildi ama acisal ivme olusmadi" kosulunun
		// LAND_CONTACT_DWELL kadar kesintisiz surmesiyle kurulur. Gerekce,
		// olculen ayrim ve neden Adim 109'un elenen olcutu OLMADIGI:
		// TiltrotorIndiParams.hpp, LAND_CONTACT_DIFF notu.
		const bool contact = (fabsf(att_now(0)) > LAND_CONTACT_ROLL) || _land_contact_latch;
		const float d_lim  = contact ? 0.0f : (0.5f * LAND_DIFF_MAX);
		const float t_diff = math::constrain(0.5f * (u_cmd(0) - u_cmd(1)), -d_lim, d_lim);
		u_cmd(0) = math::constrain(t_mean + t_diff, ROTOR_TMIN, ROTOR_TMAX);
		u_cmd(1) = math::constrain(t_mean - t_diff, ROTOR_TMIN, ROTOR_TMAX);

		// Sayac GONDERILEN farkla guncellenir; bir tick gecikmeli degerlendirme
		// (bu tick'in olcumu bir sonraki tick'in d_lim'ini kurar).
		if ((fabsf(u_cmd(0) - u_cmd(1)) > LAND_CONTACT_DIFF)
		    && (fabsf(omega_dot_meas(0)) < LAND_CONTACT_ACC)) {
			_land_contact_dwell += dt;

			if (!_land_contact_latch && (_land_contact_dwell >= LAND_CONTACT_DWELL)) {
				_land_contact_latch = true;
				PX4_INFO("land contact latched (diff %.1f N, no roll accel)",
					 (double)fabsf(u_cmd(0) - u_cmd(1)));
			}

		} else {
			_land_contact_dwell = 0.0f;
		}

		// DIKEY ITKI TAVANI (Adim 145) -- gerekce ve olculen esik ayrimi
		// TiltrotorIndiParams.hpp, LAND_TZ_MAX notunda. Tahsisat momenti
		// kovalarken kuvvet komutunu cignemesin diye NET KALDIRMA kisilir;
		// uc rotor birlikte olceklendigi icin moment ORANLARI korunur.
		const float ctz = u_cmd(0) * cosf(u_cmd(3)) + u_cmd(1) * cosf(u_cmd(4))
				  + u_cmd(2) * cosf(u_cmd(5));
		const float tz_cap = LAND_TZ_MAX * fabsf(_fz_sp);

		if (PX4_ISFINITE(ctz) && PX4_ISFINITE(tz_cap) && (ctz > tz_cap) && (ctz > 1e-3f)) {
			const float k = tz_cap / ctz;

			for (int i = 0; i < 3; i++) {
				u_cmd(i) = math::constrain(u_cmd(i) * k, ROTOR_TMIN, ROTOR_TMAX);
			}
		}

	} else {
		// Kapi kapali: ucus, veya datum yok. Mandal ve sayac temizlenir --
		// arac 2 m'nin ustune cikinca ucusa donus otomatiktir.
		_land_contact_dwell = 0.0f;
		_land_contact_latch = false;
	}

	// Kuyruk rotorunun tavani AYRI (adim 133): TAIL_TILT_MAX = 20 deg.
	// Gerekce TiltrotorIndiParams.hpp'de -- pervane diski 90 derecede kuyruk
	// cubugunun icinden geciyor, ve motoru yukseltmek geri gecisi bozdu.
	for (int i = 0; i < 2; i++) { u_cmd(3 + i) = math::constrain(u_cmd(3 + i), TILT_MIN, TILT_MAX); }
	u_cmd(5) = math::constrain(u_cmd(5), TILT_MIN, TAIL_TILT_MAX);

	for (int j = 0; j < N_SURF; j++) { u_cmd(6 + j) = math::constrain(u_cmd(6 + j), -SURF_MAX[j], SURF_MAX[j]); }

	// --- publish actuator outputs ---
	actuator_motors_s motors{};
	motors.timestamp_sample = angular_velocity.timestamp_sample;

	for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; i++) { motors.control[i] = NAN; }

	// Quadratic inverse of the Gazebo motor model, NOT a linear u_cmd/ROTOR_TMAX
	// -- see thrustToNormalized() for why (step 11 bug fix).
	for (int i = 0; i < 3; i++) { motors.control[i] = thrustToNormalized(u_cmd(i)); }

	motors.reversible_flags = 0;
	motors.timestamp = hrt_absolute_time();
	_actuator_motors_pub.publish(motors);

	// Land-detector telemetry only (see header comment on _vehicle_thrust_setpoint_pub):
	// mean normalized rotor thrust as a body-frame -Z (up) setpoint, matching
	// what MulticopterLandDetector expects from the normal control_allocator path.
	{
		const float mean_thrust_norm = (motors.control[0] + motors.control[1] + motors.control[2]) / 3.0f;
		vehicle_thrust_setpoint_s thrust_sp{};
		thrust_sp.timestamp_sample = angular_velocity.timestamp_sample;
		thrust_sp.xyz[0] = 0.0f;
		thrust_sp.xyz[1] = 0.0f;
		thrust_sp.xyz[2] = -mean_thrust_norm;
		thrust_sp.timestamp = hrt_absolute_time();
		_vehicle_thrust_setpoint_pub.publish(thrust_sp);
	}

	actuator_servos_s servos{};
	servos.timestamp_sample = angular_velocity.timestamp_sample;

	for (int i = 0; i < actuator_servos_s::NUM_CONTROLS; i++) { servos.control[i] = NAN; }

	// Control surfaces (indices 0-4). Held at 0 unless SURF_ENABLE (the OLD,
	// twice-failed MC-mode WLS surface allocation, see SURF_ENABLE's header --
	// stays disabled) OR the fixed-wing control law (Adim 59-83) is driving them,
	// which is a completely separate, independently MATLAB-validated code path
	// that also writes u_cmd(6..10). Before Adim 84b this loop gated BOTH on the
	// same SURF_ENABLE=false, so the entire fixed-wing aileron/elevator law
	// (Adim 66-83) was silently discarded here -- actuator_servos.control[0]
	// measured EXACTLY 0.0 for the full duration of every SITL FW test that day,
	// even as roll grew to the recurring ~57-60 deg attractor. That attractor is
	// the airframe's own passive/uncontrolled trim with zero aileron authority,
	// not a control-law defect -- see WLS_LOCKUP_INVESTIGATION_REPORT.md Adim 84b.
	// The gz bridge maps control=+-1 onto SIM_GZ_SV_MINA/MAXA, whose default is
	// +-57.29578 deg = EXACTLY +-1 rad -- so control = deflection in radians,
	// with no scale factor and no param change needed. (Verified against the
	// airframe file: MINA/MAXA are set only for servos 6-8, the tilts.)
	const bool fw_surf_active = (_fw_state != FwState::IDLE);
	for (int j = 0; j < 5; j++) {
		servos.control[j] = (SURF_ENABLE || fw_surf_active)
				    ? math::constrain(u_cmd(6 + j), -SURF_MAX[j], SURF_MAX[j])
				    : 0.0f;
	}

	// tilt servos (indices 5-7): SIM_GZ_SV_MINA/MAXA map control=-1 -> TILT_MIN, +1 -> TILT_MAX
	for (int i = 0; i < 3; i++) {
		const float norm = -1.0f + 2.0f * (u_cmd(3 + i) - TILT_MIN) / (TILT_MAX - TILT_MIN);
		servos.control[5 + i] = math::constrain(norm, -1.0f, 1.0f);
	}

	servos.timestamp = hrt_absolute_time();
	_actuator_servos_pub.publish(servos);

	// --- shadow actuator model: integrate our own copy toward u_cmd (tiltrotor_plant_deriv.m Tdot/ddelta) ---
	for (int i = 0; i < 3; i++) {
		const float tau = (u_cmd(i) >= _u_actual(i)) ? ROTOR_TAU_UP : ROTOR_TAU_DOWN;
		_u_actual(i) += dt * (u_cmd(i) - _u_actual(i)) / tau;
	}

	for (int i = 0; i < 3; i++) {
		// STICTION: TRIED AND REVERTED (2026-07-28, step 24). Kept as a warning.
		//
		// The real gz tilt servo is a
		// JointPositionController (p_gain=100, i=d=0, cmd_max=2 Nm) driving a joint
		// with <friction>1.0</friction>. It therefore CANNOT MOVE AT ALL until the
		// position error is big enough to break Coulomb friction:
		//     |err| >= friction / p_gain = 1.0 / 100 = 0.01 rad = 0.573 deg
		// Without this the shadow model creeps to the command while the real joint
		// stays parked inside the friction band -- exactly the persistent 0.25-0.55
		// deg offset measured in steps 18 and 21 (shadow read *exactly* 0.000 while
		// the real joints rested at 0.52-0.53 deg, which made the WLS box constraint
		// abs_lo = TILT_MIN - _u_actual hide ~0.53 deg of real travel on yaw's only
		// actuator).
		//
		// Validated OFFLINE first, replaying the recorded u_cmd against the recorded
		// real joint angles (sitl/servo_model.py), shadow-vs-real RMS error:
		//        current 1st-order | full 2nd-order | 1st-order + this deadband
		//   d0        0.287 deg    |   0.414 deg    |   0.082 deg   (3.5x better)
		//   d1        0.408 deg    |   0.462 deg    |   0.051 deg   (8.0x better)
		//   d2        0.554 deg    |   0.553 deg    |   0.0040 deg  (139x better)
		// Note the full torque-limited SECOND-ORDER model is WORSE, not better: the
		// joint inertia (J = 0.0168 kg m^2, max accel (2-1)/J = 59.4 rad/s^2) settles
		// in a few ms, far inside one 4 ms tick, so the dynamics are irrelevant at
		// this loop rate. The whole fidelity gap was the friction deadband.
		//
		// WHY IT WAS REVERTED -- it DEADLOCKS the loop, and only a closed-loop test
		// could show it. `u_cmd = _u_actual + du`, so the command is anchored to the
		// shadow state, while `du` is capped by the allocator slew box at
		// TILT_SLEW_BOX_RATE*TS_BOX = 1.75*0.004 = 0.007 rad = 0.40 deg -- SMALLER
		// than the 0.573 deg deadband. So no single tick can ever break stiction,
		// the shadow freezes, and because u_cmd is built from it the command freezes
		// too. Measured in SITL: all three tilts stuck for an entire flight (d0
		// pinned at 9.31 deg, zero variance), yaw band 238 deg, vehicle spinning.
		//
		// The offline replay could NOT have caught this: it drove the model with a
		// RECORDED u_cmd sequence from a run where the shadow did move, so the
		// command wandered far from the frozen state. **An open-loop replay cannot
		// reveal a closed-loop feedback trap** -- validate model changes that sit
		// inside the command path in closed loop.
		//
		// The deeper point: this incremental architecture REQUIRES the shadow to
		// creep toward the command. A stiction/deadband term is fundamentally
		// incompatible with it unless the absolute command is tracked as its own
		// state, independent of the shadow -- which would break the INDI
		// linearisation point (the WLS increment is by definition relative to the
		// actuator's current state).
		//
		// If retried: TILT_STICTION_BAND (0.01 rad) is still in TiltrotorIndiParams.hpp.
		float ddelta = (u_cmd(3 + i) - _u_actual(3 + i)) / TILT_TAU;
		ddelta = math::constrain(ddelta, -TILT_RATE_MAX, TILT_RATE_MAX);
		_u_actual(3 + i) += dt * ddelta;
	}

	// Surface shadow state (item (V), step 45). Deliberately NO first-order lag:
	// the allocator box already rate-limits the command to SURF_RATE_MAX, so the
	// box IS the servo model, and inventing a `tau` the SDF does not specify would
	// add an unmeasured constant to the shadow -- exactly the divergence class that
	// cost steps 8/24. If the surfaces are later measured to lag, model it HERE and
	// against data, not from a plausible-looking number.
	for (int j = 0; j < N_SURF; j++) { _u_actual(6 + j) = u_cmd(6 + j); }

	// --- diagnostics (mirrors indi_attitude_controller.m's `diagn` struct) ---
	tiltrotor_indi_status_s status{};
	status.timestamp = hrt_absolute_time();

	for (int i = 0; i < 3; i++) {
		status.omega_sp[i] = omega_sp(i);
		status.d_hat[i] = _d_hat(i);
	}

	for (int i = 0; i < N_ACT; i++) {
		// u_cmd - _u_actual, not the WLS-internal `du` (out of scope here, and in
		// FIXED-WING mode there is no WLS solve to report) -- identical to the old
		// value on the WLS path (u_cmd was formed as _u_actual + du there too),
		// and the only meaningful "commanded delta" on the fixed-wing path.
		status.du[i] = u_cmd(i) - _u_actual(i);
		status.sat_flag[i] = sat_flag[i];
	}

	for (int i = 0; i < 5; i++) { status.nu_des[i] = nu_des(i); }

	for (int i = 0; i < N_ACT; i++) { status.u_actual[i] = _u_actual(i); }

	status.codegen_du_diff = _cg_du_diff;
	status.qbar = qbar;

	status.gain_schedule_smooth = sched.smooth;
	status.avg_tilt_rad = delta_bar;
	status.ekf_healthy = ekf_ok;
	status.shadow_u_actual_valid = _u_actual_seeded;
	status.failsafe_level = (uint8_t)_fs_level;
	status.bt_state = (uint8_t)_bt_state;
	status.bt_tilt_ceil = _bt_tilt_ceil;
	status.bt_tilt_floor = _bt_tilt_floor;
	status.pos_hold_active = _pos_hold_active;
	status.ft_state = (uint8_t)_ft_state;
	status.fz_sp = _fz_sp;
	status.cruise_fx_cmd = _cruise_fx_cmd;
	status.cruise_pitch_sp = _cruise_pitch_sp;
	status.cruise_v_fwd = bt_v_fwd;
	status.fw_state = (uint8_t)_fw_state;
	status.fw_tilt = _fw_tilt;
	_status_pub.publish(status);

	perf_end(_loop_perf);
}

int MulticopterIndiTiltrotor::task_spawn(int argc, char *argv[])
{
	MulticopterIndiTiltrotor *instance = new MulticopterIndiTiltrotor();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int MulticopterIndiTiltrotor::custom_command(int argc, char *argv[])
{
	// slewbox <rad/s>  -- see the note above _slew_box_rate.
	if (argc >= 2 && strcmp(argv[0], "slewbox") == 0) {
		const float v = atof(argv[1]);

		// Test range. The old upper bound was TILT_RATE_MAX, which turned out to be
		// the WRONG reference (2026-07-28, step 26): TILT_RATE_MAX clamps the SHADOW
		// model's ddelta, and that clamp NEVER BINDS -- measured 0.000% of samples,
		// 43x of headroom -- because ddelta = du/TILT_TAU <= box/TILT_TAU
		// = 0.007/0.15 = 0.047 rad/s. So the box rate and TILT_RATE_MAX are not even
		// in the same units of effect; capping one by the other was meaningless.
		// The quantity that actually governs tilt authority is
		//     effective slew = TILT_SLEW_BOX_RATE * TS_BOX / TILT_TAU
		// This bound is therefore just a test-range guard, not a physical claim.
		if (!(v >= 0.1f && v <= 4.0f)) {
			PX4_ERR("slewbox %.3f out of test range [0.1, 4.0] rad/s", (double)v);
			return 1;
		}

		_slew_box_rate_mrs.store((int32_t)roundf(v * 1000.0f));
		PX4_INFO("slewbox: tilt allocator box rate = %.3f rad/s (%.6f rad/tick at TS_BOX)",
			 (double)v, (double)(v * TS_BOX));
		return 0;
	}

	// tiltjerk <rad/s>  -- TEST HOOK, see _tilt_jerk_limit_mrs's declaration.
	// Bounds how fast the tilt RATE ITSELF may change tick-to-tick (one
	// derivative above slewbox), to sweep for where the pos_hold+climb
	// pitch/yaw instability (Adim 90-94) stops -- default (2x slewbox, i.e.
	// wide enough to already contain a full box-to-box swing) is a NO-OP,
	// same discipline as slewbox's own default. Range mirrors slewbox's.
	if (argc >= 2 && strcmp(argv[0], "tiltjerk") == 0) {
		const float v = atof(argv[1]);

		if (!(v >= 0.05f && v <= 8.0f)) {
			PX4_ERR("tiltjerk %.3f out of test range [0.05, 8.0] rad/s", (double)v);
			return 1;
		}

		_tilt_jerk_limit_mrs.store((int32_t)roundf(v * 1000.0f));
		PX4_INFO("tiltjerk: tilt du-vs-du jerk limit = %.3f rad/s (%.6f rad/tick at TS_BOX)",
			 (double)v, (double)(v * TS_BOX));
		return 0;
	}

	// tiltceil <deg>  -- see the note above _tilt_ceiling_mdeg.
	// Sweeps the wing tilt ceiling WITHIN a single flight, for the same reason
	// slewbox does: a back-transition can only be compared against itself if
	// everything except the swept quantity is held fixed.
	if (argc >= 2 && strcmp(argv[0], "tiltceil") == 0) {
		const float v = atof(argv[1]);

		// Test range only. The lower end is the hover trim (hoverTrim() seeds
		// delta0 ~ 9.4 deg); going below it would ask the allocator to hold the
		// tilts under the angle the yaw trim itself needs.
		if (!(v >= 5.0f && v <= 90.0f)) {
			PX4_ERR("tiltceil %.2f out of test range [5, 90] deg", (double)v);
			return 1;
		}

		_tilt_ceiling_mdeg.store((int32_t)roundf(v * 1000.0f));
		PX4_INFO("tiltceil: wing tilt ceiling = %.2f deg (tail rotor unaffected)", (double)v);
		return 0;
	}

	// ftceil <deg>  -- TEST HOOK, see _ft_tilt_ceiling_mdeg's declaration.
	// Same shape as tiltceil, but applies ONLY during FtState::RAMP/CRUISE
	// (forward transition) -- does not touch hover, back-transition, or
	// fixed-wing GLIDE/ACTIVE. Prevents the two wing rotors from both
	// approaching TILT_MAX together, which step 97 found collapses their
	// differential yaw/roll authority and Fz authority simultaneously,
	// pinning all three rotors at max thrust (uncontrolled tumbling).
	if (argc >= 2 && strcmp(argv[0], "ftceil") == 0) {
		const float v = atof(argv[1]);

		if (!(v >= 5.0f && v <= 90.0f)) {
			PX4_ERR("ftceil %.2f out of test range [5, 90] deg", (double)v);
			return 1;
		}

		_ft_tilt_ceiling_mdeg.store((int32_t)roundf(v * 1000.0f));
		PX4_INFO("ftceil: forward-transition wing tilt ceiling = %.2f deg (FtState only)", (double)v);
		return 0;
	}

	// force_fw <0|1>  -- TEST HOOK, see the note above _fw_force_arm. Bypasses the
	// fixed-wing mode entry gate (speed/altitude/FtState::CRUISE) so GLIDE/ACTIVE
	// can be validated on their own; the ONE-WAY-DOOR property is unaffected --
	// force_fw 0 does NOT retreat an already-armed GLIDE/ACTIVE back to IDLE.
	if (argc >= 2 && strcmp(argv[0], "force_fw") == 0) {
		const bool v = atoi(argv[1]) != 0;
		_fw_force_arm.store(v);
		PX4_INFO("force_fw: %d (TEST HOOK -- bypasses the real entry gate)", (int)v);
		return 0;
	}

	// fw_hdg <deg>  -- TEST HOOK, see _fw_hdg_override_mdeg's declaration.
	// Commands a live bank-to-turn heading change during FwState::ACTIVE (right/
	// left maneuvers, drive back to 0), the same law MATLAB validated under a
	// moving hdg_sp (Adim 75-76). No effect in IDLE/GLIDE -- _fw_yaw_entry is
	// consumed only in ACTIVE, and the next IDLE->GLIDE edge overwrites it
	// bumplessly from the vehicle's own heading regardless.
	if (argc >= 2 && strcmp(argv[0], "fw_hdg") == 0) {
		const float deg = atof(argv[1]);

		if (!(deg >= -180.0f && deg <= 180.0f)) {
			PX4_ERR("fw_hdg %.2f out of range [-180, 180] deg", (double)deg);
			return 1;
		}

		_fw_hdg_override_mdeg.store((int32_t)roundf(deg * 1000.0f));
		PX4_INFO("fw_hdg: %.2f deg (TEST HOOK -- live hdg_sp during FwState::ACTIVE)", (double)deg);
		return 0;
	}

	// Manual/scripted test helper: publish a one-shot tiltrotor_indi_setpoint.
	// Args: roll pitch yaw fx z_sp [leso_roll leso_pitch leso_yaw [pos_hold [bt]]]
	// leso_* default to 1,1,0 (matches the validated MATLAB hover-gust config)
	// if omitted; pass 0/1 explicitly to run the LESO on/off comparison (M5).
	// pos_hold (2026-07-29, step 28) turns on the horizontal position loop, which
	// then OVERRIDES roll/pitch -- the target is captured on the rising edge, so
	// pass it once and keep passing 1 to stay in hold. Report item (N).
	if (argc >= 6 && strcmp(argv[0], "test_sp") == 0) {
		tiltrotor_indi_setpoint_s sp{};
		sp.timestamp = hrt_absolute_time();
		sp.roll_sp = atof(argv[1]);
		sp.pitch_sp = atof(argv[2]);
		sp.yaw_sp = atof(argv[3]);
		sp.fx_sp = atof(argv[4]);
		sp.z_sp = atof(argv[5]);
		sp.leso_enable_roll = (argc >= 7) ? (atoi(argv[6]) != 0) : true;
		sp.leso_enable_pitch = (argc >= 8) ? (atoi(argv[7]) != 0) : true;
		sp.leso_enable_yaw = (argc >= 9) ? (atoi(argv[8]) != 0) : false;
		sp.pos_hold_enable = (argc >= 10) ? (atoi(argv[9]) != 0) : false;
		// 11th arg: back-transition (blocker B5, step 31). While it runs it owns
		// the wing tilt ceiling and pitch, and requests pos_hold itself at the end,
		// so a caller does NOT need to pass pos_hold as well.
		sp.bt_enable = (argc >= 11) ? (atoi(argv[10]) != 0) : false;
		sp.ft_enable = (argc >= 12) ? (atoi(argv[11]) != 0) : false;
		// 13th arg: fixed-wing terminal mode (Adim 59-61 port). Only takes effect
		// once FtState::CRUISE (i.e. ft_enable has already run to completion) and
		// past FW_TRIGGER_V/FW_MIN_ALT -- see the entry gate in Run(). ONE-WAY
		// DOOR: there is no corresponding disable path once GLIDE has started.
		sp.fw_enable = (argc >= 13) ? (atoi(argv[12]) != 0) : false;

		// BUG FIX 2026-07-27 (step 11): this Publication used to be a plain
		// function-local, so its destructor ran orb_unadvertise() the instant
		// custom_command() returned. The node then reports itself as
		// un-advertised, uORB::Subscription::advertised() keeps failing, and
		// Run()'s `_setpoint_sub.updated()` NEVER fires -- so _setpoint_valid
		// stayed false and Run() silently fell back to `z_sp = lpos.z`
		// (hold current altitude) with roll/pitch/yaw_sp = 0.
		//
		// This was verified empirically: after 60 consecutive `test_sp` calls
		// `listener tiltrotor_indi_setpoint` still printed "never published"
		// and nu_des(4) stayed at ~0.2 N instead of the ~-60 N a 6 m climb
		// demand produces. Every SITL run in this investigation before this
		// fix was therefore an ALTITUDE-HOLD test, not the intended climb --
		// see the report's step 11 for what that invalidates.
		//
		// Making it static keeps the advertisement alive for the lifetime of
		// the px4 process, which is what a setpoint publisher needs anyway.
		static uORB::Publication<tiltrotor_indi_setpoint_s> pub{ORB_ID(tiltrotor_indi_setpoint)};
		const bool ok = pub.publish(sp);

		if (!ok) {
			PX4_ERR("test setpoint publish FAILED");
			return 1;
		}

		PX4_INFO("published test setpoint: roll=%.3f pitch=%.3f yaw=%.3f fx=%.2f z_sp=%.2f pos_hold=%d bt=%d ft=%d fw=%d",
			 (double)sp.roll_sp, (double)sp.pitch_sp, (double)sp.yaw_sp, (double)sp.fx_sp, (double)sp.z_sp,
			 (int)sp.pos_hold_enable, (int)sp.bt_enable, (int)sp.ft_enable, (int)sp.fw_enable);
		return 0;
	}

	return print_usage("unknown command");
}

int MulticopterIndiTiltrotor::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
INDI attitude/rate controller + LESO disturbance observer + WLS control
allocation for the 3-tilt-rotor tailplane tiltrotor. Ported from the MATLAB
reference controller (`tiltrotor_Matlab files/indi_attitude_controller.m`
and related files). Replaces `mc_att_control`, `mc_rate_control` and
`control_allocator` for the `gz_tiltrotor_indi` airframe only — other
airframes are unaffected.

Two setpoint sources, in priority order:

1. **Pilot** — `manual_control_setpoint`, used whenever manual control is
   enabled and a fresh valid sample exists. Roll/pitch sticks are angle
   commands (±15°), yaw is a leashed rate command, throttle is a leashed
   climb-rate command, centring the roll/pitch sticks engages position hold,
   and the VTOL transition switch starts/cancels the back-transition.
2. **Bench** — the `tiltrotor_indi_setpoint` uORB topic (`test_sp`, see
   `sitl/indi_sitl_common.py`), used when no pilot input is present.

A degraded OUTER-loop estimate degrades the controller, it does not stop it:
losing the altitude or horizontal estimate switches off only the loop that
consumed it. Attitude is different — it is a hard prerequisite, and losing it
cuts the output, because rate damping alone was measured not to be survivable
on this airframe (step 35; the old `RATE_ONLY` level is retracted). Output is
otherwise cut only on a kill or flight termination. See `failsafe_level` in
`tiltrotor_indi_status`.

Attitude/Fx/altitude setpoints on the bench path mirror
`run_hover_gust_test.m` / `run_transition_test.m` — this controller has no
position/velocity outer loop by design (see the MATLAB README's
"Bilinen sınırlamalar" section).
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_indi_tiltrotor", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("slewbox",
					 "test hook: set the WLS tilt slew box rate in rad/s (default TILT_SLEW_BOX_RATE=3.00)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("tiltjerk",
					 "test hook: set the WLS tilt du-vs-du jerk limit in rad/s (default 2x slewbox = off)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("tiltceil",
					 "test hook: set the WING tilt ceiling in deg (default 90 = off; tail rotor unaffected)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("ftceil",
					 "test hook: set the forward-transition-only wing tilt ceiling in deg (default 90 = off)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("force_fw",
					 "test hook: force-arm fixed-wing mode <0|1>, bypassing the real entry gate (speed/altitude/FtState::CRUISE)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("fw_hdg",
					 "test hook: command a live bank-to-turn heading <deg, -180..180> during FwState::ACTIVE");
	PRINT_MODULE_USAGE_COMMAND_DESCR("test_sp",
					  "publish a one-shot test setpoint: roll pitch yaw fx z_sp [leso_roll leso_pitch leso_yaw [pos_hold [bt [ft [fw]]]]]");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int mc_indi_tiltrotor_main(int argc, char *argv[])
{
	return MulticopterIndiTiltrotor::main(argc, argv);
}
