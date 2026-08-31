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

#pragma once

// INDI attitude/rate controller + LESO disturbance observer + WLS control
// allocation, ported from tiltrotor_Matlab files/indi_attitude_controller.m
// (see TiltrotorIndiControl.hpp for the per-function port and
// ~/.claude/plans/cached-dancing-puppy.md for the integration design).
// Replaces mc_att_control + mc_rate_control + control_allocator for the
// gz_tiltrotor_indi airframe only (see the 4023_gz_tiltrotor_indi.post rc
// script) -- other airframes are untouched.

#include "TiltrotorIndiControl.hpp"

#include <lib/perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/estimator_status_flags.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/manual_control_switches.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/tiltrotor_indi_setpoint.h>
#include <uORB/topics/tiltrotor_indi_status.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>

using namespace time_literals;

class MulticopterIndiTiltrotor : public ModuleBase<MulticopterIndiTiltrotor>, public px4::WorkItem
{
public:
	MulticopterIndiTiltrotor();
	~MulticopterIndiTiltrotor() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	bool init();

private:
	void Run() override;

	bool ekfHealthy(hrt_abstime now);
	bool ekfYawAligned(hrt_abstime now);
	void resetState();
	void publishDisarmed();
	// Ground z datum for the land_diff gate (step 116). Only ever called from
	// the DISARMED branch of Run(); see the definition for why.
	void captureGroundDatum();
	// True when the kill/termination path is commanded -- the ONLY remaining
	// case where cutting the actuators is the intent rather than an accident.
	bool terminationCommanded();

	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};

	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _estimator_status_flags_sub{ORB_ID(estimator_status_flags)};
	uORB::Subscription _actuator_armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _manual_control_switches_sub{ORB_ID(manual_control_switches)};
	uORB::Subscription _setpoint_sub{ORB_ID(tiltrotor_indi_setpoint)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Publication<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};
	uORB::Publication<actuator_servos_s> _actuator_servos_pub{ORB_ID(actuator_servos)};
	uORB::Publication<tiltrotor_indi_status_s> _status_pub{ORB_ID(tiltrotor_indi_status)};
	// MulticopterLandDetector reads this to decide landed/airborne (see
	// MulticopterLandDetector.cpp:_vehicle_thrust_setpoint_throttle) -- without
	// it the land detector never sees a "not low throttle" signal and holds
	// `landed=true` for the whole flight regardless of real Gazebo thrust,
	// which fires COM_DISARM_PRFLT ~10s after arming. We bypass the standard
	// control_allocator (this module does its own WLS), so nothing else
	// publishes this topic -- we must, purely as landed-state telemetry.
	uORB::Publication<vehicle_thrust_setpoint_s> _vehicle_thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)};

	vehicle_control_mode_s _vehicle_control_mode{};
	tiltrotor_indi_setpoint_s _setpoint{};
	bool _setpoint_valid{false};
	bool _z_sp_latched{false}; // latches z_sp to the current altitude on first valid position, until a real setpoint arrives

	// --- shadow actuator model (u_actual), see TiltrotorIndiControl.hpp header note ---
	// [T0,T1,T2, d0,d1,d2] -- our own first-order-lag estimate of what the
	// real Gazebo actuators are doing, since PX4 has no servo/ESC position
	// feedback. Same tau_up/tau_down/tilt.tau/tilt.rate_max as
	// tiltrotor_plant_deriv.m, integrated with the module's own measured dt.
	matrix::Vector<float, tiltrotor_indi::N_ACT> _u_actual;  // [T0..T2 d0..d2 s0..s4]
	bool _u_actual_seeded{false};

	// Previous tick's tilt du (WLS increment for actuators 3,4,5), for the
	// `tiltjerk` test hook -- see its declaration in the .cpp for why.
	matrix::Vector3f _prev_du_tilt{0.f, 0.f, 0.f};

	// --- LESO state (leso_axis_update.m), one 2nd order observer per axis ---
	tiltrotor_indi::LesoAxis _leso[3];
	matrix::Vector3f _d_hat{0.f, 0.f, 0.f};
	matrix::Vector3f _prev_u_leso{0.f, 0.f, 0.f}; // ESO "input" = post-compensation omega_dot_des
	float _leso_accum{0.f};

	// --- altitude outer loop (altitude_loop.m) ---
	float _alt_integral_vz{0.f};
	float _alt_accum{0.f};
	float _fz_sp{-tiltrotor_indi::MASS *tiltrotor_indi::GRAVITY};

	// --- horizontal position outer loop (position_loop.m, item (N)) ---
	// Shares the altitude loop's ALT_TS decimation. _pos_sp is captured on the
	// pos_hold_enable rising edge, so a test never has to know its coordinates.
	bool _pos_hold_active{false};
	matrix::Vector2f _pos_sp{0.f, 0.f};
	matrix::Vector2f _pos_integral_v{0.f, 0.f};
	matrix::Vector2f _pos_att_sp{0.f, 0.f};   // [roll_sp, pitch_sp] from the loop
	float _pos_fx_trim{0.f};                  // N, Fx setpoint from the loop (item (P))
	bool _pos_hold_refused{false};            // latch so the refusal warns once, not every tick

	// Back-transition state machine (blocker B5, step 31). Own ALT_TS accumulator
	// because it must run BEFORE the pos_hold block -- it can request the hold.
	// See backTransition() in TiltrotorIndiControl.hpp.
	float _bt_accum{0.f};
	tiltrotor_indi::BtState _bt_state{tiltrotor_indi::BtState::IDLE};
	float _bt_tilt_ceil{tiltrotor_indi::TILT_MAX}; // rad, wing tilt ceiling (TILT_MAX = off)
	float _bt_tilt_floor{tiltrotor_indi::TILT_MIN}; // rad, wing tilt floor (step 101, TILT_MIN = off)
	float _bt_floor_dwell{0.f};               // s, dwell at the ceiling floor (item (R) backstop)
	float _bt_pitch_sp{0.f};                  // rad, nose-up brake command
	bool _bt_req_pos_hold{false};
	bool _bt_refused{false};                  // latch so the refusal warns once
	// Item (S) residual (step 39): HANDOFF now fires on the forward axis, but
	// pos_hold's engage gate is still a magnitude gate, so lateral drift can hold
	// the hold off after the handoff was requested. This times that gap so the
	// residual is a MEASUREMENT in the log, not an assumption in a comment.
	float _bt_handoff_wait{0.f};              // s, HANDOFF requested but hold not engaged

	// Forward transition, hover -> cruise (item (V), step 42). Shares the
	// back-transition's ALT_TS accumulator: the two are mutually exclusive by
	// construction (an FT abort REQUESTS the BT), and running them off one
	// decimator keeps that exclusion visible in one place.
	tiltrotor_indi::FtState _ft_state{tiltrotor_indi::FtState::IDLE};
	float _ft_fx_cmd{0.f};                    // N, ramped body-x force command
	float _ft_z_entry{0.f};                   // m, altitude-band reference (NED)
	float _ft_t_ramp{0.f};                    // s, time spent in RAMP
	float _ft_pitch_sp{0.f};                  // rad, always 0 (step 29) -- see forwardTransition()
	bool _ft_release_hold{false};
	bool _ft_req_abort{false};
	bool _ft_refused{false};                  // latch so the refusal warns once
	uint8_t _ft_warn_prev{0};                 // rising-edge latch for the safety-detector warning
						  // (FT_ALLOW_ABORT = false: detectors report, do not act)
	// Thrust saturation from the PREVIOUS tick. The forward transition needs it to
	// decide whether to keep ramping fx, but it runs BEFORE the allocator, so the
	// current tick's flags do not exist yet. One tick (4 ms) of lag on a ramp that
	// takes 14.4 s is irrelevant; reordering the loop to avoid it would not be.
	bool _sat_thrust_prev{false};

	// --- cruise energy management (this airframe's TECS, steps 47/49/53) ---
	// Runs off the ALTITUDE accumulator, not the FT/BT one, and that placement is
	// deliberate: the pitch loop's input is _fz_sp, so it must see the value from
	// THIS tick's altitude loop rather than the previous one.
	float _cruise_fx_I{0.f};       // N, speed-loop integrator (tracks fx while inactive)
	float _cruise_pitch_I{0.f};    // rad, pitch-trim integrator (MODEL ERROR only, step 53)
	float _cruise_fx_cmd{0.f};     // N, speed-loop output actually applied
	float _cruise_pitch_sp{0.f};   // rad, pitch-loop output actually applied

	// --- fixed-wing terminal mode (Adim 59-61 port, ARCHITECTURE CORRECTED
	// Adim 75-77, see TiltrotorIndiParams.hpp FW_* constants and
	// fixedWingTransition()/fixedWingControlLaw() for the mechanism).
	// ONE-WAY DOOR: only reachable from FtState::CRUISE. ---
	tiltrotor_indi::FwState _fw_state{tiltrotor_indi::FwState::IDLE};
	float _fw_tilt{0.f};           // rad, ramped/commanded wing-rotor tilt (GLIDE), TILT_MAX once ACTIVE
	float _fw_v_I{0.f};            // N*s, wing-thrust speed-loop integrator
	float _fw_alt_I{0.f};          // m*s, altitude PI integrator
	float _fw_v_fwd_filt{0.f};     // m/s, filtered v_fwd for the pitch feed-forward only (Adim 68)
	float _fw_yaw_entry{0.f};      // rad, heading captured bumplessly on the IDLE->GLIDE edge (Adim 68) -- now the hdg_sp bank-to-turn target (Adim 75)
	bool _fw_refused{false};       // latch so the entry-refusal warning fires once

	// --- graded failsafe (blocker B1, step 32; RATE_ONLY retracted step 35) ---
	// See the FsLevel note in TiltrotorIndiParams.hpp for why a degraded OUTER
	// loop must not cut the actuators -- and why attitude is the one input that
	// is not gradeable, so it is handled as a hard prerequisite in Run() rather
	// than as a level. _fs_level is recomputed every tick; _fs_level_reported
	// only throttles the log so a flickering estimate does not spam the console
	// at 250 Hz, and _att_loss_reported does the same for the attitude cut.
	tiltrotor_indi::FsLevel _fs_level{tiltrotor_indi::FsLevel::NONE};
	tiltrotor_indi::FsLevel _fs_level_reported{tiltrotor_indi::FsLevel::NONE};
	bool _att_loss_reported{false};

	// --- ground z datum (step 116) ---
	// `-lpos.z` is height above the EKF LOCAL ORIGIN, not above ground: the
	// origin is set once at estimator init and carries that instant's error as
	// a permanent per-flight offset (measured over 23 runs: -0.67 .. +1.77 m).
	// AGL = _z_datum - lpos.z. NAN until a datum is captured, which closes the
	// land_diff gate -- the same safe side the alt_ok check already takes.
	float _z_datum{NAN};

	// INIS DIZISI (Adim 153, madde B0). Profil artik MODULDE; oncesinde
	// PC tarafindaki run_mission_test.py'deydi ve o yol kartta yok.
	tiltrotor_indi::LandState _land_state{tiltrotor_indi::LandState::IDLE};
	float _land_step_timer{0.f};
	float _land_touch_dwell{0.f};
	float _land_stall_dwell{0.f};
	float _land_z_step{0.f};
	float _land_z_cmd{0.f};
	bool _z_datum_reported{false};

	// --- second contact criterion (step 118) ---
	// Dwell timer + latch for "large differential command, no angular
	// acceleration". Rationale and the measured separation are in
	// TiltrotorIndiParams.hpp next to LAND_CONTACT_DIFF.
	float _land_contact_dwell{0.f};
	bool _land_contact_latch{false};

	// --- WLS working storage (step 120) ---
	// Hoisted off wlsAllocate()'s stack: the function's frame was 2184 B and
	// the real-board build enforces a 2048 B limit. Owned here so the function
	// stays reentrant-safe by construction rather than by luck; see the
	// WlsScratch comment in TiltrotorIndiControl.hpp.
	tiltrotor_indi::WlsScratch _wls_scratch{};

	// --- golge codegen durumu (Adim 131) ---
	// Uretilen sf_wls_alloc'un kendi durumu ([temas_bekleme; mandal;
	// prev_du_tilt(3)]) -- C++ kendi durumunu ayri tutar, boylece iki
	// uygulama birbirini KIRLETMEZ ve fark gercekten bagimsiz olculur.
	double _cg_state[5] {};
	float _cg_du_diff{0.f};

	// --- pilot input (blocker B1, step 33) ---
	// _man_yaw_sp / _man_z_sp are the integrated stick commands; they are
	// re-seeded from the vehicle's current heading/altitude whenever the pilot
	// takes over, so a handover never starts with a stale target.
	bool _man_active{false};
	bool _man_lost{false};        // was flying manually, then the link went away
	float _man_yaw_sp{0.f};
	float _man_z_sp{0.f};

	hrt_abstime _last_run{0};

	perf_counter_t _loop_perf;
};
