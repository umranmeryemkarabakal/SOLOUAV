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

// C++ port of the MATLAB reference controller in
// tiltrotor_Matlab files/{gain_schedule,effectiveness_matrix,wls_allocate,
// leso_axis_update,leso_bandwidth_gains,altitude_loop}.m. The WLS solve
// follows sf_wls_alloc.m (the fixed-size "big-M penalty" rewrite made for
// Simulink codegen) rather than wls_allocate.m's active-set/variable-index
// version, since it maps directly onto matrix::Matrix<float,N,N> without
// dynamic-size sub-matrices -- same convergence, same box constraints.

#include <lib/matrix/matrix/math.hpp>
#include <mathlib/math/Limits.hpp>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>

#include "TiltrotorIndiParams.hpp"

namespace tiltrotor_indi
{

struct GainSchedule {
	matrix::Vector3f kp_att;
	matrix::Vector3f kp_rate;
	float wu_tilt[3]; // per-rotor WLS tilt preference weight (tail penalised x3)
	float smooth;     // 0=hover .. 1=cruise
};

// delta_bar: mean tilt angle (rad) of the shadow actuator state, 0=hover, pi/2=cruise
inline GainSchedule computeGainSchedule(float delta_bar)
{
	GainSchedule sched{};

	float s = delta_bar / (M_PI_F / 2.0f);
	s = math::constrain(s, 0.0f, 1.0f);
	const float w = 3.0f * s * s - 2.0f * s * s * s; // smoothstep

	for (int i = 0; i < 3; i++) {
		sched.kp_att(i) = KP_ATT_HOVER[i] + (KP_ATT_CRUISE[i] - KP_ATT_HOVER[i]) * w;
		sched.kp_rate(i) = KP_RATE_HOVER[i] + (KP_RATE_CRUISE[i] - KP_RATE_HOVER[i]) * w;
	}

	const float wu_tilt = WU_TILT_HOVER + (WU_TILT_CRUISE - WU_TILT_HOVER) * w;
	sched.wu_tilt[0] = wu_tilt;
	sched.wu_tilt[1] = wu_tilt;
	sched.wu_tilt[2] = wu_tilt * WU_TAIL_PENALTY; // tail rotor: tilt has no roll/yaw authority

	sched.smooth = w;
	return sched;
}

// Single-axis 2nd order LESO (leso_axis_update.m). Model: omega_dot = u_applied + d(t).
// z1 -> omega estimate, z2 -> d_hat (cumulative disturbance angular accel estimate).
struct LesoAxis {
	float z1{0.0f};
	float z2{0.0f};

	void update(float y_meas, float u_applied, float dt)
	{
		const float e = y_meas - z1;
		z1 += dt * (z2 + u_applied + LESO_BETA1 * e);
		z2 += dt * (LESO_BETA2 * e);
	}
};

// u = [T0,T1,T2, d0,d1,d2, s0..s4]. G = d(nu)/d(u) (5xN_ACT), nu0 = actual nu at u.
// nu = [taux,tauy,tauz,Fx,Fz]. Ported from effectiveness_matrix.m / sf_wls_alloc.m.
//
// `qbar` = 0.5*rho*V^2. THE SURFACE COLUMNS ARE EXACTLY ZERO AT qbar = 0, and that
// is the whole reason this needs no mode switch (step 45, item (V)): a
// zero-effectiveness column costs only Wu*du^2, whose minimum is du = 0, so in
// hover the allocation is bit-identical to the 6-actuator version -- a neutrality
// that is PROVABLE before it is measured. As speed rises the columns grow and the
// surfaces take over continuously, exactly like the gain schedule does.
inline void effectivenessMatrix(const matrix::Vector<float, N_ACT> &u, float qbar,
				 matrix::Matrix<float, 5, N_ACT> &G, matrix::Vector<float, 5> &nu0)
{
	G.setZero();
	nu0.setZero();

	for (int i = 0; i < 3; i++) {
		const float T = u(i);
		const float de = u(3 + i);
		const float s = sinf(de);
		const float c = cosf(de);

		const matrix::Vector3f dir{s, 0.0f, -c};
		const matrix::Vector3f ddir{c, 0.0f, s};
		const matrix::Vector3f r{ROTOR_PX[i], ROTOR_PY[i], ROTOR_PZ[i]};
		const float km = ROTOR_KM[i];

		const matrix::Vector3f f = dir * T;
		const matrix::Vector3f m = dir * (km * T);
		const matrix::Vector3f tau = r.cross(f) + m;

		const matrix::Vector3f dtau_dT = r.cross(dir) + dir * km;
		const matrix::Vector3f dtau_ddelta = (r.cross(ddir) + ddir * km) * T;

		nu0(0) += tau(0); nu0(1) += tau(1); nu0(2) += tau(2);
		nu0(3) += f(0); // Fx
		nu0(4) += f(2); // Fz

		G(0, i) = dtau_dT(0); G(1, i) = dtau_dT(1); G(2, i) = dtau_dT(2);
		G(0, 3 + i) = dtau_ddelta(0); G(1, 3 + i) = dtau_ddelta(1); G(2, 3 + i) = dtau_ddelta(2);

		G(3, i) = dir(0);
		G(3, 3 + i) = T * ddir(0);
		G(4, i) = dir(2);
		G(4, 3 + i) = T * ddir(2);
	}

	// --- aerodynamic control surfaces (item (V), step 45) ---
	// F_j = qbar * k_j * delta_j * e_j at cp_j; linear in delta, so the Jacobian
	// does not depend on delta -- only on qbar. See the note above for why qbar=0
	// makes this block a provable no-op.
	for (int j = 0; SURF_ENABLE && j < N_SURF; j++) {
		const matrix::Vector3f e{SURF_EX[j], SURF_EY[j], SURF_EZ[j]};
		const matrix::Vector3f r{SURF_CPX[j], SURF_CPY[j], SURF_CPZ[j]};
		const matrix::Vector3f dF = e * (qbar * SURF_K[j]);
		const matrix::Vector3f dtau = r.cross(dF);

		const int col = 6 + j;
		G(0, col) = dtau(0); G(1, col) = dtau(1); G(2, col) = dtau(2);
		// THE FORCE ROWS ARE DELIBERATELY LEFT AT ZERO -- surfaces are declared to
		// the allocator as MOMENT actuators only. Measured the hard way (step 45,
		// first surface flight): with the force rows populated, all three rotors
		// pinned at ROTOR_TMAX (45.0/45.0/44.9 N), thrust saturation 16-24%, 1958
		// BIG_M, and the cruise tilt COLLAPSED from 39.7-44.1 deg to 13.3 deg.
		// Cause, and it is a number in the SDF I had read past: servo_0/1's cp is
		// at x = -0.05 m, i.e. essentially AT the CG. They are not pitch surfaces
		// at all -- they are the MAIN WING's flap: pitch effectiveness 24.5 N*m/rad
		// against an Fz effectiveness of 490 N/rad at 245 Pa, a factor of 20. So
		// any use of them produced an enormous vertical force (measured ~107 N of
		// download at 13.4 deg, more than twice the weight) and the rotors maxed
		// out fighting it -- a self-consistent bad equilibrium.
		// Declaring them moment-only is not a fiction the vehicle pays for: a
		// differential (roll) deflection has zero NET Fz by symmetry, and the
		// residual force still appears in nu0 below, so the allocator sees it and
		// trims it out with the rotors. What it can no longer do is CHASE the
		// altitude channel with the wing's flap.
		// G(3, col) = dF(0);   // Fx  -- intentionally not exposed
		// G(4, col) = dF(2);   // Fz  -- intentionally not exposed

		const float dj = u(col);
		nu0(0) += dtau(0) * dj; nu0(1) += dtau(1) * dj; nu0(2) += dtau(2) * dj;
		nu0(3) += dF(0) * dj;
		nu0(4) += dF(2) * dj;
	}
}

// Weighted least-squares control allocation, box-constrained (sf_wls_alloc.m big-M method).
// Returns du (actuator increment); sat_flag[i] true if actuator i pinned at a bound.
// WLS CALISMA ALANI -- YIGINDAN CIKARILDI (Adim 120).
//
// NEDEN: `make cubepilot_cubeorange` su hatayla durdu:
//   error: the frame size of 2184 bytes is larger than 2048 bytes
//          [-Werror=frame-larger-than=]
// 2048 B keyfi bir uyari degil, PX4'un GOREV YIGINI butcesi. SITL (x86) bu
// bayragi uygulamadigi icin sorun ancak gercek kart derlemesinde goruldu --
// yani tam olarak HITL'e gecmeden yakalanmasi gereken sinifta.
//
// NEDEN `static` DEGIL: en kisa yol oydu ama fonksiyonu reentrant olmaktan
// cikarirdi. Modul bugun tek bir work-queue ogesinde kosuyor, yani "calisirdi"
// -- ama sansa dayanan bir kural olurdu ve header'da inline bir fonksiyonda
// ODR sorunlari da cikarirdi. Sahipligi acikca cagirana vermek daha durust.
//
// NEDEN SINIRI YUKSELTMEK DEGIL: -Wno-error=frame-larger-than uyariyi susturur,
// yigin butcesini BUYUTMEZ; gercek kartta stack tasmasi demek olurdu.
//
// SAYISAL OLARAK KIMLIK: ayni islemler, ayni sira, ayni float'lar. Depolamanin
// yeri IEEE aritmetigini degistirmez, dolayisiyla SITL metriklerinde HERHANGI
// bir sapma dogrudan transkripsiyon hatasi demektir.
struct WlsScratch {
	matrix::Matrix<float, N_ACT, 5> Gt;
	matrix::SquareMatrix<float, N_ACT> H;
	matrix::SquareMatrix<float, N_ACT> Hinv;
	matrix::Matrix<float, 5, N_ACT> WsG;
	matrix::Vector<float, N_ACT> Wu_eff;
	matrix::Vector<float, N_ACT> du_pref;
	matrix::Vector<float, N_ACT> du;
	matrix::Vector<float, N_ACT> rhs;
};

inline matrix::Vector<float, N_ACT> wlsAllocate(const matrix::Matrix<float, 5, N_ACT> &G,
		const matrix::Vector<float, 5> &nu_des, const matrix::Vector<float, N_ACT> &du_min,
		const matrix::Vector<float, N_ACT> &du_max, const matrix::Vector<float, 5> &Ws,
		const matrix::Vector<float, N_ACT> &Wu, bool sat_flag[N_ACT], WlsScratch &s)
{
	static constexpr float BIG_M = 1e6f;

	matrix::Vector<float, 5> Ws2;
	for (int i = 0; i < 5; i++) { Ws2(i) = Ws(i) * Ws(i); }

	matrix::Vector<float, N_ACT> &Wu_eff = s.Wu_eff;
	matrix::Vector<float, N_ACT> &du_pref = s.du_pref;
	matrix::Vector<float, N_ACT> &du = s.du;
	Wu_eff = Wu;
	du_pref.setZero();
	du.setZero();

	matrix::Matrix<float, N_ACT, 5> &Gt = s.Gt;
	Gt = G.transpose();

	// Inverse from the final solved iteration, kept for the WRdbg per-axis
	// attribution below (diagnostic only -- not used by the allocation itself).
	matrix::SquareMatrix<float, N_ACT> &Hinv = s.Hinv;

	// Iteration cap = one round per actuator: each round can only freeze
	// actuators, so it cannot need more. Grew with N_ACT (step 45) -- leaving it
	// at 6 with 11 actuators would silently return a still-violating solution.
	for (int it = 0; it < N_ACT; it++) {
		matrix::Matrix<float, 5, N_ACT> &WsG = s.WsG;
		WsG = G;

		for (int r = 0; r < 5; r++) {
			for (int c = 0; c < N_ACT; c++) { WsG(r, c) *= Ws2(r); }
		}

		matrix::SquareMatrix<float, N_ACT> &H = s.H;
		H = Gt * WsG;

		for (int i = 0; i < N_ACT; i++) { H(i, i) += Wu_eff(i) * Wu_eff(i); }

		matrix::Vector<float, N_ACT> &rhs = s.rhs;
		rhs = Gt * (matrix::Vector<float, 5>)(Ws2.emult(nu_des));

		for (int i = 0; i < N_ACT; i++) { rhs(i) += Wu_eff(i) * Wu_eff(i) * du_pref(i); }

		Hinv = H.I();
		du = Hinv * rhs;

		bool any_violation = false;

		for (int i = 0; i < N_ACT; i++) {
			if (du(i) > du_max(i)) {
				du_pref(i) = du_max(i);
				Wu_eff(i) = BIG_M;
				any_violation = true;

			} else if (du(i) < du_min(i)) {
				du_pref(i) = du_min(i);
				Wu_eff(i) = BIG_M;
				any_violation = true;
			}
		}

		if (!any_violation) {
			break;
		}
	}

	for (int i = 0; i < N_ACT; i++) {
		du(i) = math::constrain(du(i), du_min(i), du_max(i));
		sat_flag[i] = (Wu_eff(i) >= BIG_M);
	}

	// TEMP DIAGNOSTIC (2026-07-25, T0 lockup investigation, remove before merge):
	// throttled dump of why WLS is picking du(0) -- rotor-0 effectiveness
	// column, weights, and the resulting box/weight state.
	{
		static hrt_abstime last_dbg = 0;
		const hrt_abstime now_dbg = hrt_absolute_time();

		if (now_dbg - last_dbg > 500000) { // 500 ms, microseconds
			last_dbg = now_dbg;
			PX4_INFO("T0dbg1 G0=[%.3f %.3f %.3f %.3f %.3f] nu_des=[%.2f %.2f %.2f %.2f %.2f]",
				 (double)G(0, 0), (double)G(1, 0), (double)G(2, 0), (double)G(3, 0), (double)G(4, 0),
				 (double)nu_des(0), (double)nu_des(1), (double)nu_des(2), (double)nu_des(3), (double)nu_des(4));
			PX4_INFO("T0dbg2 Ws34=[%.1f %.1f] Wu0=%.1f du0=%.4f dmin0=%.4f dmax0=%.4f",
				 (double)Ws(3), (double)Ws(4),
				 (double)Wu_eff(0), (double)du(0), (double)du_min(0), (double)du_max(0));

			// T2dbg (2026-07-26, yaw-runaway / tail-tilt investigation): rotor-2
			// (tail) thrust column (index 2) and tilt column (index 5)
			// effectiveness, plus yaw (nu_des(2)) and the resulting du for both,
			// to find which nu_des component keeps pushing delta2 up.
			PX4_INFO("T2dbg1 G2=[%.3f %.3f %.3f %.3f %.3f] G5=[%.3f %.3f %.3f %.3f %.3f]",
				 (double)G(0, 2), (double)G(1, 2), (double)G(2, 2), (double)G(3, 2), (double)G(4, 2),
				 (double)G(0, 5), (double)G(1, 5), (double)G(2, 5), (double)G(3, 5), (double)G(4, 5));
			PX4_INFO("T2dbg2 Wu25=[%.1f %.1f] du25=[%.4f %.4f] dmax25=[%.4f %.4f] nu_yaw=%.3f",
				 (double)Wu_eff(2), (double)Wu_eff(5), (double)du(2), (double)du(5),
				 (double)du_max(2), (double)du_max(5), (double)nu_des(2));

			// WRdbg (2026-07-27, step 11 / report section 4 (G)): per-axis
			// attribution for BOTH wing rotors. T1 was previously uninstrumented
			// even though it is the rotor that locked to zero in step 9 run 2.
			//
			// The unconstrained solve is du = Hinv * (G'*Ws2*nu_des + Wu^2*du_pref),
			// which is linear in nu_des, so the contribution of each individual
			// nu_des axis k to du(i) can be separated exactly:
			//     a[k] = (Hinv * (G(k,:)' * Ws2(k) * nu_des(k)))(i)
			// and the BIG_M pinning term contributes pin(i). By construction
			// sum_k a[k] + pin == du(i) (before the final box clamp), so the
			// printed row is self-checking. Whichever axis carries the large
			// negative a[k] is the one pushing that rotor to zero.
			float a0[5], a1[5];

			for (int k = 0; k < 5; k++) {
				matrix::Vector<float, N_ACT> rhs_k;

				for (int i = 0; i < N_ACT; i++) { rhs_k(i) = G(k, i) * Ws2(k) * nu_des(k); }

				const matrix::Vector<float, N_ACT> du_k = Hinv * rhs_k;
				a0[k] = du_k(0);
				a1[k] = du_k(1);
			}

			matrix::Vector<float, N_ACT> rhs_pin;

			for (int i = 0; i < N_ACT; i++) { rhs_pin(i) = Wu_eff(i) * Wu_eff(i) * du_pref(i); }

			const matrix::Vector<float, N_ACT> du_pin = Hinv * rhs_pin;

			// NOTE: PX4_INFO truncates long lines (this silently clipped
			// "Wu1=1000000" down to "Wu1=100" in the first step-11 run) --
			// keep the pinned/lock indicators (Wu, box lo) EARLY in the format
			// string so they survive, and keep each line short.
			PX4_INFO("WRdbg0 Wu0=%.0f lo0=%.2f du0=%.4f pin0=%.3f a0=[%.3f %.3f %.3f %.3f %.3f]",
				 (double)Wu_eff(0), (double)du_min(0), (double)du(0), (double)du_pin(0),
				 (double)a0[0], (double)a0[1], (double)a0[2], (double)a0[3], (double)a0[4]);
			PX4_INFO("WRdbg1 Wu1=%.0f lo1=%.2f du1=%.4f pin1=%.3f a1=[%.3f %.3f %.3f %.3f %.3f]",
				 (double)Wu_eff(1), (double)du_min(1), (double)du(1), (double)du_pin(1),
				 (double)a1[0], (double)a1[1], (double)a1[2], (double)a1[3], (double)a1[4]);
		}
	}

	return du;
}

// Newton -> normalized rotor command, inverting the Gazebo motor model.
//
// BUG FIX 2026-07-27 (step 11, see sitl/WLS_LOCKUP_INVESTIGATION_REPORT.md).
// This used to be a plain linear `u / ROTOR_TMAX`, which silently assumed
// thrust is linear in the normalized command. It is not. The chain is:
//
//   MixingOutput:          w = ROTOR_WMIN + control * (ROTOR_WMAX - ROTOR_WMIN)
//   MulticopterMotorModel: T = ROTOR_KF * w^2
//
// so thrust is QUADRATIC in `control`. The two agree only at the endpoints
// (control=1 -> ROTOR_TMAX), which is why the error was easy to miss: the
// linear map under-delivers everywhere in between, by up to ~70% at low
// thrust. Correct inverse of T = ROTOR_KF * w^2:
//
//   w = sqrt(u / ROTOR_KF) = ROTOR_WMAX * sqrt(u / ROTOR_TMAX)
//   control = (w - ROTOR_WMIN) / (ROTOR_WMAX - ROTOR_WMIN)
//
// This is a SITL/PX4-specific output-mapping fix with no MATLAB counterpart:
// the MATLAB plant (tiltrotor_plant_deriv.m) is driven in Newtons directly and
// has no normalized-command layer at all, so there is nothing to keep in sync
// there (same kind of deliberate exception to the safe-control-change
// MATLAB-first rule as TILT_RATE_MAX in step 9).
inline float thrustToNormalized(float u)
{
	const float u_clamped = math::constrain(u, ROTOR_TMIN, ROTOR_TMAX);
	const float w = ROTOR_WMAX * sqrtf(u_clamped / ROTOR_TMAX);
	return math::constrain((w - ROTOR_WMIN) / (ROTOR_WMAX - ROTOR_WMIN), 0.0f, 1.0f);
}

// altitude_loop.m: P (position) + PI (vertical speed, anti-windup clamp).
// Called at the decimated ALT_TS cadence -- integral uses the fixed ALT_TS
// (not real elapsed time) to match the MATLAB reference exactly.
// Inner half of altitude_loop.m: the PI on vertical speed alone. Pure
// extraction -- altitudeLoop() below is the same arithmetic in the same order,
// so it is bit-for-bit neutral for the flown path.
//
// NOT dead code, despite what the step-34 checklist entry said: altitudeLoop()
// calls it every ALT_TS on the NOMINAL path, so it runs in every flight. What
// was dead was its second CALLER -- a failsafe branch commanding a descent rate
// when "z is gone but vz survives", a state EKF2 cannot produce
// (EKF2.cpp:1588-1590 derives both bits from one OR). That caller was removed in
// step 36; the split itself stays because it is what makes altitudeLoop
// readable.
inline float altitudeLoopVz(float vz_sp, float vz, float &integral_vz)
{
	const float err_vz = vz_sp - vz;

	integral_vz = math::constrain(integral_vz + err_vz * ALT_TS, -ALT_INT_MAX, ALT_INT_MAX);

	const float az_corr = ALT_KP_VZ * err_vz + ALT_KI_VZ * integral_vz;
	// Hard physical clamp (step 57) -- the proportional term above is
	// unbounded in err_vz (only vz_sp is limited, not measured vz), so a
	// diverging vz previously produced a Fz_sp WLS could never satisfy. See
	// TiltrotorIndiParams.hpp ALT_FZ_MIN/ALT_FZ_MAX_CLAMP for the full
	// rationale -- deliberately not ported to altitude_loop.m.
	return math::constrain(MASS * (az_corr - GRAVITY), ALT_FZ_MIN, ALT_FZ_MAX_CLAMP);
}

inline float altitudeLoop(float z_sp, float z, float vz, float &integral_vz)
{
	const float err_z = z_sp - z;
	const float vz_sp = math::constrain(ALT_KP_Z * err_z, -ALT_VZ_MAX, ALT_VZ_MAX);
	return altitudeLoopVz(vz_sp, vz, integral_vz);
}

// position_loop.m: horizontal position outer loop -- P (position) -> PI
// (velocity) -> roll/pitch setpoint. Report item (N); see TiltrotorIndiParams.hpp
// for why this produces ATTITUDE rather than an Fx setpoint.
//
// SIGN CONVENTION: +pitch = nose UP = BACKWARD force, so forward acceleration
// needs NEGATIVE pitch. Step 15's measurement corroborates it -- commanding
// pitch_sp = +0.061 rad (nose up) cut the structural forward drift from 9.4 to
// 1.7 m/s. +roll = right wing down = +y (right) force.
//
// Shares altitude_loop's ALT_TS decimation, so the integral advances by the
// fixed ALT_TS (not real elapsed time) to match the MATLAB reference exactly.
// Returns [roll_sp, pitch_sp]; integral_v is [x, y] in NED.
// fx_trim (out): the body-x force setpoint this loop asks the allocator for.
// Report item (P): the tilt range is one-sided, so hover Fx >= 0 and the yaw
// trim leaves ~2.9 N of forward force the vehicle CANNOT cancel. Demanding
// Fx = 0 therefore hands the allocator an unreachable target -- measured, the
// unconstrained solution goes negative on all three tilt channels and delta1 /
// delta2 sit permanently pinned on TILT_MIN, which IS item (P)'s direction
// asymmetry (-yaw free, +yaw against the bound). Adding the trim removes that
// endless downward push: measured asymmetry 1.33x -> 1.02x on a +-30 deg step.
// It lives HERE, not in the allocator, because its cost is a persistent +Fx
// that only this loop can carry -- so (P)'s fix is structurally tied to (N)'s.
// (It was first put in the allocator; that regressed hover-gust pitch-rate RMS
// 0.0004 -> 0.0013 because that test has no position loop to absorb the force.)
inline matrix::Vector2f positionLoop(const matrix::Vector2f &pos_sp, const matrix::Vector2f &pos,
				     const matrix::Vector2f &vel_ned, float psi, float delta_bar,
				     matrix::Vector2f &integral_v, float &fx_trim)
{
	// faded out across the transition with gain_schedule.m's smoothstep: in
	// cruise the tilt is large and Fx is high on purpose, so a trim is meaningless
	const float s_sched = math::constrain(delta_bar / TILT_MAX, 0.0f, 1.0f);
	const float w_sched = 3.0f * s_sched * s_sched - 2.0f * s_sched * s_sched * s_sched;
	fx_trim = FX_TRIM * (1.0f - w_sched);

	// P: position error -> velocity setpoint (NED); magnitude-clamped so the
	// commanded direction is preserved
	matrix::Vector2f v_sp = (pos_sp - pos) * POS_KP_P;
	const float nv = v_sp.norm();

	if (nv > POS_V_MAX) { v_sp *= (POS_V_MAX / nv); }

	// PI: velocity error -> acceleration setpoint (NED)
	const matrix::Vector2f err_v = v_sp - vel_ned;
	integral_v += err_v * ALT_TS;
	integral_v(0) = math::constrain(integral_v(0), -POS_INT_MAX, POS_INT_MAX);
	integral_v(1) = math::constrain(integral_v(1), -POS_INT_MAX, POS_INT_MAX);

	matrix::Vector2f a_ned = err_v * POS_KP_V + integral_v * POS_KI_V;
	const float na = a_ned.norm();

	if (na > POS_A_MAX) { a_ned *= (POS_A_MAX / na); }

	// NED acceleration -> body-heading frame
	const float c = cosf(psi);
	const float s = sinf(psi);
	const float ax_b =  a_ned(0) * c + a_ned(1) * s;
	const float ay_b = -a_ned(0) * s + a_ned(1) * c;

	matrix::Vector2f att_sp_xy;
	att_sp_xy(0) = math::constrain(atan2f(ay_b, GRAVITY), -POS_TILT_MAX, POS_TILT_MAX);   // roll
	att_sp_xy(1) = math::constrain(-atan2f(ax_b, GRAVITY), -POS_TILT_MAX, POS_TILT_MAX);  // pitch
	return att_sp_xy;
}

// decel_loop.m: back-transition braking. TRIED AND REVERTED (2026-07-29,
// step 30) -- kept, unused, so the same idea is not retried blind.
//
// Idea: nose-up proportional to speed, bounded by an adaptive cap that shrinks
// whenever the vehicle climbs (model-free, so it would transfer to hardware).
//
// WHY IT FAILED, measured in SITL: the vehicle did not decelerate at all
// (speed stayed 11.7-19.2 m/s for ~27 s) and eventually crashed. The log shows
// the reason, and it is NOT the pitch law: during the attempt the TILTS RAN
// FORWARD, 43 deg -> 80 deg (tilt0) and 36 deg -> 69 deg (tilt1). Setting
// fx_sp = 0 does not retract them, because Fx is only a very weak WLS
// objective (Ws_Fx = 0.05, chosen deliberately -- the differential tilt that
// trims yaw also produces Fx, so penalising Fx at roll/pitch weight wrecks
// yaw), and the gain schedule makes tilt CHEAPER as it grows
// (wu_tilt 3.0 -> 1.5). Meanwhile the adaptive cap did its job and collapsed
// the pitch command to ~0-4 deg, leaving no braking authority at all.
//
// CONCLUSION: on a tiltrotor the braking authority for a back-transition is
// the TILT, not the pitch attitude. A real back-transition has to command tilt
// retraction directly (a tilt setpoint, or scheduling Ws_Fx up during the
// manoeuvre) rather than hoping the allocator infers it from fx_sp. That is a
// deliberate design change, not a tuning fix, and it is the recommended next
// step. Until then pos_hold simply REFUSES above POS_ENGAGE_V_MAX.
//
// vz is NED (negative = climbing). pitch_cap carries state between calls and
// must be initialised to DECEL_PITCH_MAX when the phase starts.
inline float decelLoop(float v_h, float vz, float &pitch_cap)
{
	const float climb = math::max(0.0f, -vz - DECEL_VZ_DEAD);

	if (climb > 0.0f) {
		pitch_cap -= DECEL_CAP_DOWN * climb * ALT_TS;

	} else {
		pitch_cap += DECEL_CAP_UP * ALT_TS;
	}

	pitch_cap = math::constrain(pitch_cap, 0.0f, DECEL_PITCH_MAX);

	return math::min(DECEL_KP * math::max(0.0f, v_h), pitch_cap);
}

// backtrans_loop.m: the cruise -> hover back-transition state machine (blocker
// B5). This is what REPLACES decelLoop() above, and the difference is not a
// tuning difference but a mechanism difference.
//
// decelLoop commanded nose-up proportional to speed and HOPED `fx_sp = 0` would
// retract the tilts. The vehicle never slowed. Step 31 phase 0 measured why, by
// re-solving the allocator offline for every logged sample: what drives the tilt
// forward is the ALTITUDE (Fz) channel, not Fx. At 15 m/s the wing carries part
// of the load, the altitude loop asks for less lift (nu_des(Fz) = +2.9 N), and
// since dFz/ddelta = T*sin(delta) is large while dFz/dT = -cos(delta) -> 0 as
// tilt grows, tilting FORWARD is the allocator's cheapest way to dump lift.
// Positive feedback: tilt forward -> faster -> more wing lift -> larger demand.
// Zeroing the Fz demand reverses the drift; zeroing Fx changes nothing.
//
// Hence the tilt is driven by a BOX CONSTRAINT (the wing ceiling), never by an
// objective term -- Ws_Fx cannot outbid Ws_Fz, and raising it to that magnitude
// wrecked yaw in step 7. Step 19's lesson: in an incremental allocator you cannot
// impose an actuator configuration by preference, only by objective or constraint.
//
// *** AND THE CEILING MUST BE RELEASED AGAIN. *** Held on, delta0 pins to the
// ceiling and delta1 to TILT_MIN, so the differential tilt is not merely small
// but UNVARIABLE -- and differential tilt is yaw's only real actuator, so yaw
// authority goes to ZERO (only a fixed trim remains). Cruise aero damping masks
// that (9.1 Nm/rad at 11.6 m/s vs 0.27 at 2 m/s); slowing removes the mask, and
// the vehicle DEPARTED in yaw while braking (981 deg of rotation at pitch +4,
// 2117 deg at +6). Released: +-2 deg per phase, twice.
// This is not a compromise: the ceiling's justification is gone by then too
// (measured nu_des(Fz) ~ 0.00 at the floor -- the wing has stopped lifting).
//
// GENERAL LESSON: a constraint that stays in force after the problem it solved
// has gone becomes pure harm. Design when you will REMOVE a constraint at the
// same time you add it.
//
// ITEM (R) -- RETRACT'S EXIT SAT ON AN EQUILIBRIUM BOUNDARY TOO (step 38).
// The continuation of that lesson, and the same disease step 37 found in BRAKE,
// one phase earlier: designing WHEN to remove a constraint is not enough, you
// must also show the removal CONDITION IS REACHABLE.
// At the floor ceiling with pitch = 0 the vehicle settles at a terminal speed
// (yaw trim pins delta0 at the floor; the residual ~2.4 N balances drag) --
// measured 8.0-9.3 m/s against an exit threshold of 8.0. So the exit sometimes
// happened in 22 s and sometimes NEVER: one flight held RETRACT for 200.3 s and
// drifted +117.7 deg in yaw, because a ceiling parked at the trim leaves yaw
// with zero modulation authority. The exit is therefore two terms now:
//   1) v_h < BT_RELEASE_V  -- threshold moved OUTSIDE the equilibrium (8 -> 10).
//      Keeps the normal path fast, but the number belongs to SITL aerodynamics.
//   2) time at the floor >= BT_FLOOR_DWELL -- never looks at speed, so it holds
//      for ANY terminal speed. On hardware, where (1) may be mis-scaled, this
//      still bounds the time spent in the yaw-starving configuration.
// Term (2) is PROGRESS, not an abort: the dwell expires at the lowest speed
// RETRACT can deliver, and BRAKE raises the ceiling, which is what restores yaw.
//
// ITEM (S) -- A THRESHOLD MUST READ THE AXIS ITS LAW CONTROLS (step 39).
// The fourth sighting of the same disease, and this time neither the authority
// fading (step 37) nor the threshold sitting on an equilibrium (item R): the
// threshold was reading the WRONG SIGNAL.
// BRAKE -> HANDOFF tested `v_h < BT_HANDOFF_V`, and v_h is a MAGNITUDE. The
// brake law commands nose-up pitch, i.e. it controls the BODY FORWARD axis only;
// nothing controls the lateral axis until HANDOFF engages the position loop. So
// the component holding the magnitude above the threshold can be one the
// manoeuvre cannot remove. Measured (2026-07-31, sitl/diag_brake_reversal.py,
// log 14_13_11): min|v_h| = 3.08 m/s against a 3.0 threshold while body forward
// u = -0.51 m/s and lateral v = +3.04 m/s -- the vehicle had already stopped
// going forward. Handoff was never requested, the non-fading brake pitch
// (3.39 deg trim + 4.0 deg margin = 6.31 N against a 3.1-4.1 N push) kept
// pushing, and the vehicle ran away BACKWARD to 12.8 m/s; five times in one
// flight. Worse, a magnitude GROWS as the vehicle accelerates backward, so the
// threshold gets less reachable the further the failure runs: positive feedback.
// Fixed in two places, both from the same principle:
//   1) the exit reads SIGNED body forward speed (v_fwd < BT_HANDOFF_V). Signed
//      matters: if u is negative BRAKE's job is already done, and taking a
//      magnitude here would repeat the very mistake.
//   2) the braking MARGIN fades on max(0, v_fwd) too, so it is exactly zero once
//      the vehicle moves backward and only the station-keeping trim remains.
//      That is the term that actually stops the runaway; (1) alone changes the
//      state but would leave the push on.
// RETRACT deliberately keeps reading v_h: its question is not "do I still have
// forward speed to kill" but "is the wing still lifting, i.e. is the Fz-driven
// tilt runaway still possible", and that is an aerodynamic question whose answer
// is airspeed MAGNITUDE. Two phases may legitimately watch different signals --
// each threshold must ask ITS OWN law's question.
// KNOWN RESIDUAL, deliberately not closed here: pos_hold's engage gate
// (POS_ENGAGE_V_MAX) is still a magnitude gate, so with lateral drift the
// handoff can be requested and REFUSED, and retried every tick. During that wait
// pitch is at trim, the ceiling is released and the lateral axis is still
// uncontrolled -- strictly better than the runaway, but not a completed handoff.
// Measure the wait in SITL; if it is long the honest fixes are a lateral damper
// during BRAKE or a direction-aware engage gate, and each deserves its own
// measurement rather than a guess.
//
// States are one-way: overshooting the brake makes speed RISE again (+6 deg on a
// stopped vehicle drove it backward, 0.63 -> 5.77 m/s), so rising speed is not a
// valid reason to go back to RETRACT.
//
// `enable` going false RELEASES the ceiling (removing a constraint is the safe
// direction; leaving it clamped unsupervised kills yaw).
inline float btBrakePitch(float v_fwd)
{
	// Station-keeping trim + a speed-proportional braking margin. The trim term
	// does NOT fade: the forward force this law fights is structural and roughly
	// constant (delta1/delta2 pinned at TILT_MIN, so the yaw trim holds delta0 at
	// 10-15 deg = 3.1-4.1 N), so an angle is needed merely to STAND STILL. The old
	// single-term law faded to zero and stalled the manoeuvre above the handoff
	// speed -- twice at 3.2-3.5 m/s, and once more at 4.9 m/s even after the fade
	// point was moved. See BT_PITCH_MAX in TiltrotorIndiParams.hpp.
	// Derived from FX_TRIM (not a literal) so that re-measuring the trim on real
	// hardware carries into the brake law automatically. 50 Hz, so the asinf is free.
	//
	// ITEM (S) (step 39): the margin now fades on SIGNED body forward speed, not on
	// the horizontal magnitude. With the magnitude, a vehicle that had stopped going
	// forward and started going BACKWARD saw its margin grow again and was pushed
	// further backward -- measured, to 12.8 m/s. max(0, .) makes the margin exactly
	// zero at and below zero forward speed, i.e. "do not apply braking force when
	// you are already going backward", without a hard cut at the sign change.
	const float trim_pitch = asinf(math::min(1.0f, FX_TRIM / (MASS * GRAVITY)));
	return trim_pitch + BT_PITCH_MAX * math::min(1.0f, math::max(0.0f, v_fwd) / BT_BRAKE_V_FULL);
}

// Returns the wing tilt ceiling; writes the pitch setpoint and the pos_hold
// request. `state`, `tilt_ceil`, `tilt_floor` and `floor_dwell` carry across
// calls (expected at ALT_TS). `floor_dwell` is the item (R) backstop timer --
// see above.
//
// `v_h` is the horizontal speed MAGNITUDE and is used by RETRACT only (an
// aerodynamic question). `v_fwd` is the SIGNED body forward speed -- the
// horizontal velocity projected on the heading -- and is what BRAKE/HANDOFF
// switch on, because it is the axis the brake law controls (item S).
//
// `tilt_floor` (step 101): a LOWER bound on wing tilt, separate from and
// always kept below `tilt_ceil`. The ceiling alone (step 100) only stops the
// allocator being FORCED below what v_h justifies -- it does not stop the
// allocator CHOOSING a low tilt anyway once forward-Fx demand fades, which is
// what the step 100 measurement actually showed happening throughout BRAKE
// (measured tilt 8.4-13.2 deg at v_h = 4.6-9.6 m/s, i.e. resting WELL under
// even the unchanged 20 deg BRAKE ceiling -- the ceiling was never the
// binding constraint down there). A low tilt at real forward airspeed means
// the wing-rotor disks sit closer to the incoming flow's plane instead of
// axial to it -- the edgewise-loading risk this file cannot see in SITL (its
// propeller is an idealised disk with no blade-flapping model) but which is
// real on hardware. The floor forces the allocator to keep the disks leaned
// toward the direction of travel for as long as real airspeed says it should,
// regardless of whether the trim law "needs" that lean for Fx. See
// BT_TILT_FLOOR_MAX's declaration for why it is capped well under
// BT_BRAKE_CEIL: pinning floor == ceiling reproduces the exact yaw-starvation
// failure BT_CEIL_FLOOR's own header documents (delta1 pinned at TILT_MIN
// leaves delta0 nowhere to move) -- the floor must open genuine headroom
// under the ceiling, not close it.
inline float backTransition(bool enable, float v_h, float v_fwd, float delta_wing_max,
			    BtState &state, float &tilt_ceil, float &tilt_floor, float &floor_dwell,
			    float &pitch_sp, bool &req_pos_hold)
{
	pitch_sp = 0.0f;
	req_pos_hold = false;

	if (!enable) {
		state = BtState::IDLE;
		tilt_ceil = TILT_MAX;
		tilt_floor = TILT_MIN;
		floor_dwell = 0.0f;
		return tilt_ceil;
	}

	// Shared by RETRACT and BRAKE (step 101): the floor tapers to TILT_MIN as
	// v_h -> 0, so it never fights HANDOFF's need for a true hover tilt, and it
	// re-opens/closes with CURRENT v_h exactly like the ceiling (step 100) --
	// no separate timer, no way to be caught behind the real deceleration.
	const float v_frac_floor = math::constrain(v_h / BT_TILT_V_REF, 0.0f, 1.0f);
	const float floor_target = TILT_MIN + v_frac_floor * (BT_TILT_FLOOR_MAX - TILT_MIN);

	switch (state) {
	case BtState::IDLE:
		// Rising edge: start the ceiling at the CURRENT tilt so it binds at once.
		state = BtState::RETRACT;
		tilt_ceil = math::min(TILT_MAX, delta_wing_max);
		tilt_floor = floor_target;
		floor_dwell = 0.0f;
		break;

	case BtState::RETRACT: {
		// Step 100: tilt_ceil is a direct function of the CURRENT v_h, not a
		// clocked ramp -- see BT_TILT_V_REF's declaration for the measured
		// failure this replaces (tilt pinned near-vertical for ~8 s while v_h
		// was still 3.7-8.9 m/s). Recomputed fresh every tick: it can never be
		// caught "ahead" of the real deceleration, and it re-opens immediately
		// if v_h rises again (a gust), rather than staying wherever a timer
		// left it.
		const float v_frac = math::constrain(v_h / BT_TILT_V_REF, 0.0f, 1.0f);
		tilt_ceil = BT_CEIL_FLOOR + v_frac * (TILT_MAX - BT_CEIL_FLOOR);
		tilt_floor = floor_target;

		// Aero-independent backstop (item (R), step 38): elapsed time IN
		// RETRACT, no longer gated on "at the floor" -- with the ceiling now
		// tracking v_h directly, gating the timer on the floor would only ever
		// start it once v_h was already near zero, which defeats its purpose
		// as a fallback for a v_h reading that never converges.
		floor_dwell += ALT_TS;

		if (v_h < BT_RELEASE_V || floor_dwell >= BT_FLOOR_DWELL) {
			state = BtState::BRAKE;
		}

		break;
	}

	case BtState::BRAKE:
		// RAISE the ceiling, do not remove it. Raising it off the trim gives yaw
		// its modulation back (delta1 sits on TILT_MIN, so the differential is
		// delta0, now free over [0, BT_BRAKE_CEIL]); keeping a ceiling at all is
		// what stops the Fz-driven runaway from restarting -- releasing fully at
		// this speed made the manoeuvre undo itself. See BT_BRAKE_CEIL.
		tilt_ceil = BT_BRAKE_CEIL;
		// BT_TILT_FLOOR_MAX is kept comfortably under BT_BRAKE_CEIL (see its
		// declaration) so this never closes the box BRAKE just opened.
		tilt_floor = floor_target;
		pitch_sp = btBrakePitch(v_fwd);

		// Item (S): SIGNED body forward speed. A magnitude can be held above
		// this threshold forever by a lateral component the manoeuvre cannot
		// remove -- measured, and the vehicle ran away backward. v_fwd < 0 is
		// a valid exit: BRAKE's job was to end forward motion, and it has.
		if (v_fwd < BT_HANDOFF_V) {
			state = BtState::HANDOFF;
		}

		break;

	case BtState::HANDOFF:
	default:
		// Now the constraint really can go: below BT_HANDOFF_V there is no wing
		// lift left to drive the runaway, and leaving a constraint in force after
		// the problem it solved has gone is the exact mistake this file warns
		// about at the top. Same for the floor -- by now v_fwd < BT_HANDOFF_V
		// (3 m/s) so floor_target is already small, but HANDOFF's job is to
		// reach true hover, and nothing should stand between it and TILT_MIN.
		tilt_ceil = TILT_MAX;
		tilt_floor = TILT_MIN;
		pitch_sp = btBrakePitch(v_fwd);
		req_pos_hold = true;
		break;
	}

	return tilt_ceil;
}

// forwardtrans_loop.m: the hover -> cruise FORWARD transition (item (V)).
// The mirror of backTransition() above, written 2026-08-03 (step 42) once the
// requirement was stated as a fully AUTONOMOUS mission: takeoff MC -> cruise
// fixed-wing -> land MC. Until then the forward direction had no law at all --
// only an open `fx_sp` ramped by hand from the debug console, which is a
// capability nothing on board could invoke.
//
// IT COMMANDS NO TILT ANGLE, AND THAT IS THE POINT. This machine grows `fx_cmd`
// and lets the WLS choose the tilt (measured: 45-54 deg at 15 m/s). That is the
// OPPOSITE mechanism from the back-transition, for a measured reason. Step 31
// phase 0 showed the allocator's own preference is what moves the tilt: going
// FORWARD that preference already points the right way (as the wing loads up,
// dFz/dT shrinks and tilting is cheap), so an objective term suffices. Going
// BACK it points the wrong way, and only a BOX CONSTRAINT could overrule it.
// *** General: which tool moves an actuator depends on which way the allocator
// already wants to go. Same actuator, opposite mechanism. ***
//
// PITCH IS HELD AT ZERO. Step 29 measured that above ~5-6 m/s nose-up on this
// airframe is a CLIMB command, not an acceleration one, and the altitude loop
// cannot fight wing lift -- engaging pitch at 14.5 m/s produced a 35 s, 1.1 m/s
// climb. Accelerating is exactly when that lever is most dangerous, so altitude
// stays with the altitude loop (Fz) alone. Same reasoning as RETRACT's pitch = 0.
//
// pos_hold MUST BE RELEASED: while active the position loop OWNS roll/pitch and
// supplies fx_trim (items (N)/(P)), so it would simply overwrite this command.
// Measured clean: at release the rate activity DROPS (p RMS 0.2123 -> 0.1503).
//
// AND THIS MANOEUVRE IS A ONE-WAY DOOR. The moment v_h passes POS_ENGAGE_V_MAX
// (3 m/s) the hold refuses to re-engage (step 29's gate), so the only way back
// to hover is the BACK-transition. That is why an abort here is NOT `fx = 0`:
// step 30 tried precisely that and the vehicle never slowed, because Fx is far
// too weak an objective term to retract the tilts. Aborting REQUESTS THE
// BACK-TRANSITION, which is why `req_abort` is an output rather than an
// internal state.
//
// Two safety terms, deliberately of different kinds (step 38's lesson -- every
// threshold is a measurement OF AN ENVIRONMENT, so pair it with one that is not):
//   1) ALTITUDE BAND -- |z - z_entry| > FT_ALT_BAND. Aero-dependent, and the
//      direct detector for step 29's escape climb.
//   2) TIME -- RAMP that has not reached FT_CRUISE_V within FT_TIMEOUT_S aborts
//      WITHOUT looking at speed at all, so it holds for any real wing.
//
// *** ABORTING IS OFF BY DEFAULT (FT_ALLOW_ABORT = false, 2026-08-04). ***
// REQUIREMENT: the mission profile is one piece -- multicopter takeoff, FIXED-WING
// cruise, tilt-rotor transitions -- and THE FORWARD TRANSITION DOES NOT CANCEL.
// Neither safety term was deleted: both still evaluate, and both are reported
// through `warn_code` (and logged). Only their ACTION is gone.
//
// The COST is stated plainly: step 29's escape climb no longer triggers an
// automatic response, only a report. The BENEFIT is equally measured -- the abort
// had no guaranteed escape path. Aborting means REQUESTING the back-transition
// (step 30: `fx_sp = 0` does not retract the tilts, the vehicle does not slow),
// and the back-transition REFUSES to start below BT_MIN_ALT = 15 m. The constants
// left exactly zero margin for that: FT_MIN_ALT(20) - FT_ALT_BAND(5) = 15 =
// BT_MIN_ALT. So a transition aborting while DESCENDING did so at precisely the
// altitude where its escape is refused -- and in that corner aborting was WORSE
// than continuing (cruise speed, tilts forward, nothing owning the vehicle; the
// same shape as items (U) and (R)). If the abort is ever re-enabled, close that
// margin FIRST.
//
// warn_code: 0 = none, 1 = altitude band, 2 = timeout.
//
// `state`, `fx_cmd`, `z_entry` and `t_ramp` carry across calls (expected at ALT_TS).
inline float forwardTransition(bool enable, float v_h, float z, bool sat_thrust,
			       FtState &state, float &fx_cmd, float &z_entry, float &t_ramp,
			       float &pitch_sp, bool &release_hold, bool &req_abort,
			       uint8_t &warn_code)
{
	pitch_sp = 0.0f;          // step 29 -- see above
	release_hold = false;
	req_abort = false;
	warn_code = 0;

	if (!enable) {
		state = FtState::IDLE;
		fx_cmd = 0.0f;
		z_entry = 0.0f;
		t_ramp = 0.0f;
		return fx_cmd;
	}

	switch (state) {
	case FtState::IDLE:
		// Rising edge: freeze the altitude-band reference HERE.
		state = FtState::RAMP;
		fx_cmd = 0.0f;
		z_entry = z;
		t_ramp = 0.0f;
		release_hold = true;
		break;

	case FtState::RAMP:
		release_hold = true;
		t_ramp += ALT_TS;

		// Saturated thrust is a reason to WAIT, not to abort: the allocator is
		// already on a box bound, so asking for more Fx only creates a demand it
		// cannot solve (step 11's lesson).
		if (!sat_thrust) {
			fx_cmd = math::min(FT_FX_CRUISE, fx_cmd + FT_FX_RATE * ALT_TS);
		}

		// The detectors always run; only the ACTION depends on FT_ALLOW_ABORT.
		if (fabsf(z - z_entry) > FT_ALT_BAND) {
			warn_code = 1;

		} else if (t_ramp >= FT_TIMEOUT_S && v_h < FT_CRUISE_V) {
			warn_code = 2;
		}

		if (warn_code != 0 && FT_ALLOW_ABORT) {
			req_abort = true;

		} else if (fx_cmd >= FT_FX_CRUISE - 1e-6f && v_h >= FT_CRUISE_V) {
			// With aborting off a warning does NOT block the handover: the
			// manoeuvre completes and the warning is only reported.
			state = FtState::CRUISE;
		}

		break;

	case FtState::CRUISE:
	default:
		release_hold = true;
		fx_cmd = FT_FX_CRUISE;

		if (fabsf(z - z_entry) > FT_ALT_BAND) {
			warn_code = 1;

			if (FT_ALLOW_ABORT) {
				req_abort = true;
			}
		}

		break;
	}

	return fx_cmd;
}

// fixed_wing_transition (Adim 58-61 MATLAB port: run_full_transition_glide_test.m's
// GLIDE/relight logic). See the FW_* constants block in TiltrotorIndiParams.hpp
// for the full rationale -- this is only the STATE MACHINE (mirrors
// forwardTransition()'s shape): IDLE -> GLIDE cuts every rotor (the caller does
// that, not this function -- see the u_cmd branch in Run()) and free-slews the
// wing tilt to TILT_MAX; once there, ACTIVE hands off to fixedWingControlLaw()
// below. Called every tick (the tilt ramp needs no decimation, unlike
// forwardTransition()'s ALT_TS -- there is no allocator box or LESO here to
// synchronise with).
//
// `tilt` is seeded by the CALLER on the IDLE->GLIDE edge (from the current
// shadow tilt state, mirroring forwardTransition() seeding z_entry from the
// current z) -- this function only advances it.
//
// `return_request`: step 103's mirror path. Requesting it while ACTIVE (or
// GLIDE) moves to RETURN, which slews `tilt` back DOWN to TILT_MIN at the
// same rate GLIDE slews it up -- see RETURN's own case below and the FwState
// comment in TiltrotorIndiParams.hpp for why this exists and what it
// replaced. Ignored in every other state (RETURN cannot be re-entered mid-
// RETURN, and requesting it from IDLE/GLIDE-not-yet-ACTIVE is meaningless --
// GLIDE already reaches ACTIVE in ~4.5s on its own).
inline void fixedWingTransition(bool enable, bool return_request, float dt, FwState &state, float &tilt)
{
	switch (state) {
	case FwState::IDLE:
		if (enable) {
			state = FwState::GLIDE;
		}

		break;

	case FwState::GLIDE:
		tilt = fminf(tilt + FW_TILT_RAMP_RATE * dt, TILT_MAX);

		if (tilt >= TILT_MAX - 1e-3f) {
			state = FwState::ACTIVE;
		}

		break;

	case FwState::ACTIVE:
		if (return_request) {
			state = FwState::RETURN;
		} else {
			tilt = TILT_MAX;
		}

		break;

	case FwState::RETURN:
		// Symmetric with GLIDE, opposite direction: the caller (Run()) mirrors
		// GLIDE's own u_cmd assembly for this state too -- every rotor cut,
		// surfaces alone holding attitude with GLIDE's own wings-level/pitch-
		// hold law, tilt free-slewing. Reaching TILT_MIN hands off to IDLE,
		// which is the ordinary hover-arm starting point (rotors off, tilt
		// vertical) that the WLS/INDI path already starts every flight from --
		// not a new case for it to handle.
		tilt = fmaxf(tilt - FW_TILT_RAMP_RATE * dt, TILT_MIN);

		if (tilt <= TILT_MIN + 1e-3f) {
			state = FwState::IDLE;
		}

		break;

	default:
		tilt = TILT_MAX;
		break;
	}
}

// fixedwing_control_law.m port (Adim 59, sign-fixed Adim 59, end-to-end
// validated Adim 60-61, ARCHITECTURE CORRECTED Adim 75-77). Called every
// tick, same cadence as the INDI path it replaces (MATLAB validated it at
// the full p.Ts_ctrl rate, not decimated). Runs ONLY once FwState::ACTIVE
// (tilt already at TILT_MAX, T2 already 0 -- both enforced by the caller,
// not by this function). Conventional-airplane-style autopilot, NOT INDI/WLS:
//   heading <- BANK ANGLE (bank-to-turn, see FW_KP_HDG's header note in
//              TiltrotorIndiParams.hpp for why -- this replaced holding
//              heading with rudder directly, which caused Adim 66-74's whole
//              banked-turn-equilibrium saga)
//   roll    <- aileron (differential elevon, PD on the commanded bank + p)
//   pitch   <- elevator (symmetric servo_2/3, PD on theta + q); theta_sp is a
//              feed-forward trim (analytic, see FW_KP_ALT's header comment)
//              plus a small PI correction on altitude
//   yaw     <- NOTHING (Adim 77: rudder removed, passive stability +
//              bank-to-turn alone matched the with-rudder result exactly)
//   speed   <- shared wing-rotor thrust (T0=T1), PI with a measured feed-forward
//
// Surface signs are derived from effectivenessMatrix()'s own SURF_CPX/CPY/CPZ,
// SURF_EX/EY/EZ, SURF_K (verified by hand against MATLAB's
// effectiveness_matrix.m: elevon differential -> tau_x = -1.2*qbar*a_ail,
// elevator symmetric -> tau_y = +0.806*qbar*a_ele -- matching coefficients to
// 3 significant figures from an independently-derived model, which is why
// this law can trust MATLAB's PD gains directly rather than re-tuning them
// here).
//
// `v_I`/`alt_I` carry across calls (MATLAB's state_in/state_out =
// [integral_v; integral_alt]) -- caller owns them as persistent members, same
// pattern as _cruise_fx_I/_cruise_pitch_I.
//
// `v_fwd_filt` (2026-08-21, Adim 68) is a SEPARATE persistent state from the
// raw `v_fwd` used by the speed loop below -- added after Adim 67 found a
// clean, coordinated GLIDE entry still diverging in ACTIVE purely from the
// transition's own violence: GLIDE is an unpowered dive that gains real
// energy (Adim 61/66/67 all measured 7-8 -> 14+ m/s in 2-3 s), and theta_ff
// below is proportional to 1/v_fwd^2 through qbar_ff -- so a raw,
// instantaneous v_fwd feeds a RAPIDLY SWINGING pitch reference into the loop
// during exactly the few seconds attitude is most vulnerable. Filtering only
// the feed-forward's input (not the speed loop's own error signal, which
// must stay responsive) is the same "smooth the reference, not the
// tracking" principle as cruiseSpeedLoop's fx_track bumpless handover and
// _ft_z_entry/_man_yaw_sp's entry-captured targets elsewhere in this module.
inline void fixedWingControlLaw(float hdg_sp, float z_sp, float v_sp,
				 const matrix::Vector3f &att, const matrix::Vector3f &omega,
				 float z, float v_fwd, float dt,
				 float &v_I, float &alt_I, float &v_fwd_filt,
				 float &T_wing, float &T_tail, float surf[N_SURF])
{
	// --- pitch feed-forward trim: theta_ff = (W + Fz_target)/(qbar*S*cla) - a0,
	// Fz_target = 0 (see FW_KP_ALT's header comment for why this differs from
	// TECS_PITCH_FZ_SP = -12 N) ---
	v_fwd_filt += (v_fwd - v_fwd_filt) * math::constrain(dt / FW_V_FILT_TAU, 0.0f, 1.0f);
	const float v_ff = fmaxf(v_fwd_filt, FW_V_QBAR_MIN);
	const float qbar_ff = 0.5f * AERO_RHO * v_ff * v_ff;
	const float dLdth = qbar_ff * AERO_WING_S * AERO_CLA;
	const float theta_ff = MASS * GRAVITY / dLdth - AERO_WING_A0;

	// --- altitude PI correction on top of the feed-forward ---
	// NED down-positive: err_z > 0 <=> below target <=> more nose-up needed.
	// THIS SIGN (z - z_sp, not z_sp - z) is Adim 59's critical fix -- see the
	// FW_KP_ALT header comment in TiltrotorIndiParams.hpp.
	const float err_z = z - z_sp;
	alt_I = math::constrain(alt_I + err_z * dt, -FW_ALT_I_MAX, FW_ALT_I_MAX);
	const float pitch_corr = math::constrain(FW_KP_ALT * err_z + FW_KI_ALT * alt_I,
			-FW_PITCH_CORR_MAX, FW_PITCH_CORR_MAX);
	const float pitch_sp = theta_ff + pitch_corr;

	const float phi = att(0), theta = att(1), psi = att(2);
	const float p_rate = omega(0), q_rate = omega(1);

	// --- bank-to-turn: heading error -> commanded roll angle (Adim 75) ---
	const float e_hdg = atan2f(sinf(hdg_sp - psi), cosf(hdg_sp - psi));
	const float roll_sp = math::constrain(FW_KP_HDG * e_hdg, -FW_MAX_BANK, FW_MAX_BANK);

	const float e_roll  = roll_sp  - phi;
	const float e_pitch = pitch_sp - theta;

	const float a_ail = -(FW_KP_ROLL  * e_roll  - FW_KD_ROLL  * p_rate);
	const float a_ele =  (FW_KP_PITCH * e_pitch - FW_KD_PITCH * q_rate);

	surf[0] =  a_ail; surf[1] = -a_ail; // elevon L/R (s0/s1): differential = aileron
	surf[2] =  a_ele; surf[3] =  a_ele; // elevator L/R (s2/s3): symmetric = elevator
	surf[4] =  0.0f;                    // rudder (s4): removed, Adim 77 -- see FW_MAX_BANK's header note

	// --- speed: shared wing-rotor thrust, feed-forward + PI ---
	const float err_v = v_sp - v_fwd;
	v_I = math::constrain(v_I + err_v * dt, -FW_V_I_MAX, FW_V_I_MAX);
	T_wing = math::constrain(FW_T_FF + FW_KP_V * err_v + FW_KI_V * v_I, ROTOR_TMIN, ROTOR_TMAX);

	// --- tail-rotor pitch backstop (Adim 69, see FW_TAIL_PITCH_LIMIT's header
	// note) -- reacts to the ACTUAL pitch (theta), not pitch_sp: this is a
	// backstop against the vehicle exceeding a safe attitude, not a tracking
	// loop, so it must not care what the setpoint calculation currently wants.
	// Proportional-only and always >= 0: T2's own [ROTOR_TMIN, ROTOR_TMAX]
	// clamp is applied by the caller (Run()'s shared actuator clamp), same as
	// every other actuator channel.
	// GATED OFF during any real bank (Adim 81, see FW_TAIL_ROLL_GATE's header
	// note) -- a banked turn needs MORE nose-up authority for lift, not a
	// nose-down backstop fighting the elevator for it.
	T_tail = (fabsf(phi) < FW_TAIL_ROLL_GATE) ? FW_KP_TAIL * fmaxf(0.0f, theta - FW_TAIL_PITCH_LIMIT) : 0.0f;
}

// cruise_speed_loop.m: takes fx OUT OF OPEN LOOP (step 47, 2026-08-03).
//
// WHY IT EXISTS -- not a choice, the answer to a measured obstacle. Until this
// loop, `fx` was not a SPEED command but an ACCELERATION command: a constant
// force, with the equilibrium speed set by drag and, above ~12 N, by the wing
// tilt's 90 deg MECHANICAL STOP. That stop was an accidental protection, and
// step 46 measured that enabling the control surfaces REMOVES it -- the vehicle
// keeps accelerating into a regime it cannot trim. So closing the speed loop is
// a PREREQUISITE for the surfaces, not a refinement of them.
//
// BUMPLESS HANDOVER is the reason for `fx_track`: the instant this loop engages,
// its output must equal the fx already being applied, because on this airframe
// an fx step IS a tilt transient. While the loop is not active the integrator
// TRACKS the applied value, so engagement is exactly continuous.
//
// `v_fwd` is the SIGNED body forward speed, never the magnitude -- item (S)'s
// lesson: a law that controls the forward axis must read that axis, or lateral
// drift looks like a speed error.
//
// `active` is passed rather than latched (MATLAB carries it in state_in(2); the
// semantics are identical, the caller owns the condition either way).
inline float cruiseSpeedLoop(bool active, float v_fwd, float fx_track, float &I, float v_sp)
{
	if (!TECS_ENABLE) {
		// Feature off: behaviour is BIT-IDENTICAL to the step 42/46 open loop.
		I = fx_track;
		return fx_track;
	}

	if (!active) {
		I = math::constrain(fx_track, 0.0f, TECS_FX_MAX);
		return fx_track;
	}

	const float e = v_sp - v_fwd;
	const float P = TECS_KP * e;

	float I_cand = math::constrain(I + TECS_KI * e * ALT_TS, 0.0f, TECS_FX_MAX);
	const float fx_cand = I_cand + P;

	if (fx_cand > TECS_FX_MAX || fx_cand < 0.0f) {
		// Output saturated: FREEZE the integrator if the update pushes further out
		// (conditional integration -- stops an accumulated error from kicking back
		// when the bound is released).
		const bool pushing_out = (fx_cand > TECS_FX_MAX && e > 0.0f)
					 || (fx_cand < 0.0f && e < 0.0f);

		if (pushing_out) {
			I_cand = I;
		}
	}

	I = I_cand;
	return math::constrain(I + P, 0.0f, TECS_FX_MAX);
}

// cruise_pitch_loop.m: the ENERGY-DISTRIBUTION (pitch) half, steps 49 + 53.
//
// WHAT IT IS: an angle-of-attack TRIM, not a manoeuvre loop. Altitude is still
// held by the altitude loop (the Fz channel); this only takes that loop's
// PERSISTENT load and hands it to the wing -- like an autopilot's elevator trim
// following the servo load. The two do not fight because they are separated in
// TIME SCALE by ~10x, and TECS_PITCH_KI is derived from exactly that.
//
// THE OBJECTIVE: `Fz_sp` is precisely "the vertical force the actuators must
// carry", so driving it to a target IS energy distribution in this
// parametrisation. More negative than target (actuators overloaded, wing not
// pulling its weight) -> nose UP; more positive -> nose DOWN. Above V_wb =
// 16.9 m/s the needed answer is the second one, and pitch was hard-coded to zero
// everywhere, which is what step 47 measured as the speed wall.
//
// THE FEED-FORWARD (step 53) IS WHAT REMOVED THAT WALL, and it is worth being
// precise about why, because four other candidates were refuted first (elevator
// weight, speed-loop gain, a wing-rotor thrust floor, the cruise tilt ceiling).
// Measured channel-by-channel just before divergence at 19 m/s: the REQUIRED
// trim was -0.71 deg against a +-6 deg limit (12% of authority) and the law had
// reached -0.59 deg. Neither authority nor saturation -- the law had simply LOST
// THE RACE, because it was a pure integrator (tau ~ 19-24 s) chasing a target
// that moves as speed rises. The closed form was already known:
//
//     theta_ff(V) = (W + Fz_target) / (qbar*S*cla) - a0
//
// With it in the law the integrator only closes MODEL ERROR, and 16/17/18/19/20/
// 22/24/26 m/s are all stable where 19 and 20 used to diverge at ~39 s.
//
// AGAINST MODEL ERROR (this matters on hardware, where S/cla/a0/rho are
// uncertain): two structural protections. The term is clamped to
// TECS_PITCH_MAX, and the required range is itself small (+3.4 deg at 12 m/s,
// -1.0 deg at 20), so even a 100% model error is a few degrees of command; and
// the integrator is still there, closing the remainder at the derived tau -- the
// feed-forward works IN FRONT OF it, not instead of it.
inline float cruisePitchLoop(float v_fwd, float fz_sp, float &I)
{
	if (!TECS_PITCH_ENABLE) {
		I = 0.0f;
		return 0.0f;
	}

	// Gate: smoothstep, same shape as the gain schedule -- a continuous blend,
	// not a second state machine. Below V_ON the authority is EXACTLY zero
	// (step 29's regime, where nose-up is a climb command).
	float s = (v_fwd - TECS_PITCH_V_ON) / (TECS_PITCH_V_FULL - TECS_PITCH_V_ON);
	s = math::constrain(s, 0.0f, 1.0f);
	const float w = 3.0f * s * s - 2.0f * s * s * s;

	if (w <= 0.0f) {
		// Gate shut: also zero the integrator, so opening it does not start from
		// an accumulated history (bumpless entry).
		I = 0.0f;
		return 0.0f;
	}

	const float lim = TECS_PITCH_MAX;

	// --- feed-forward: the analytic trim (step 53) ---
	// The speed floor is the gate's lower edge. w > 0 already implies
	// v_fwd > V_ON, but the guard lives in the expression itself so that a future
	// change to the gate cannot make this divide silently blow up.
	const float v_ff = fmaxf(v_fwd, TECS_PITCH_V_ON);
	const float qbar_ff = 0.5f * AERO_RHO * v_ff * v_ff;
	const float dLdth = qbar_ff * AERO_WING_S * AERO_CLA;      // N/rad
	float th_ff = (MASS * GRAVITY + TECS_PITCH_FZ_SP) / dLdth - AERO_WING_A0;
	th_ff = math::constrain(th_ff, -lim, lim);

	// --- integral trim: MODEL ERROR only ---
	const float err = fz_sp - TECS_PITCH_FZ_SP;
	float I_cand = I - TECS_PITCH_KI * err * ALT_TS;

	// Anti-windup, and the criterion is the TOTAL command (th_ff + I), not the
	// integrator. Before the feed-forward the two were the same thing; now they
	// differ, and the honest test is the value actually commanded.
	const float th_tot = th_ff + I_cand;

	if (th_tot > lim || th_tot < -lim) {
		const bool pushing_out = (th_tot > lim && err < 0.0f)
					 || (th_tot < -lim && err > 0.0f);

		if (pushing_out) {
			I_cand = I;
		}
	}

	// The hard clamp is written against the TOTAL as well, i.e. I is bounded to
	// [-lim - th_ff, lim - th_ff], so clamp and conditional integration look at
	// the SAME limit. Clamping I directly to +-lim instead is a real defect that
	// was measured: with th_ff of the opposite sign the hard clamp binds first
	// and the total can NEVER reach the limit -- authority silently cut to
	// 4.4 deg instead of 6.0 at 20 m/s.
	I = math::constrain(I_cand, -lim - th_ff, lim - th_ff);

	return w * math::constrain(th_ff + I, -lim, lim);
}

// hover_trim.m: closed-form hover trim (Fz + pitch balance at delta=0, plus a
// single-axis differential-tilt correction on rotor 1 to null the yaw
// reaction-torque imbalance from 2xCCW+1xCW rotors). Used to seed the shadow
// actuator model on arm -- MATLAB's test scripts start from this analytic
// trim directly (x0(14:19)=u_trim), never from a cold naive guess. Seeding
// the shadow model naively (equal thrust split, zero tilt) instead produces
// a large (~90 deg observed in SITL) yaw excursion in the first ~1s after
// arming, before the intentionally-low-priority (Ws_yaw, see WLS weights)
// yaw correction has time to act -- a transient the MATLAB tests never had
// to handle since they start pre-trimmed.
inline matrix::Vector<float, N_ACT> hoverTrim()
{
	const float arm_ratio = 2.0f * ROTOR_PX[0] / fabsf(ROTOR_PX[2]); // wing PX / |tail PX|
	const float Tw = MASS * GRAVITY / (2.0f + arm_ratio);
	const float Tt = arm_ratio * Tw;

	matrix::Vector<float, N_ACT> u0;
	u0(0) = Tw; u0(1) = Tw; u0(2) = Tt;
	u0(3) = 0.0f; u0(4) = 0.0f; u0(5) = 0.0f;

	for (int j = 0; j < N_SURF; j++) { u0(6 + j) = 0.0f; }

	matrix::Matrix<float, 5, N_ACT> G0;
	matrix::Vector<float, 5> nu0;
	// qbar = 0: this is the HOVER trim by definition, and at zero dynamic
	// pressure the surfaces have no effectiveness, so the trim is unchanged
	// from the 6-actuator version.
	effectivenessMatrix(u0, 0.0f, G0, nu0);

	// Step 12 (2026-07-27): with the corrected ROTOR_KM signs the net yaw
	// reaction torque changed sign, so nulling it via rotor 1's tilt would now
	// need a NEGATIVE tilt -- infeasible, tilt is physically limited to
	// [TILT_MIN, TILT_MAX] = [0, pi/2]. The opposite wing rotor (opposite PY
	// sign) gives the exact equivalent correction with a POSITIVE tilt, so the
	// trim picks whichever wing rotor yields a feasible (>= 0) correction.
	const float d0_trim = -nu0(2) / G0(2, 3); // d(tauz)/d(delta0)
	const float d1_trim = -nu0(2) / G0(2, 4); // d(tauz)/d(delta1), opposite sign

	matrix::Vector<float, N_ACT> u_trim = u0;

	if (d0_trim >= 0.0f) {
		u_trim(3) = math::min(d0_trim, TILT_MAX);

	} else {
		u_trim(4) = math::min(d1_trim, TILT_MAX);
	}

	return u_trim;
}

// ---------------------------------------------------------------------------
// INIS DIZISI (2026-08-31, Adim 153 -- madde B0)
// ---------------------------------------------------------------------------
// forwardTransition()/backTransition() ile AYNI SEKIL: saf fonksiyon, durumu
// disaridan referansla alir, hicbir yere yazmaz. Boylece MATLAB tarafinda
// birebir ayni aritmetikle sinanabilir.
//
// NEDEN VAR: bu profil PC tarafindaki run_mission_test.py icindeydi ve o
// betik hedefi POSIX kabuk istemcisiyle gonderiyordu -- gercek kartta olmayan
// bir yol (madde B0). Sayilar oradan BIREBIR tasindi; gerekceleri
// TiltrotorIndiParams.hpp'deki LAND_* blogunda.
//
// GIRDILER
//   enable   : setpoint.land_enable
//   z        : lpos.z (NED, asagi pozitif)
//   z_datum  : YER referansi -- modulun disarm'da yakaladigi deger. Betikteki
//              `z0` ile ayni sey; ikisi de arac YERDEYKEN okunur.
//   ctz      : gerceklesen DIKEY itki toplami [N] (temas olcutu icin)
//
// CIKTI: z_cmd (irtifa dongusune verilecek hedef). enable=false iken IDLE'a
// doner ve z_cmd = z (tut).
//
// MODUL DISARM ETMEZ. TOUCHDOWN yalnizca "temas olustu" der; arming karari
// disaridadir. PX4'un kendi mimarisi de boyle ayirir (land_detector bildirir,
// commander karar verir) ve profili tasimak arming yetkisini tasimak degildir.
inline float landingSequence(bool enable, float z, float z_datum, float ctz,
			     LandState &state, float &step_timer, float &touch_dwell,
			     float &stall_dwell, float &z_step, float dt)
{
	if (!enable || !PX4_ISFINITE(z) || !PX4_ISFINITE(z_datum)) {
		state = LandState::IDLE;
		step_timer = 0.f;
		touch_dwell = 0.f;
		stall_dwell = 0.f;
		z_step = z;
		return z;
	}

	// AGL, DATUMA GORE (Adim 117'nin dersi): ham -z DEGIL. Olculen datum
	// ofseti kosumdan kosuma -1.0 .. +0.9 m arasinda degisiyor ve mutlak bir
	// esik o yuzden guvenilmez.
	const float agl = z_datum - z;

	// TEMAS: dikey itki agirligin altinda VE bu kesintisiz sursun.
	const bool low_thrust = PX4_ISFINITE(ctz)
				&& (ctz < LAND_GROUND_THRUST_FRAC * MASS * GRAVITY);
	touch_dwell = low_thrust ? (touch_dwell + dt) : 0.f;
	const bool contact = (touch_dwell >= LAND_TOUCH_DWELL) || (agl < LAND_DONE_ALT);

	switch (state) {
	case LandState::IDLE:
		state = LandState::DESCEND;
		z_step = z;
		step_timer = 0.f;
		stall_dwell = 0.f;
		break;

	case LandState::DESCEND: {
			// KADEME: hedef her LAND_STEP_S'de bir LAND_STEP_M asagi iner.
			// Surekli rampa DENENDI VE GERI ALINDI (Adim 134): inis hizini
			// dusurdu ama salinim frekansi degismedi (0.417 -> 0.419 Hz) ve
			// pitch tepe-tepe 7.96 -> 11.00 deg KOTULESTI. Yani salinim
			// profilin uyarmasi degil, sistemin kendi modu.
			step_timer += dt;

			if (step_timer >= LAND_STEP_S) {
				step_timer = 0.f;
				z_step += LAND_STEP_M;
			}

			// TAKILMA: irtifa ilerlemiyorsa say. Modul bir sey YAPMAZ --
			// yalnizca durumu tasir; karar disaridadir.
			stall_dwell = (fabsf(z_step - z) > LAND_STEP_M + LAND_STALL_DZ)
				      ? (stall_dwell + dt) : 0.f;

			if (agl < LAND_FLARE_ALT) {
				state = LandState::FLARE;
			}

			if (contact) {
				state = LandState::TOUCHDOWN;
			}

			return z_step;
		}

	case LandState::FLARE:
		// Hedef YERIN ALTINA surulur: "0.30 m'de asili kal" demek, aracin
		// yer etkisinde takilmasi demekti (2026-08-28 olcumu). Irtifa
		// dongusu boylece temasa kadar alcalmayi surdurur.
		if (contact) {
			state = LandState::TOUCHDOWN;
		}

		return z_datum + LAND_TOUCH_Z;

	case LandState::TOUCHDOWN:
	default:
		// Temas olustu. Hedef yerde tutulur; disarm karari disarida.
		return z_datum + LAND_TOUCH_Z;
	}

	return z_step;
}

// ---------------------------------------------------------------------------
// GOREV DIZICISI (2026-08-31, Adim 154 -- madde B0'in kalan yarisi)
// ---------------------------------------------------------------------------
// Adim 153 inis PROFILINI tasidi; bu, BAYRAKLARI da tasir. Tek `enable` ile
// tam gorev modulden yurur: tirmanis -> hover -> ileri gecis -> seyir ->
// sabit kanat -> geri gecis -> oturma -> inis.
//
// Dizici mevcut durum makinelerinin USTUNDE durur; hicbirinin isini yapmaz,
// yalnizca "hangi bayrak ne zaman" der. Gecisler yine forwardTransition(),
// backTransition(), fixedWing*() ve landingSequence() tarafindan yurutulur.
//
// GECISLER OLAY TABANLI (betikteki wait_until ile ayni): ft_state==CRUISE,
// bt_state==HANDOFF, fw_state==ACTIVE, land_state==TOUCHDOWN. Yalnizca seyir
// sureleri ve hover oturmasi zamanlidir -- onlar bir olayi degil bir SUREYI
// bekler.
//
// TIMEOUT: bir olay MSN_PHASE_TIMEOUT_S icinde gelmezse dizici GERI GECISE
// duser (BACK). Bu bilerek en guvenli yon: her arizada arac hover'a doner ve
// oradan iner. Ileri gitmek ya da oldugu yerde donmak degil.
//
// MODUL DISARM ETMEZ (Adim 153'teki ayni ayrim). DONE yalnizca "gorev bitti,
// arac yerde" der.
inline void missionSequencer(bool enable, float agl, float v_h,
			     FtState ft, BtState bt, FwState fw, LandState land,
			     MissionState &state, float &phase_timer, float dt,
			     bool &req_pos_hold, bool &req_ft, bool &req_bt,
			     bool &req_fw, bool &req_land, float &z_sp_out,
			     float z_datum, float z_now,
			     float pos_x, float pos_y, float &home_x, float &home_y, float &home_yaw,
			     bool &req_home, float yaw_now, float &yaw_sp_out,
			     float &yaw_hold)
{
	if (!enable) {
		state = MissionState::IDLE;
		phase_timer = 0.f;
		req_home = false;
		req_pos_hold = false;
		req_ft = false;
		req_bt = false;
		req_fw = false;
		req_land = false;
		z_sp_out = z_now;
		yaw_sp_out = yaw_now;
		return;
	}

	phase_timer += dt;

	// Varsayilanlar: her evre yalnizca kendi bayragini kaldirir.
	//
	// pos_hold VARSAYILAN OLARAK KAPALI, VE BU BILEREK (2026-08-31 duzeltmesi).
	// Ilk yazimda her evrede aciktı; SITL'de olculdu ve GOREVIN TAMAMINI
	// bozdu: pozisyon dongusu ileri gecisle guresti, ft_state CRUISE'a HIC
	// ulasmadi, FWD/BACK/SETTLE ucu de 60 s zaman asimina dustu ve arac
	// 39 m'de 7-15 m/s ile ucup gitti. Betik de zaten gecis evrelerinde
	// pos_hold GONDERMIYOR (send(ft=True), pos_hold varsayilani False).
	// Hover tutan evreler onu ACIKCA acar.
	req_pos_hold = false;
	req_ft = false;
	req_bt = false;
	req_fw = false;
	req_land = false;
	req_home = false;
	z_sp_out = z_datum - MSN_CLIMB_ALT;
	// YAW: varsayilan olarak gorev basindaki istikamet korunur.
	yaw_sp_out = home_yaw;

	const bool timed_out = phase_timer > MSN_PHASE_TIMEOUT_S;

	switch (state) {
	case MissionState::IDLE:
		// EVI YAKALA: gorev basladigi an neredeyse orasi evdir.
		home_x = pos_x;
		home_y = pos_y;
		home_yaw = yaw_now;
		yaw_hold = yaw_now;
		yaw_sp_out = yaw_now;
		state = MissionState::CLIMB;
		phase_timer = 0.f;
		break;

	case MissionState::CLIMB:
		req_pos_hold = true;

		if (fabsf(agl - MSN_CLIMB_ALT) < MSN_CLIMB_TOL) {
			state = MissionState::HOVER;
			phase_timer = 0.f;
		}

		break;

	case MissionState::HOVER:
		req_pos_hold = true;

		if (phase_timer >= MSN_SETTLE_S) {
			state = MissionState::FWD;
			phase_timer = 0.f;
		}

		break;

	case MissionState::FWD:
		req_ft = true;

		if (ft == FtState::CRUISE) {
			state = MissionState::CRUISE;
			phase_timer = 0.f;

		} else if (timed_out) {
			state = MissionState::BACK;
			phase_timer = 0.f;
		}

		break;

	case MissionState::CRUISE:
		req_ft = true;

		if (phase_timer >= MSN_CRUISE_S) {
			state = MSN_FW_PHASE ? MissionState::FW : MissionState::BACK;
			phase_timer = 0.f;
		}

		break;

	case MissionState::FW:
		// ft BAYRAGI KALKIK KALMALI: sabit kanat giris kapisi
		// _ft_state == FtState::CRUISE istiyor (fixedWingTransition()).
		req_ft = true;
		req_fw = true;

		if (fw == FwState::ACTIVE) {
			state = MissionState::FW_CRUISE;
			phase_timer = 0.f;

		} else if (timed_out) {
			state = MissionState::BACK;
			phase_timer = 0.f;
		}

		break;

	case MissionState::FW_CRUISE:
		req_ft = true;
		req_fw = true;

		if (phase_timer >= MSN_FW_CRUISE_S) {
			state = MissionState::BACK;
			phase_timer = 0.f;
		}

		break;

	case MissionState::BACK:
		// ft DUSER, bt KALKAR (betikteki 4. adimla ayni).
		req_bt = true;

		if (bt == BtState::HANDOFF || timed_out) {
			state = MissionState::RETURN;
			phase_timer = 0.f;
		}

		break;

	case MissionState::RETURN: {
			// EVE DON (Adim 155). Olculdu: donus olmadan arac 683 m uzaga
			// gidip ORAYA iniyordu. Pozisyon dongusu hover-only ve <=3 m/s'de
			// dogrulanmis; hedefi ev yapmak yeterli, yeni bir mekanizma yok.
			req_pos_hold = true;
			req_home = true;

			const float dx = pos_x - home_x;
			const float dy = pos_y - home_y;
			const float dist = sqrtf(dx * dx + dy * dy);

			// BURNU EVE CEVIR (2026-08-31 duzeltmesi). Olculdu: bu satir
			// olmadan arac 683 m'yi GERI GERI geliyordu -- govde cercevesinde
			// ileri hiz -2.93 m/s, burun evden 163 derece sapik. Pozisyon
			// dongusu araci hedefe goturur ama burnunu CEVIRMEZ; yon komutu
			// ayri verilmeli.
			// YAKINDA DONDURME: mesafe kuculunce kerteriz gurultulenir ve
			// arac hedefin uzerinde donmeye baslar. MSN_HOME_R'nin iki kati
			// altinda yon komutu DONDURULUR.
			if (dist > 2.f * MSN_HOME_R) {
				yaw_sp_out = atan2f(home_y - pos_y, home_x - pos_x);

			} else {
				yaw_sp_out = yaw_now;
			}

			// AYRI ZAMAN SINIRI: 683 m'yi 3 m/s ile katetmek ~228 s surer,
			// yani genel 60 s'lik sinir burada anlamsizdir.
			if (dist < MSN_HOME_R || phase_timer > MSN_RETURN_TIMEOUT_S) {
				// YAW'I DONDUR (2026-08-31 duzeltmesi). Olculdu: bu satir
				// olmadan SETTLE/LAND varsayilan yaw_sp = home_yaw'a donuyor
				// ve arac INERKEN 166 derece geri doniyordu (SETTLE +70,
				// LAND +96, SETTLE'da |yaw hizi| ort 24 deg/s). Inis
				// sirasinda donmek, temas anini bilinmeyen bir yonelime
				// birakmak demektir.
				yaw_hold = yaw_now;
				state = MissionState::SETTLE;
				phase_timer = 0.f;
			}

			break;
		}

	case MissionState::SETTLE:
		req_pos_hold = true;
		yaw_sp_out = yaw_hold;

		// Yatay hiz sonmeden inise gecmek, araci yana suruklenirken
		// indirmek demektir.
		if (v_h < MSN_LAND_VH || timed_out) {
			state = MissionState::LAND;
			phase_timer = 0.f;
		}

		break;

	case MissionState::LAND:
		req_pos_hold = true;
		req_land = true;
		// YAW: RETURN CIKISINDA DONDURULAN YON TUTULUR (2026-08-31).
		//
		// UC SECENEK OLCULDU, ayni gorev, ayni inis:
		//   home_yaw'a don : SETTLE +70.0 deg, LAND +96.0 deg donus  ⛔
		//   yaw'i BIRAK    : LAND +41.3 deg surukleme               (betigin yolu)
		//   yaw'i DONDUR   : LAND  +1.4 deg                          ✅ secildi
		// Inis KALITESI ucunde de ayni (BIG_M 0, |T0-T1| 0.59 vs 0.61 N,
		// T2 min 11.76 vs 11.44 N) -- fark yalnizca yon tutmada, 30 kat.
		//
		// NEDEN BETIK "BIRAK" DIYORDU VE BURADA GECERLI DEGIL:
		// run_mission_test.py 2026-08-28'de yaw referansini inis boyunca
		// birakiyordu ve gerekcesi dogruydu -- ama coezdugu sey BAYAT
		// REFERANSTI: betigin tuttugu yon KALKIS yonuydu (yaw0) ve gorev
		// sonunda arac ondan cok sapmis oluyordu; o buyuk hatayla guresmek
		// kanat itki farkini buyutup kilitlenme uretiyordu.
		// Burada referans RETURN cikisinda O ANKI yonde yakalanir, yani hata
		// SIFIRDAN baslar. Birakmanin faydasi (hata yok) korunur, bedeli
		// (geri getirici referans yok, arac suruklenir) odenmez.
		//
		// Bu ayni zamanda LAND_YAW_FREE_ALT'in (Adim 129) neden "etkisiz"
		// olctugunu da acikliyor: betik zaten birakiyordu, modulun ayni seyi
		// tekrar yapmasinin olculebilir bir etkisi olamazdi.
		//
		// SETTLE'da da ayni yaw_hold: orada arac hala yatay hizi soduruyor.
		yaw_sp_out = yaw_hold;
		z_sp_out = z_now;   // profili landingSequence uretir

		if (land == LandState::TOUCHDOWN) {
			state = MissionState::DONE;
			phase_timer = 0.f;
		}

		break;

	case MissionState::DONE:
	default:
		req_pos_hold = true;
		req_land = true;
		yaw_sp_out = yaw_hold;
		z_sp_out = z_now;
		break;
	}
}

} // namespace tiltrotor_indi
