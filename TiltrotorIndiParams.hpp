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

// 1:1 constants from tiltrotor_Matlab files/tiltrotor_params.m and
// gain_schedule.m. These describe the fixed physical geometry of the
// gz_tiltrotor_tailplane airframe (CA_ROTOR*_P{X,Y,Z}/KM in
// 4022_gz_tiltrotor_tailplane) and are not meant to be runtime-tuned the
// way rate/attitude gains normally are in PX4 -- the MATLAB reference
// itself hardcodes them as literals (not as tunable "params"), so this
// mirrors that design rather than inventing a new INDI_* param group.

namespace tiltrotor_indi
{

// --- allocator width (step 45, 2026-08-03, item (V)) ---
// 6 rotor channels [T0 T1 T2 d0 d1 d2] + 5 aerodynamic control surfaces.
// The surfaces used to be held at zero ("out of scope for the ported
// controller"), and step 43 measured what that cost: with no aerodynamic
// attitude authority, ALL of it has to come from the rotors, so the rotors can
// never be shut down and the vehicle cannot become fully wing-borne -- at
// 20.6 m/s both WING rotors were already at 0.0 N while the TAIL rotor still
// carried 21.8 N purely to hold pitch, with the allocator 20% saturated.
static constexpr int N_ACT = 11;

// --- aerodynamic control surfaces (surface index 0..4 -> u index 6..10) ---
// Transcribed 1:1 from Tools/simulation/gz/models/tiltrotor_tailplane/model.sdf
// LiftDrag plugins, WITH the FLU->FRD conversion (x, -y, -z) applied -- skipping
// that same conversion is what made blocker B4 look like a sign contradiction
// for months (step 34).
//   servo_0/1  left/right ELEVON    cp FLU (-0.05, +-0.30, 0.05)  area 0.5    rad_to_cl -4.0
//   servo_2/3  left/right ELEVATOR  cp FLU (-0.70, +-0.15, -0.04) area 0.048  rad_to_cl -12.0
//   servo_4    RUDDER               cp FLU (-0.74, 0, 0.12)       area 0.032  rad_to_cl -6.0
// Force model: F = qbar * k * delta * e_up applied at cp, LINEAR in delta, so
// the Jacobian is delta-independent and scales only with qbar = 0.5*rho*V^2.
// e_up: servos 0-3 FLU (0,0,1) -> FRD (0,0,-1); rudder FLU (0,1,0) -> FRD (0,-1,0).
static constexpr int N_SURF = 5;
// RUDDER -0.74 -> -0.86 (step 132): the fin moved aft because the tail rotor
// disc passed through it at tilt = 0, i.e. for the whole hover phase.
static constexpr float SURF_CPX[N_SURF] = { -0.05f, -0.05f, -0.70f, -0.70f, -0.78f };
static constexpr float SURF_CPY[N_SURF] = { -0.30f, +0.30f, -0.15f, +0.15f,  0.00f };
static constexpr float SURF_CPZ[N_SURF] = { -0.05f, -0.05f, +0.04f, +0.04f, -0.10f };
static constexpr float SURF_EX[N_SURF]  = {  0.0f,   0.0f,   0.0f,   0.0f,   0.0f };
static constexpr float SURF_EY[N_SURF]  = {  0.0f,   0.0f,   0.0f,   0.0f,  -1.0f };
static constexpr float SURF_EZ[N_SURF]  = { -1.0f,  -1.0f,  -1.0f,  -1.0f,   0.0f };
// k = area * rad_to_cl (N per Pa per rad), sign included.
static constexpr float SURF_K[N_SURF] = { -2.0f, -2.0f, -0.576f, -0.576f, -0.192f };
// Deflection limits are the model.sdf JOINT limits, not a software choice.
static constexpr float SURF_MAX[N_SURF] = { 0.78f, 0.78f, 0.52f, 0.52f, 0.52f };
static constexpr float SURF_RATE_MAX = 4.0f;   // rad/s (servo slew, sets the box)
static constexpr float RHO_AIR = 1.225f;       // kg/m^3
// WU_SURF -- the surfaces' WLS penalty, and it is a DERIVATION, not a taste.
// gain_schedule.m's documented rule: the WLS prefers actuator i over j when
// Wu_i/|G_i| < Wu_j/|G_j| (penalty per unit torque). Because the surfaces' |G|
// grows with qbar, a CONSTANT WU_SURF automatically yields "expensive in hover,
// cheap in cruise" -- the handover point is the crossing of a RATIO, not a mode
// switch, which is why no state machine is needed.
// Placed at FT_CRUISE_V = 8 m/s:
//   qbar(8) = 39.2 Pa; elevon roll effectiveness 0.6*qbar = 23.5 N*m/rad
//   rotor tilt roll effectiveness ~ T*0.05 ~ 0.45 at cruise thrust (~9 N)
//   WU_TILT_CRUISE = 1.5 -> tilt penalty per unit torque = 1.5/0.45 = 3.33
//   equate: WU_SURF/23.5 = 3.33 -> 78.3 -> 80
// The per-axis crossovers that follow are PHYSICALLY right (a smaller surface
// takes over later): roll ~8 m/s, pitch (elevator) ~10 m/s, yaw (rudder, |G| 4x
// smaller) ~16 m/s.
// !! NOT to be treated as correct until measured in SITL: step 7 changed a
// weight by an order of magnitude and blew the Fx demand to -28 N.
static constexpr float WU_SURF = 80.0f;

// *** SURFACES ARE CURRENTLY DISABLED -- TRIED TWICE, REVERTED (step 45). ***
// The effectiveness model, the widened allocator and the servo output are all
// kept, in the same spirit as decelLoop(): they record a MEASURED negative
// result so the same two ideas are not retried blind. Setting this true is the
// only change needed to re-enable them.
//
// ATTEMPT 1 -- surfaces as full force+moment actuators. All three rotors pinned
// at ROTOR_TMAX (45.0/45.0/44.9 N), thrust saturation 16-24%, 1958 BIG_M, and
// the cruise tilt COLLAPSED 39.7-44.1 deg -> 13.3 deg. Cause: servo_0/1's cp is
// at x = -0.05 m, essentially AT the CG, so they are not pitch surfaces at all
// but the MAIN WING's flap -- pitch effectiveness 24.5 N*m/rad against an Fz
// effectiveness of 490 N/rad (a factor of 20). Any use of them dumped ~107 N of
// download (>2x the weight) and the rotors maxed out fighting it. Worse, it
// REMOVED the mechanism that tilts the rotors at all: step 31 measured that the
// tilt is driven by the Fz demand, and the flap now served that demand instead.
//
// ATTEMPT 2 -- declare them MOMENT-ONLY (force rows zeroed in G). This made it
// worse: the vehicle ROLLED INVERTED (max |roll| 180 deg, 66.9% of the flight
// beyond 90 deg), all five surfaces pinned at their deflection limits, 4 forward
// transition aborts, 22.5 m altitude excursion. Cause is the general one this
// project keeps paying for: zeroing a row does not remove the force, it only
// hides it from the allocator -- which then treats the surfaces as free moment
// sources, drives them to the stops, and the unmodelled force wrecks the
// vertical axis. THE MODEL HANDED TO THE ALLOCATOR MUST MATCH THE PHYSICS, OR
// THE ALLOCATOR WILL EXPLOIT THE DIFFERENCE.
//
// What a third attempt would have to address: the elevons are a LIFT device with
// a 0.05 m moment arm, so they belong on the Fz channel, not the moment channels
// -- most likely as a scheduled trim (a flap) rather than as a WLS actuator,
// leaving only servo_2/3 (elevator, arm 0.70 m) and servo_4 (rudder) in the
// allocation. That is a design change, not a weight change, and it needs its own
// measurement campaign.
static constexpr bool SURF_ENABLE = false;

// --- mass / inertia (SDF base_link) ---
static constexpr float MASS = 5.0f;             // kg
static constexpr float GRAVITY = 9.81f;          // m/s^2
static constexpr float I_XX = 0.2f;              // kg m^2
static constexpr float I_YY = 0.25f;
static constexpr float I_ZZ = 0.25f;

// --- rotor geometry (FRD, m) -- columns are rotor positions ---
// rotor 0: right wing, rotor 1: left wing, rotor 2: tail (pusher in cruise)
// TRIED AND REVERTED (2026-08-14): moving the wing rotors to x=0.30 (ahead
// of the wing's leading edge at x=0.25, vs. the original x=0.22 tucked
// under/behind it) passed the pure-MATLAB regression tests
// (run_hover_gust_test / run_transition_test) but showed a real SITL
// regression: sitl-lockup-check at 28s showed vx/vy/avg_tilt still growing
// without settling (vx 12.3->14.1 m/s, vy 2.7->6.7 m/s, tilt 0.36->0.60 rad
// over 15s), unlike the original position's clean converged trim. Exactly
// the failure mode safe-control-change warns about: MATLAB clean does not
// imply SITL-safe. Real-hardware prop/wing clearance for the original
// placement is a separate mechanical design question, not resolved here --
// revisit with a proper CAD/geometry answer, not a guessed offset, and
// re-run the full safe-control-change procedure before trying again.
static constexpr float ROTOR_PX[3] = { 0.27f,  0.27f, -0.55f };
// NOTE (2026-07-26 investigation, NOT applied): model.sdf comments read
// "Right wing rotor" for motor_0 @ Y=-0.25 and "Left wing rotor" for
// motor_1 @ Y=+0.35 (SDF, FLU) -- opposite sign from {0.35,-0.35,0} below. A sign-flip
// fix was tried (both here and in tiltrotor_params.m/sf_wls_alloc.m, km
// left unchanged) and REVERTED: it caused a ~100-1000x RMS regression in
// the pure-MATLAB reference test (run_hover_gust_test), where plant and
// controller share this exact same array and were otherwise self-consistent
// before AND after the change -- meaning the naive sign flip is not the
// correct fix (likely the real km-sign-to-turningDirection mapping needs
// independent verification before touching this again, not just the Y
// sign in isolation). See sitl/WLS_LOCKUP_INVESTIGATION_REPORT.md and
// RUNBOOK.md "Aday cozum 4" before attempting this again.
static constexpr float ROTOR_PY[3] = { 0.35f, -0.35f,  0.00f };
// UPDATED (2026-08-16, friend's authoritative fix): wing rotors moved from
// below the wing (Z=+0.06 FRD) to above it (Z=-0.11 FRD), mirroring the
// pylon/motor/rotor stack about the wing while keeping the existing 0.0075 m
// prop-above-motor clearance (same rule the tail rotor already used). Source
// value measured off x8_wing.dae by vertical ray cast (model.sdf, 2026-08-02):
// SDF rotor pose Z=+0.11 (FLU) -> ROTOR_PZ=-0.11 (FRD, sign flip). Synced with
// tiltrotor_params.m / sf_wls_alloc.m and model.sdf; re-validate with
// sitl-lockup-check before trusting this in flight.
// KUYRUK -0.07 -> -0.16 (2026-08-30, step 132): the 0.10 m rotor disc swept
// THROUGH the tail boom, the tailplane and the vertical fin. Geometric fix,
// not a tuning change; ROTOR_PX is untouched so hover pitch authority
// (tau_y = r_x*T) is unchanged. Measured by check_model_clearance.py.
static constexpr float ROTOR_PZ[3] = { -0.11f, -0.11f, -0.07f };
// ROTOR_KM=0.06 (was 0.05, 2026-07-26): inherited from the CA_ROTORi_KM
// airframe param (4022_gz_tiltrotor_tailplane), which is a mistranscription
// of the actual Gazebo model -- Tools/simulation/gz/models/tiltrotor_tailplane/
// model.sdf's gz-sim-multicopter-motor-model-system plugin sets
// <momentConstant>0.06</momentConstant> for all three rotors (confirmed:
// gz-sim source computes dragTorque_z = -turningDirection*thrust*momentConstant,
// i.e. momentConstant IS this km, same Nm/N convention, directly comparable).
// A ~20% low km here means hoverTrim()'s analytic yaw-nulling correction
// (single-axis delta1 trim) under-corrects the real reaction-torque
// imbalance at arm, leaving a persistent uncorrected yaw torque that -- via
// delta1's shared roll/yaw effectiveness -- leaks into the roll axis too.
// Prime suspect for the persistent/growing nu_des(0) driving the SITL
// actuator lockup (see sitl/RUNBOOK.md "Aday cozum 3"); tiltrotor_params.m
// updated to match for fidelity (MATLAB plant+controller were already
// self-consistent at the old value, so this was PX4-only divergence).
//
// SIGN FIX (2026-07-27, step 12) -- was { +0.06, -0.06, +0.06 }:
// the note above transcribes the gz formula correctly but only ever matched
// the (+,-,+) PATTERN against the ccw/cw/ccw turningDirections; the pattern's
// OVERALL sign in FRD was never compared. Full chain:
//   gz applies dragTorque = (0, 0, -turningDirection*T*km) in the rotor
//   link's own frame (SDF rpy = 0 0 0, so aligned with base_link; gz uses
//   FLU: x-fwd, y-LEFT, z-UP). Rotor 0 is ccw (turningDirection=+1, T>0):
//   tau_z(FLU) = -km*T < 0. FLU->FRD flips z, so tau_z(FRD) = +km*T > 0
//   (nose right) -- which is also the physically correct reaction for a
//   rotor spinning ccw seen from above.
//   This model instead had m_i = km_i*T_i*dir_i with dir = (0,0,-1), giving
//   m_z = -km*T < 0 (nose left) -- inverted, for all three rotors.
// Confirmed quantitatively in SITL before the fix: with the old signs
// hoverTrim()'s yaw-nulling tilt was applied in the direction that ADDS to
// the real imbalance instead of cancelling it, predicting an open-loop
// +6.2 rad/s^2 yaw acceleration at arm; two independent runs measured peak
// +6.45 and +6.56 rad/s^2, while the old model predicts ~0. The vehicle
// spun up to 5-6 rad/s within 4 s of arming even with yaw_sp set to the
// current heading (i.e. zero yaw error), so this was never a setpoint or
// tracking artifact.
// Same class of bug as step 11's thrust mapping: pure MATLAB cannot see it,
// because plant (tiltrotor_plant_deriv.m) and controller
// (effectiveness_matrix.m) share the same p.rotor.km and are wrong together.
// See sitl/WLS_LOCKUP_INVESTIGATION_REPORT.md step 12.
static constexpr float ROTOR_KM[3] = { -0.06f, 0.06f, -0.06f };

// --- rotor thrust limits (SDF MulticopterMotorModel) ---
static constexpr float ROTOR_KF = 2.0e-5f;       // N / (rad/s)^2
static constexpr float ROTOR_WMAX = 1500.0f;     // rad/s
static constexpr float ROTOR_TMAX = ROTOR_KF * ROTOR_WMAX * ROTOR_WMAX; // ~45 N
static constexpr float ROTOR_TMIN = 0.0f;

// --- YERE YAKIN KANAT ITKI FARKI SINIRI (2026-08-29) ---
// MATLAB karsiligi: tiltrotor_params.m p.ctrl.land_diff_{max,alt,enable} ve
// indi_attitude_controller.m'deki kirpma. TAM GEREKCE ORADA; burada ozet.
//
// NE COZUYOR: temastan sonra arac hafif egik oturur (olculen kalici roll
// hatasi 0.18 deg). Zemin araci tuttugu icin omega_dot_meas = 0 kalir ve
// INDI'nin artimi sonmez -- u_cmd = _u_actual + du HER TIK ayni yonde birikir.
// Olculen sonuc (ULog 09_27_25): T0 34 N'ye tirmanir, T1 sifir rayina cakilir,
// tek calisan EGIK rotor karsiligi olmayan bir yaw momenti uretir, 6793 deg
// yaw kacisi ve arac hic inemez.
//
// NEDEN BU BICIM: ORTALAMAYI korur, yalnizca FARKI sinirlar. Dikey kanal
// ortalamadan geldigi icin irtifa dongusu etkilenmez -- ki olcum onun ZATEN
// dogru davrandigini gosterdi (Fz_sp talebi -25 N'e inmisti; izlenmeyen sey
// tahsisatti, talep degil). Tutum kontrolu de CANLI kalir; ilk tasarim
// "yerdeyken tutum artimini dondur" idi ve TEHLIKELI cikti, cunku yer
// etkisinde 1.29 m'de ASILI arac ile yerde OTURAN arac irtifa ve acisal hiz
// bakimindan ayirt edilemiyor (ikisinde de |w| ~= 0.001 olculdu).
//
// DEGER: saglikli iniste |T0-T1| ort 0.40 N / max 0.95 N (ULog 10_32_31);
// arizada 45.00 N (tam olcek). 10 N saglikli calismanin 10 katindan fazla pay
// birakir. 0.25 m kolda ~2.5 Nm roll torku.
//
// LAND_DIFF_ALT'in USTUNDE MEKANIZMA TAMAMEN ETKISIZDIR. 2.0 m, olculen butun
// yer olaylarinin (oturma 0.03-0.6 m, yer etkisinde asilma 1.17-1.29 m)
// ustunde ve her ucus rejiminin cok altinda -- sabit kanat FW_MIN_ALT = 30 m
// istedigi icin o yol da yapisal olarak disaridadir.
//
// ADIM 116 -- ESIGIN KARSILASTIRILDIGI SEY: kalkis datumuna gore AGL, HAM -z
// DEGIL. -z, kestirimci orijininden yuksekliktir ve 23 SITL kosumunda olculen
// datum ofseti -0.67 .. +1.77 m, yani bu esikle AYNI MERTEBEDE. 116'ya kadar
// kapi -z ile aciliyordu: en kotu kosumda (ULog 11_26_27) arac 0.64 m'de yere
// degdi, sinyal 2.41 m dedi, mekanizma hic armanmadi ve fark tam olcege (45 N)
// gitti. Adim 113 o kosumu "2.36 m'de HAVADA kilitlenme" diye kaydetmisti --
// oyle bir arıza modu yoktu. Kaynak: MulticopterIndiTiltrotor.cpp
// captureGroundDatum(). Esigi buyutmek yanlis yoldur: hatayi olcmez, orter.
static constexpr bool  LAND_DIFF_ENABLE = true;
// DIKEY ITKI TAVANI (Adim 145). Tahsisat momentleri kuvvetten 200 KAT agir
// tartiyor (WS_ROLL/WS_PITCH = 200), yani yerde olusan SAHTE bir moment
// talebini karsilamak icin kuvvet komutunu cignemekten cekinmez.
// OLCULEN ARIZA (ULog 11_28_57): arac 0.29 m'de yere degdi, temas govdeyi
// hafifce pitch'ledi, nu_des[1] +0.13'ten +1.22'ye buyudu, tahsisat kuyrugu
// bosaltip (m2 -> 0.000) onu itti (m0 0.63 -> 0.77) ve dikey itki toplami
// |fz_sp|'nin 3.18 KATINA cikti -- arac 1.18 m'ye GERI KALKTI.
// Bu, mevcut LAND_DIFF_MAX kismasinin GORMEDIGI bir eksen: o yalniz kanat
// FARKINI (roll) kisar; ariza ise on-vs-kuyruk (pitch) kanalinda.
// NEDEN PITCH FARKINI DOGRUDAN KISMIYORUZ: olculdu, ISE YARAMAZ. Kapi altinda
// (AGL<2 m) pitch sapmasi SAGLIKLI kosumlarda da buyuk (142 sakin: ort 12.6 N,
// orneklerin %69'u 5 N ustunde -- ve o kosu temiz indi), cunku rotorlar
// kanadin AERODINAMIK pitch momentini de dengeliyor. Boyle bir kisit havada
// yanlis atesLer ve pitch otoritesini keser.
// SECILEN OLCUT kontak tespitine HIC bagli degil: "kapi acikken tahsisat,
// komut edilen dikey kuvvetin LAND_TZ_MAX katindan fazlasini uretemez".
// Esigin 2.0 secilme gerekcesi, esigi asan ornek yuzdesi:
//   143a sakin %0.0 | 143b sakin %0.0 | 143b ruzgar %0.0 | 143b ruzgar#3 %0.0
//   142 sakin %1.1  | SICRAYAN %16.7
// Yani saglikli inislerin hicbirine dokunmuyor. Uc rotor BIRLIKTE olceklenir,
// boylece moment ORANLARI korunur; kesilen sey net kaldirmadir.
static constexpr float LAND_TZ_MAX = 2.0f;      // x|fz_sp|, kapi altinda dikey itki tavani
static constexpr float LAND_DIFF_MAX = 10.0f;   // N, |T0-T1| ust siniri
static constexpr float LAND_DIFF_ALT = 2.0f;    // m AGL, ustunde etkisiz
// TEMAS ESIGI: bu roll acisinin USTUNDE sinir 0'a iner, yani kanat farki
// tamamen silinir. MATLAB karsiligi p.ctrl.land_contact_roll.
// 8 deg, olculen saglikli inisin (|roll| < 1 deg) sekiz kati ustunde ve
// olculen temas olayinin (-19.78 deg) cok altinda. Serbest hover'da tutum
// dongusu bu aciya asla izin vermez -- ve bu esik, irtifa ile acisal hizin
// ayiramadigi "yer etkisinde ASILI" ile "yere OTURMUS" durumunu da ayirir.
static constexpr float LAND_CONTACT_ROLL = 0.13962634f;  // rad (8 deg)
// IKINCI TEMAS OLCUTU (Adim 118) -- MATLAB karsiliklari p.ctrl.land_contact_*.
// Roll esigi DUZ inisi hic yakalamiyor: olculen uc kosumda da fark 10 N'de
// doydu ve orada kaldi, ama roll 0.18-0.54 derecede donmustu, yani ustteki dal
// atesLenmedi. 8 derecelik esigin gerekcesi olculmus bir TAKLA idi (-19.78 deg).
//
// OLCUT: "buyuk diferansiyel KOMUT, ama acisal ivme YOK". Serbest ucusta bu
// imkansizdir -- 6 N, 0.25 m kolda ~1.5 Nm eder ve govdeyi ivmelendirmek
// ZORUNDADIR. Yerde zemin momenti karsilar. Adim 109'un ELENEN olcutu bu DEGIL:
// o yalnizca "|w| ~ 0" istiyordu ve yer etkisinde asili sakin araci da temas
// sanmisti; bu olcut ayrica BUYUK BIR KOMUT sart kosuyor.
//
// AYRIM 26 TAM GOREV LOGUNDA OLCULDU (hover, gecis, seyir, sabit kanat, inis):
//   AGL > 2.0 m'de olcutun kesintisiz surdugu en uzun sure : 0.01 s (tek ornek)
//   AGL < 1.5 m'de                                         : 3.28 s'ye kadar
//   saglikli inislerde (temiz 6 kosum)                     : 0.00 s -- hic
// DWELL, olculen en kotu havada-yanlis-pozitifin 20 KATI.
//
// MANDAL SART: fark sifirlaninca olcut kendi kendini bozar; mandalsiz mekanizma
// acilip kapanarak salinir. Kapi kapaninca (AGL > LAND_DIFF_ALT) temizlenir.
static constexpr float LAND_CONTACT_DIFF = 6.0f;    // N
static constexpr float LAND_CONTACT_ACC = 0.05f;    // rad/s^2
static constexpr float LAND_CONTACT_DWELL = 0.20f;  // s
// ROTOR_WMIN (2026-07-27, step 11): SIM_GZ_EC_MIN1..3 in the
// 4023_gz_tiltrotor_indi airframe. GZMixingInterfaceESC publishes the mixed
// output straight into the Actuators velocity field, and MixingOutput scales
// a normalized command linearly onto [SIM_GZ_EC_MIN, SIM_GZ_EC_MAX], so the
// rotor angular velocity Gazebo actually applies is
//     w = ROTOR_WMIN + control * (ROTOR_WMAX - ROTOR_WMIN)
// and the thrust it produces is ROTOR_KF * w^2 -- quadratic in `control`.
// Needed to invert that mapping correctly, see thrustToNormalized().
static constexpr float ROTOR_WMIN = 10.0f;       // rad/s
static constexpr float ROTOR_TAU_UP = 0.0125f;   // s
static constexpr float ROTOR_TAU_DOWN = 0.0250f; // s

// --- tilt servo (CA_SV_TL*_MINA/MAXA = 0..90 deg) ---
static constexpr float TILT_MIN = 0.0f;          // rad, hover
static constexpr float TILT_MAX = 1.57079632679f; // rad, pi/2, cruise
// KUYRUK ROTORU ICIN AYRI TAVAN (2026-08-30, step 133). PHYSICAL, not tuning:
// the 0.10 m disc at 90 deg reaches z = motor_z - 0.10 = -0.055 and sweeps
// THROUGH the tail boom (measured: check_model_clearance.py). Raising the motor
// was tried and REVERTED -- it cost the back-transition dearly (three rotors
// pinned at 45 N, BIG_M 0 -> 3843) because tau_y = r_z*Fx - r_x*Fz and the
// back-transition lives in exactly that tilted regime.
// 20 deg IS MEASURED: the tail tilt never exceeded 2.5 deg in a full mission
// (ULog 2026-08-30/05_47_06), so the cap is 8x observed use. Synced with
// model.sdf motor_2_joint <upper> and tiltrotor_params.m p.tilt.max_tail.
static constexpr float TAIL_TILT_MAX = 0.349066f;   // rad (20 deg)
static constexpr float TILT_TAU = 0.15f;         // s
// TILT_RATE_MAX=2.0f (was 3.0f, 2026-07-26, testing -- see sitl/RUNBOOK.md
// "Adim 9" / WLS_LOCKUP_INVESTIGATION_REPORT.md Adim 8): model.sdf's real
// tilt servo (gz-sim-joint-position-controller-system, motor_i_joint) is
// NOT the simple TILT_TAU first-order lag this shadow model (_u_actual in
// MulticopterIndiTiltrotor.cpp) assumes -- it's a torque-limited PID
// (p_gain=100, d_gain=0, cmd_max=2 Nm, err_max=0.2 rad -> saturates past
// ~1.1 deg of error) fighting joint friction=1.0 plus velocity-dependent
// gyroscopic reaction torque from the spinning prop (~L x omega_tilt,
// grows with thrust/RPM). Live gz-model-pose comparison against the
// shadow model measured a real, sign-flipping 2.6-3.7 deg tracking error
// on the tail rotor's tilt (delta2) specifically -- confirmed real, but
// NOT purely a "too slow" bias (oscillates both directions), so this is
// an approximate, physically-motivated (not exactly derived) reduction in
// the ASSUMED max achievable rate, meant to make the WLS box constraint
// less likely to over-commit to large single-tick tilt corrections the
// real torque-limited servo can't reliably deliver under load. This is a
// SITL-specific realism adjustment -- tiltrotor_params.m/sf_wls_alloc.m
// deliberately NOT touched, since MATLAB's plant and this shadow model are
// definitionally identical there (no plant-vs-shadow divergence exists to
// test against), unlike PX4-vs-real-Gazebo-servo.
//
// TRIED AT 3.0f AND REVERTED (2026-07-27, step 14 = report item (L)).
// Rationale for trying: step 12g measured that a per-step cap of
// TILT_RATE_MAX*dt directly bounds the achievable yaw torque increment
// (ddelta pinned at +-0.005 rad every sample), and differential wing tilt is
// yaw's only real actuator -- so 2.0 looked like it was needlessly throttling
// the one axis that had just been fixed. Initial convergence did improve:
// yaw settled in ~6 s instead of ~21 s.
// Why it was reverted: the +30 deg yaw STEP response became violently
// under-damped and direction-asymmetric. Reproduced twice: yaw overshot to
// 65 deg, then wrapped a full turn (peak rate 1.69-2.14 rad/s, i.e. 4x the
// 0.5 rad/s outer-loop rate setpoint limit) before recovering. The -30 deg
// step in the same run stayed perfectly clean (steady -0.035 rad/s, no
// overshoot at all). At 2.0f the same +30 deg step tracks in 8 s with only
// ~13% overshoot.
// What this means: step 9's ORIGINAL rationale (the comment above) is
// vindicated by measurement, not weakened -- with a higher assumed slew rate
// the WLS commits to single-tick tilt corrections the real torque-limited
// Gazebo servo cannot deliver, the shadow model diverges from reality, and
// the correction lands late and too large. The direction asymmetry is a new,
// separate finding (report item (P)); do not retry 3.0f before it is
// explained.
//
// SCOPE NARROWED (2026-07-28, step 21): TILT_RATE_MAX is now ONLY the shadow
// model's physical servo slew limit (MulticopterIndiTiltrotor.cpp:420). The
// WLS allocator box no longer uses it -- see TILT_SLEW_BOX_RATE below. The two
// were the same constant, but they are different things: one is what the servo
// can physically do, the other is how much the allocator is allowed to ask for
// in one tick. Splitting them is what makes either of them tunable.
static constexpr float TILT_RATE_MAX = 2.0f;     // rad/s, PHYSICAL servo limit

// Coulomb-friction deadband of the real tilt servo (2026-07-28, step 24).
// model.sdf: JointPositionController p_gain=100, cmd_max=2 Nm, driving a joint
// with <friction>1.0</friction>. The joint cannot move until the P torque breaks
// friction, i.e. |err| >= friction/p_gain = 1.0/100 = 0.01 rad = 0.573 deg.
// Applied to the shadow model in MulticopterIndiTiltrotor.cpp (see the long note
// there for the offline validation: 3.5x / 8.0x / 139x lower shadow-vs-real RMS
// error on delta0/1/2, and why the full second-order model is NOT better).
// HARDWARE: structure transfers, value must be re-measured per servo.
static constexpr float TILT_STICTION_BAND = 0.01f; // rad, = friction / p_gain

// --- WLS allocator rate-limit box ---
//
// The box is deliberately sized from a FIXED nominal period rather than Run()'s
// measured dt: dt jitters with scheduling load, and feeding it in directly lets
// a single short tick starve an actuator's allowance and latch it into WLS
// saturation. That decision stands.
//
// What it MISSED (measured 2026-07-28, step 21): the nominal period must match
// the loop's ACTUAL period, or the box silently de-rates the actuator. This
// module is driven by the vehicle_angular_velocity callback and runs at
// ~250 Hz (measured median tick 4.00 ms) -- but the old constant was
// TS_CTRL = 1/400 = 2.5 ms. Tilt was therefore allowed
// TILT_RATE_MAX*(1/400) = 0.005 rad per tick while a tick lasts 4 ms, i.e. an
// EFFECTIVE slew ceiling of 1.25 rad/s -- 62% of the nominal 2.0. Measured
// consequences: tilt channels sat at the box 99.4-99.9% of the time and the
// allocator delivered only 20.6% of the demanded yaw torque. Wing tilt is
// yaw's only real actuator, so this throttled precisely the weak axis.
//
// It also retro-explains step 14: nominal 3.0 was really 1.875 rad/s effective
// (and diverged), while nominal 2.0 was really 1.25 (and worked). Sizing the
// box from the measured dt at nominal 2.0 would jump straight to 2.0 effective,
// i.e. PAST the 1.875 that already diverged -- so the naive fix is the wrong
// move. Instead the period and the rate are now stated separately and honestly:
//
//   box = TILT_SLEW_BOX_RATE * TS_BOX = 1.25 * (1/250) = 0.005 rad/tick
//
// which reproduces the previous box to within 1 float ULP (0.005 vs
// 0.0050000004 -- 4.7e-10 rad, i.e. 2.7e-8 deg), so the change is neutral in
// behaviour while the constant now means what it says and can be tuned in real
// rad/s. Verified in SITL after the change: |ddelta| p99 is still exactly
// 0.00500 rad, thrust sat_flag still 0.0%, no actuator pinning, |vz| <= 0.78 m/s.
//
// TS_BOX also feeds the THRUST box, where it is NOT neutral -- see the comment
// at that site in MulticopterIndiTiltrotor.cpp (the old upper bound of 22.5 N
// per tick was a live-but-never-reached constraint; it becomes 36 N).
//
// STILL OPEN: this only makes the constant honest; it does not by itself change
// behaviour. The low-speed yaw oscillation (report item (Q)) is unchanged. The
// point of the split is that TILT_SLEW_BOX_RATE can now be swept in real rad/s
// to find where the oscillation stops -- 1.25 works, 1.875 (old nominal 3.0)
// diverged, so the interesting range is between them.
// SWEPT AND RAISED 1.25 -> 1.75 (2026-07-28, step 23). Two flights, sweep order
// reversed between them, same +30 deg yaw step excitation at each value.
// Discriminator = yaw-rate RMS over the last 5 s of the step (settled or not):
//
//   box rate | run A  | run B  | speeds tested (A/B) | verdict
//   ---------+--------+--------+---------------------+-------------
//     1.25   | 0.583  | 0.466  |  2.20 / 0.86 m/s    | OSCILLATES
//     1.50   | 0.391  | 0.005  |  2.02 / 2.00 m/s    | marginal
//     1.75   | 0.0037 | 0.0051 |  1.35 / 2.76 m/s    | quiet
//     2.00   | 0.0056 | 0.0055 |  0.81 / 3.14 m/s    | quiet
//
// 1.25 oscillates at BOTH 0.86 and 2.20 m/s while 1.75/2.00 stay quiet from 0.81
// to 3.14 m/s, so airspeed is eliminated as the discriminator -- the box rate is.
// This is report item (Q): the low-speed yaw oscillation was the allocator being
// starved of tilt slew, not a damping deficit in the control law.
//
// RAISED AGAIN 1.75 -> 3.00 (2026-07-28, step 26), after step 26 showed that the
// cap used when picking 1.75 was the wrong reference. TILT_RATE_MAX clamps the
// SHADOW model's ddelta, and that clamp NEVER BINDS: measured 0.000% of samples,
// 43x of headroom, because ddelta = du/TILT_TAU <= box/TILT_TAU. So bounding the
// box by TILT_RATE_MAX was meaningless -- they do not act on the same quantity.
//
// What actually governs tilt authority:
//     effective slew = TILT_SLEW_BOX_RATE * TS_BOX / TILT_TAU
// The shadow advances only dt/TILT_TAU = 2.7% of du per tick, so the nominal box
// rate is de-rated 38x. At 1.75 this gave 0.047 rad/s; MATLAB's equivalent is
// 3.0*(1/400)/0.15 = 0.050 rad/s -- i.e. step 23's sweep had merely brought SITL
// up to MATLAB's effective rate, which is why the oscillation stopped right there.
//
// Extended sweep, TWO flights with the order reversed, same +30 deg yaw step at
// each value (overshoot / settling to +-2 deg):
//
//   box  | effective | run C        | run D        | mean overshoot
//   -----+-----------+--------------+--------------+---------------
//   1.75 | 0.047     | 83.5% 10.90s | 81.4%  5.28s | 82.5%
//   2.50 | 0.067     | 37.2%  4.71s | 38.7%  3.50s | 38.0%
//   3.00 | 0.080     | 20.7%  3.30s | 28.9%  4.37s | 24.8%   <-- MATLAB parity
//   3.50 | 0.093     | 18.2%  3.04s | 17.2%  3.07s | 17.7%
//
// MATLAB reference: 24.1% overshoot, 3.69 s settling. 3.00 lands on it. Speeds
// tested were 0.77-2.65 m/s and the BEST results came at the LOWEST speeds, so
// this is not an airspeed artefact.
//
// DEPLOYED 1.75 -> 3.00 (2026-07-29, step 27). 3.00 was chosen over 3.50 because
// it matches the validated MATLAB reference on overshoot, the marginal gain above
// it is shrinking (38.0 -> 24.8 -> 17.7), and it leaves headroom in the slewbox
// test range. Step 9/14's worry -- that the WLS would commit to tilt corrections
// the real torque-limited servo cannot deliver -- does not apply: at box 3.0 one
// tick asks for 0.69 deg and the real servo (59.4 rad/s^2) covers that in ~28 ms,
// while the shadow advances only 2.7% of it.
//
// CAVEAT on the "effective slew parity" reasoning above: it does NOT survive the
// overshoot data. Box 1.75 already gives 0.047 rad/s, i.e. parity with MATLAB's
// 0.050 -- yet its overshoot is 82.5% against MATLAB's 24.1%. Parity on that one
// quantity is therefore necessary-but-not-sufficient; what actually lands on the
// MATLAB overshoot is box 3.00 = 0.080 rad/s effective (0.012 rad/tick vs MATLAB's
// 0.0075). So 3.00 rests on the measured sweep, not on the parity argument.
//
// Deployment gate (mandatory, per the safe-control-change skill): raising the
// allocator's per-tick authority is exactly the kind of change that could re-open
// the actuator-lockup failure mode, so `sitl-lockup-check` was run on the rebuilt
// binary before this value was left in place. TWO heading-hold flights, both PASS:
//
//   metric (window arm+3..arm+32)   | flight 3      | flight 4      | criterion
//   --------------------------------+---------------+---------------+-----------
//   yaw error band                  | -7.29..+5.85  | -10.29..+10.32| +-30 deg
//   yaw-rate RMS, last 8 s          | 0.0028        | 0.0014        | no sust. osc
//   roll/pitch rate RMS             | 0.0010/0.0023 | 0.0014/0.0019 | --
//   max |vz|                        | 1.743         | 1.637         | <= 2.0 m/s
//   altitude hold RMS               | 0.072         | 0.052         | --
//   thrust sat_flag / BIG_M pinning | 0.0% / 0       | 0.0% / 0      | none
//   horizontal speed (context)      | 1.70-7.08     | 1.91-7.03 m/s | --
//
// Box value verified live from the ulog as |du(tilt)| p99.5 / TS_BOX = 3.000 rad/s
// in both flights, so the built binary really is running this constant.
// Full write-up incl. the yaw-metric correction: sitl/WLS_LOCKUP_INVESTIGATION_REPORT.md
// step 27. The in-flight `slewbox` hook still sweeps this without a rebuild.
static constexpr float TS_BOX = 1.0f / 250.0f;      // s, matches measured loop rate
static constexpr float TILT_SLEW_BOX_RATE = 3.00f;  // rad/s, allocator box only

// --- LESO ---
static constexpr float LESO_WO = 15.0f;          // rad/s
static constexpr float LESO_BETA1 = 2.0f * LESO_WO;
static constexpr float LESO_BETA2 = LESO_WO * LESO_WO;
static constexpr float LESO_TS = 1.0f / 200.0f;  // s, decimated update period

// --- altitude outer loop (altitude_loop.m) ---
static constexpr float ALT_TS = 1.0f / 50.0f;    // s, decimated update period
static constexpr float ALT_VZ_MAX = 2.0f;        // m/s
static constexpr float ALT_KP_Z = 0.6f;          // 1/s
static constexpr float ALT_KP_VZ = 4.0f;         // (m/s^2)/(m/s)
static constexpr float ALT_KI_VZ = 1.5f;         // (m/s^2)/(m/s * s)
static constexpr float ALT_INT_MAX = 3.0f;       // m/s^2
// ADDED (2026-08-16, step 57): altitudeLoopVz's PROPORTIONAL term
// (ALT_KP_VZ * err_vz) was UNBOUNDED -- err_vz = vz_sp - vz, and while vz_sp
// is clamped to +-ALT_VZ_MAX, the MEASURED vz is not. When the vertical
// channel loses authority (wing rotors saturated/near-zero effectiveness
// near 90 deg tilt -- the "duvar" failure class, steps 43-56), vz diverges,
// err_vz explodes, and so does the commanded Fz_sp, driving WLS to chase an
// unachievable target -- observed in a repeat of the step-56 ft_enable test:
// nu_des(Fz) grew -45 -> -73 -> -90 -> -139 -> -237 N tick over tick while
// FT_ALLOW_ABORT=false kept the transition running through an active
// "altitude band" warning, ending in a violent tumble (angular rates
// >20 rad/s). This is a hard physical clamp, not a gain/weight retune, so it
// is intentionally NOT mirrored into altitude_loop.m (MATLAB's plant does
// not reproduce this SITL-specific collapse -- see report step 56/57): -110N
// covers a hover (-49N) plus a large margin for genuine climb demand without
// ever exceeding what 3 rotors could plausibly deliver; +20N is generous for
// descent (Fz_sp=0 is already free-fall-equivalent in this convention).
static constexpr float ALT_FZ_MIN = -110.0f;     // N, most lift demand allowed
static constexpr float ALT_FZ_MAX_CLAMP = 20.0f; // N, most "push down" demand allowed

// --- horizontal position outer loop (position_loop.m, 2026-07-29 step 28) ---
// Report item (N): the tilt range is one-sided ([0, pi/2]) so hover Fx >= 0 and
// the yaw trim leaves a persistent ~3 N forward thrust. With no horizontal loop
// the vehicle drifted 235 m in 25 s -- so EVERY "hover" validation up to step 27
// was really a ~10 m/s cruise test (step 16), and true station keeping, which is
// the WORST case for this weak yaw axis, was never actually flown.
//
// This loop deliberately produces ATTITUDE setpoints, not an Fx setpoint: the
// tiltrotor-native channel (tilt the rotors for body-x force) can only push
// FORWARD because of that same one-sided range, so it cannot brake. Tilting the
// airframe works in both directions and transfers directly to hardware. Step
// 15's manual "pitch_sp = +0.061 rad" workaround is exactly this, done by hand.
//
// Gains mirror altitude_loop.m's structure (P on position -> PI on velocity)
// and share its ALT_TS = 1/50 s decimation. Validated in MATLAB first
// (run_station_keeping_test.m): drift 137.15 m -> 0.34 m over 40 s.
// TRIED AND REVERTED (2026-08-25, Adim 91): halving POS_KP_V/POS_KI_V to
// 1.00/0.20 was hypothesized to restore stability margin against the pos_hold
// + simultaneous-climb pitch limit-cycle (Adim 90: 7.3-10.9 deg, ~2-2.5s
// period, SITL-only -- MATLAB clean at nominal gains). run_poshold_climb_gain_sweep.m
// DID measure a thin MATLAB-side margin (unstable at 2x nominal), which
// motivated trying half. SITL result at half gain: NO IMPROVEMENT -- the
// vehicle instead settled into a SUSTAINED, near-constant-rate full-360-deg
// yaw rotation (r steady +0.3 to +0.6 rad/s for 40+ s, never damping) with
// WORSE position drift (up to ~4.4 m vs MATLAB's sub-meter). This DISPROVES
// the Kp_v/Ki_v margin hypothesis as the (primary) root cause -- reverted to
// the original, MATLAB-validated values. See WLS_LOCKUP_INVESTIGATION_REPORT.md
// Adim 92.
// ACIK KONU (2026-08-31, Adim 143): AYNI limit cevrimi simdi ROLL ekseninde.
// Adim 90 bunu PITCH'te bulmustu (7.3-10.9 deg, ~2-2.5 s); tiltjerk (Adim 95/96)
// pitch'i kapatti. Ruzgarli (0 6 0) pos_hold + esZAMANLI TIRMANIS'ta roll simdi
// 15.3-15.8 deg p2p (iki tekrar, sacilim 0.5 deg -- gurultu DEGIL). Eski
// geometride 8.9 deg idi; ROTOR_PX 0.22 -> 0.27 ile buyudu.
//   OLCULDU: omega_sp ve geri hesaplanan roll_sp AYNI frekansta (0.459 Hz) ve
//   roll ile ayni genlikte (oran 0.97). Yani ic INDI/rate dongusu SAGLAM --
//   arac salinan bir KOMUTU sadakatle izliyor. Kaynak bu POZISYON dongusu.
//   Saf tutum regulasyonu (10 deg'den donus, bozucusuz) iyi sonumlu.
//   DENENDI, ISE YARAMADI (Adim 143): tiltjerk yeniden ayari. 0.30 -> tirmanis
//   roll 10.6 deg'e duser AMA hover |roll|max 19.6 deg'e cikar ve GOREV KALIR
//   (BIG_M 113). 0.70 -> 22.6 deg, daha kotu. 0.45 yeni geometride de optimum.
//   Kp_v/Ki_v yariya indirme zaten Adim 91-92'de curutuldu (asagiya bakin) --
//   TEKRARLAMAYIN.
// Kalan hipotez: Adim 94'un tilt-reaksiyon bulgusunun YANAL karsiligi, ya da
// aktuator gecikmesinin (Adim 93b) pozisyon dongusu bant genisligiyle
// etkilesimi. Pitch icin gereken derinlikte bir arastirma ister; knob cevirmek
// degil. Gorevin 6 olcutunun 6'si 0.45'te GECIYOR.
static constexpr float POS_V_MAX = 3.0f;         // m/s, horizontal speed limit
static constexpr float POS_KP_P = 0.80f;         // 1/s, position -> velocity
static constexpr float POS_KP_V = 2.00f;         // (m/s^2)/(m/s)
static constexpr float POS_KI_V = 0.40f;         // (m/s^2)/(m/s * s)
static constexpr float POS_INT_MAX = 2.0f;       // m/s^2, anti-windup clamp
static constexpr float POS_A_MAX = 3.0f;         // m/s^2, horizontal accel limit
static constexpr float POS_TILT_MAX = 0.2618f;   // rad (15 deg), att_sp saturation

// Engagement gate -- THIS LOOP IS HOVER-ONLY (learned the hard way 2026-07-29,
// step 29: engaging it at 14.5 m/s crashed the vehicle in SITL).
//
// positionLoop() uses the flat multicopter relation theta_sp = -atan2(a_x, g),
// i.e. it assumes the ONLY way to get horizontal force is to tilt the thrust
// vector. That holds for a multicopter in hover. THIS AIRFRAME HAS A WING:
// above roughly 5-6 m/s the wing dominates and a nose-up command becomes an
// ENERGY/CLIMB command, not a deceleration. Measured when engaged at 14.5 m/s:
// pitch saturated at POS_TILT_MAX for 94% of samples, speed fell 14.5 -> 7.6
// then settled back to a stuck 9.6 m/s while the vehicle climbed steadily at
// ~1.1 m/s for 35 s (z -9.9 -> -54.0 m). The altitude loop cannot counter wing
// lift -- it can only take rotor thrust to zero, and the wing keeps lifting.
// The vehicle departed and crashed on the subsequent manual braking attempt.
//
// There is no safe automatic recovery without proper energy management (a
// TECS-like controller), so engagement is simply restricted to the envelope the
// loop was actually validated in: station keeping from hover. Engagement above
// this speed is REFUSED (and warned about); an already-engaged loop is left
// alone, because disengaging mid-flight is its own hazard.
// NOTE (2026-08-31, step 136): this is now compared against the BODY FORWARD
// speed, not the ground-speed magnitude. The hazard it guards is aerodynamic
// (the wing dominating above ~5-6 m/s), so the right measure is the component
// the wing sees. In wind the two differ: measured at 6 m/s of wind, the
// back-transition finished with v_fwd = 0.02 m/s while ground speed sat at
// 7.1 m/s, and the old magnitude gate refused pos_hold for 207 s.
static constexpr float POS_ENGAGE_V_MAX = 3.0f;  // m/s, BODY FORWARD speed
// Lateral component, gated separately and WIDER: the wing produces no lift
// sideways, so the runaway mechanism POS_ENGAGE_V_MAX guards simply does not
// exist on this axis. It is still bounded, because a large sideways rate at
// engagement is its own transient.
//
// 6.0 -> 12.0 (2026-08-31, step 138). MEASURED: in 6 m/s of wind the vehicle
// settled with the wind on its BEAM -- v_fwd stayed inside its limit (only 15%
// of samples above 3.0) but v_lat sat at -9.8 m/s for the whole HANDOFF phase,
// so the lateral gate alone refused pos_hold 100% of the time and HANDOFF ran
// 205.9 s instead of ~18 s. The first fix (step 136) got the AXIS right and
// the NUMBER wrong: 6.0 was picked as "twice the forward limit", which is
// reasoning by symmetry, not by measurement.
// 12.0 is chosen from the physics of what the gate protects: the wing stalls
// the position loop by producing LIFT, and it produces none sideways at any
// speed this airframe reaches. What remains is a transient bound, and 12 m/s
// of pure sideways drift is already far outside anything the mission produces
// (measured max 10.6 m/s, in a 6 m/s wind that is itself the design case).
// If a future case needs more, the honest fix is to drop the lateral gate
// entirely rather than keep widening it -- but that must be measured first.
static constexpr float POS_ENGAGE_V_LAT_MAX = 12.0f;  // m/s, body lateral speed

// --- BACK-TRANSITION, cruise -> hover (blocker B5, 2026-07-29 step 31) ---
// See backTransition() in TiltrotorIndiControl.hpp for the state machine and
// backtrans_loop.m for the full derivation. Every value below was MEASURED in
// SITL; MATLAB cannot reproduce this manoeuvre at all (one longitudinal surface,
// ~25 N of lift at 12 m/s against a 49 N weight), so the MATLAB twins of these
// constants in tiltrotor_params.m (p.bt.*) exist for SYNC ONLY -- do not
// "validate" a change to them there.
//
// Why a ceiling instead of weighting Fx: step 31 phase 0 re-solved the allocator
// offline per sample and attributed the cruise tilt runaway -- zeroing the Fz
// demand REVERSES the drift (+0.64 -> -0.45 deg/s) while zeroing Fx changes
// almost nothing. The driver is the altitude channel, and WS_FZ/WS_FX = 400
// (160,000x in the objective). Raising WS_FX to that magnitude is what wrecked
// yaw in step 7. A box constraint is the one thing the weights cannot argue with.
//
// BT_TILT_V_REF (2026-08-27, step 100): RETRACT's ceiling used to ramp down on
// a fixed clock (BT_RETRACT_RATE, 2 deg/s), independent of how fast v_h was
// actually falling. Measured consequence (WLS_LOCKUP_INVESTIGATION_REPORT.md
// step 100): tilt reached the 9 deg floor at t=88.3s with v_h STILL 8.92 m/s,
// and stayed pinned near the floor (8.5-10.9 deg) through t=93.3s while v_h was
// still 3.72-6.44 m/s -- ~8 seconds with the wing rotors nearly VERTICAL while
// real forward airspeed was still substantial. SITL cannot see why that
// matters: its propeller is an idealised disk, with no blade-flapping/edgewise
// loading model, so this schedule mismatch is invisible in simulation and only
// bites on real hardware (the same family of failure as a helicopter's
// retreating-blade stall -- a disk spun close to perpendicular to a real
// crosswind it was not designed to see). The fix: tilt_ceil is no longer an
// independent integrator at all -- it is recomputed FRESH every tick, directly from
// the CURRENT measured v_h, so it can never be lower than what the actual
// (still-falling) speed justifies, and it also RISES again immediately if v_h
// does (e.g. a gust) rather than staying wherever a clock left it.
// BT_TILT_V_REF is the speed at/above which the ceiling is fully open
// (TILT_MAX, i.e. inert): chosen near this airframe's typical FtState::CRUISE
// settling speed (FT_CRUISE_V_SP = 11) so the corridor spans the realistic
// entry range without sitting saturated at either end for most of RETRACT.
static constexpr float BT_TILT_V_REF = 12.0f;        // m/s, v_h at which ceiling = TILT_MAX
// BT_CEIL_FLOOR: 9 deg. Lower starves yaw -- a 9/7/5 deg floor sweep gave yaw
//   drift +0.0205/+0.0308/+0.0384 rad/s, confirmed by REVERSING the sweep order
//   in a second flight, and at 5 deg the vehicle departed on its own. The reason
//   is structural: delta1 pins on TILT_MIN, so the differential EQUALS the
//   ceiling, and with delta1 = 0, tau_z = -0.25*Fx -- the yaw trim torque and the
//   residual forward force are the same physical quantity.
// BT_RELEASE_V: below this the ceiling is RELEASED. Not a compromise -- the
//   ceiling's own justification is gone by then (measured nu_des(Fz) ~ 0.00 at
//   the floor, the wing has stopped lifting), while keeping it clamped kills yaw
//   (981 deg of rotation at pitch +4, 2117 deg at +6; released: +-2 deg).
// BT_PITCH_MAX / BT_BRAKE_V_FULL: below the release speed body pitch really does
//   brake (measured 5.74 -> 3.01 -> 0.10 m/s at pitch 0/+2/+4 deg, vz within
//   +-0.23). The law fades to zero with speed because +6 deg on a stopped vehicle
//   accelerates it BACKWARD (v_h 0.63 -> 5.77) -- but it must not fade below the
//   pitch that merely holds station BEFORE the handoff speed; see BT_BRAKE_V_FULL.
static constexpr float BT_CEIL_FLOOR = 0.1570796f;   // rad (9.0 deg)
// BT_RELEASE_V was 8.0; RAISED to 10.0 on 2026-07-31 (step 38) -- item (R).
// The old value sat ON THE BOUNDARY OF AN EQUILIBRIUM. At the floor ceiling
// (9 deg) with pitch = 0 the vehicle does not stop, it approaches a TERMINAL
// SPEED: delta1/delta2 are pinned at TILT_MIN, so the yaw trim holds delta0 at
// the floor and the residual ~2.4 N forward thrust balances drag. Step 37
// measured that speed at 8.0-9.3 m/s -- i.e. AT or ABOVE the 8.0 threshold, so
// the exit condition may never be met. Measured consequence: one flight sat in
// RETRACT for 200.3 s without ever going below 8.97 m/s, and drifted +117.7 deg
// in yaw, because a ceiling parked exactly at the trim leaves delta0 nowhere to
// move and yaw's modulation authority is ZERO (the same mechanism BT_CEIL_FLOOR
// documents above). The flights that DID complete crossed the threshold on the
// initial deceleration transient, not from the settled state: 5 of 8.
// 10.0 leaves 0.7 m/s over the worst measured terminal speed.
// (The "terminal speed is 5.7-7.9 m/s" note under BT_BRAKE_CEIL below was an
// under-sampled record from step 31 and is corrected there.)
static constexpr float BT_RELEASE_V = 10.0f;         // m/s, ceiling RAISED below this
// BT_FLOOR_DWELL: the AERO-INDEPENDENT half of the item (R) fix (step 38).
// Raising BT_RELEASE_V closes (R) in SITL but NOT on hardware: 10.0 was chosen
// against a terminal speed measured on Gazebo's five lift-drag surfaces, and if
// the real wing drags differently the threshold can land inside the equilibrium
// again -- the same class of risk as B2/B3. So the exit was decomposed into two
// terms, and this is the one that never looks at speed: once the ceiling has sat
// on the floor this long, go to BRAKE anyway. It converts an UNBOUNDED failure
// (yaw starved indefinitely) into a BOUNDED delay, whatever the aerodynamics.
// Why BRAKE and not an abort to IDLE: the dwell expires exactly at the LOWEST
// speed RETRACT can ever deliver (the speed has reached its asymptote, so
// waiting longer buys nothing), and BRAKE raises the ceiling to BT_BRAKE_CEIL,
// which is what GIVES YAW ITS AUTHORITY BACK -- it removes the actual harm
// rather than merely stopping the manoeuvre. Aborting would rescue yaw too, but
// would leave the vehicle in cruise with no way home.
// 20 s comes from the measurement: after the floor was reached the speed went
// 9.50 -> 8.62 in 16.7 s, 8.29 in 33.4 s, 8.00 in 89.1 s, so by 20 s the vehicle
// is within ~0.4 m/s of terminal. Bounded yaw exposure: ~0.59 deg/s * 20 s
// ~ 12 deg, comfortably inside the <= 45 deg criterion.
static constexpr float BT_FLOOR_DWELL = 20.0f;       // s, dwell at the floor before BRAKE anyway
// BT_BRAKE_CEIL: what the ceiling is raised TO in BRAKE -- NOT all the way off.
// The first automatic flight (2026-07-29) released it fully to TILT_MAX at
// 8.4 m/s and the manoeuvre UNDID ITSELF: the Fz-driven runaway restarted, tilt
// went 24 -> 90 deg and the vehicle re-accelerated 6.3 -> 12 m/s. The manual
// probes had survived a full release only because they released later, at a
// settled 5.7-6.9 m/s, where the wing lift is ~1.5x smaller.
// The correction is not just a lower threshold (the terminal speed at the floor
// can deadlock RETRACT -- this line used to record that speed as "5.7-7.9 m/s";
// step 37 measured 8.0-9.3 and item (R) came out of exactly that gap, see
// BT_RELEASE_V above). The real diagnosis:
// yaw was never starved by the ceiling's EXISTENCE, only by the ceiling sitting
// AT the trim, leaving delta0 nowhere to move. A ceiling well above the trim
// restores full yaw modulation (delta1 stays on TILT_MIN, so the differential is
// delta0, free over [0, ceil]) while still capping the runaway. 20 deg is what
// the flown data supports: in the successful manual probe the tilt transiently
// reached 18.8 deg after release and settled back to ~10 deg.
static constexpr float BT_BRAKE_CEIL = 0.3490659f;   // rad (20.0 deg)
// BT_TILT_FLOOR_MAX (2026-08-27, step 101): the wing tilt LOWER bound reaches
// this value once v_h >= BT_TILT_V_REF, tapering to TILT_MIN as v_h -> 0 (see
// backTransition()'s header for the propeller-loading rationale). Deliberately
// kept well under BT_BRAKE_CEIL (20 deg): a floor that reaches or exceeds the
// active ceiling pins tilt at a single value, which is the EXACT yaw-starvation
// failure BT_CEIL_FLOOR documents above (delta1 pinned at TILT_MIN, delta0
// nowhere to move) -- the floor must open real headroom under whatever ceiling
// is active (74-90 deg in RETRACT at this speed range, a fixed 20 deg in
// BRAKE), not close it. 12 deg leaves 8 deg of box under BT_BRAKE_CEIL, the
// same order of headroom BT_BRAKE_CEIL's own header measured tilt actually
// using (transient peak 18.8 deg, settled ~10 deg).
static constexpr float BT_TILT_FLOOR_MAX = 0.2094395f; // rad (12.0 deg)
// BT_PITCH_MAX is the braking MARGIN above the station-keeping trim, not the
// total command -- see BT_TRIM_PITCH below and btBrakePitch().
static constexpr float BT_PITCH_MAX = 0.0698132f;    // rad (4.0 deg)
// The brake law's other half is the station-keeping trim angle,
// asin(FX_TRIM / (MASS*GRAVITY)) = 3.39 deg. It is COMPUTED in btBrakePitch()
// from FX_TRIM rather than written here as a number, so that re-measuring the
// trim on the real vehicle (a hardware-checklist item) updates the brake law
// too -- a literal would have to be remembered, and this project has already
// paid for constants that must be changed in pairs (steps 22, 27).
// Why the brake law needs it: delta1 and delta2 sit pinned at TILT_MIN (item
// (P), one-sided tilt range), so the yaw trim keeps delta0 at 10-15 deg and that
// is a PERMANENT 3.1-4.1 N of forward thrust. A brake law that fades to zero
// therefore fades below the force it exists to fight, and the manoeuvre parks in
// a stable equilibrium instead of stopping -- measured three times.
// ITEM (S), 2026-08-03 (step 39): the VALUE did not change, the SIGNAL it is
// applied to did. This threshold is now tested against the SIGNED body forward
// speed, not against the horizontal magnitude. The brake law only controls the
// forward axis, nothing controls the lateral axis before HANDOFF, and a lateral
// drift of 3.04 m/s held the magnitude at 3.08 while the vehicle was already
// going backward at -0.51 -- handoff never fired and the vehicle ran away
// BACKWARD to 12.8 m/s. See btBrakePitch()/backTransition().
// CAUTION: it is still the same NUMBER as POS_ENGAGE_V_MAX but no longer the
// same TEST -- pos_hold's engage gate remains a magnitude gate, so with lateral
// drift the handoff can be requested and refused, and is retried every tick.
// Deliberate residual, measured rather than assumed; see backTransition().
static constexpr float BT_HANDOFF_V = POS_ENGAGE_V_MAX; // m/s, forward-speed handoff threshold
// BT_BRAKE_V_FULL was 5.0 m/s; TIED to BT_HANDOFF_V on 2026-07-30 (step 37)
// after the manoeuvre STALLED above the handoff speed in two independent
// flights. The brake pitch fades with speed, but the forward force it has to
// beat does NOT: delta1 and delta2 sit pinned at TILT_MIN (item (P), one-sided
// tilt range), so the yaw trim holds delta0 near 10.5 deg and that produces a
// steady ~3.1 N of forward thrust. The nose-up angle needed merely to STAND
// STILL is therefore asin(3.13 / (MASS*GRAVITY)) = 3.66 deg -- only just under
// BT_PITCH_MAX. At 5.0 the law delivered 2.80 deg = 2.42 N at 3.5 m/s, i.e.
// LESS than the 3.13 N push, and the vehicle settled into a stable equilibrium
// at 3.2-3.5 m/s that it never left (90+ s, measured 3.54 +- 0.04 and 3.64 m/s
// in logs 10_46_58 / 10_51_38) -- never reaching BT_HANDOFF_V, so pos_hold was
// never requested and the back-transition never completed.
// Step 31's two flights DID complete, but on drag alone and only just: one of
// them spent 12.1 s on the last 0.5 m/s. So the recorded "verified" result was
// really a marginal one -- exactly the situation step 20 warns about.
// The fade itself is right (+6 deg on a stopped vehicle accelerates it BACKWARD
// to 5.77 m/s); what was wrong is WHERE it fades. Full braking authority must
// survive down to the handoff speed, below which pos_hold owns pitch anyway.
// Written as a dependency rather than a number so the two cannot drift apart.
// Item (S) (step 39): this fade is now driven by max(0, v_fwd), so the margin is
// exactly zero once the vehicle is moving backward -- that is the term that
// actually stopped the measured reversal.
static constexpr float BT_BRAKE_V_FULL = BT_HANDOFF_V; // m/s, forward speed at which pitch caps
static constexpr float BT_MIN_ALT = 15.0f;           // m, refuse to start lower than this

// State machine states, mirroring backtrans_loop.m.
enum class BtState : int32_t { IDLE = 0, RETRACT = 1, BRAKE = 2, HANDOFF = 3 };

// --- forward transition, hover -> cruise (item (V), step 42, 2026-08-03) ---
// tiltrotor_params.m p.ft.*. The MIRROR of the BT_* block above, and it did not
// exist until now: the mission profile is meant to be fully autonomous (takeoff
// MC -> cruise fixed-wing -> land MC) but going FORWARD was only ever an open
// `fx_sp` ramped by hand from the debug console. The asymmetry was historical --
// the back-transition got a state machine because its absence was a measured
// FAILURE (blocker B5); forward flight "already worked" because a human drove it.
//
// FT_FX_CRUISE / FT_FX_RATE come from the MEASURED transition profile, not from a
// guess: run_transition_test ramps 0 -> 10 N over 12 s and reaches 10.86 m/s with
// 0.0% thrust saturation and a clean pos_hold release (rate activity DROPS at
// release, p RMS 0.2123 -> 0.1503). The back-transition tests enter at 15 N /
// 15-16.5 m/s. 12 N sits between them and puts the wing tilt near 45 deg (at
// 15 m/s the measured tilt is 45-54 deg).
static constexpr float FT_FX_CRUISE = 12.0f;       // N, cruise body-x force command
// FT_FX_RATE raised from the originally-measured 10/12 N/s (2026-08-21, Adim
// 64/65): item (N)'s persistent ~3 N forward bias (TiltrotorIndiParams.hpp,
// "horizontal position outer loop" note above) is NOT owned by this manoeuvre
// -- pos_hold is released on entry (see forwardTransition()'s header) and
// RAMP controls only fx/pitch, not roll/yaw -- so it accumulates unopposed for
// as long as RAMP takes to reach FT_CRUISE_V. Three live SITL runs the same
// day (report Adim 62-64) all showed 60-130 deg of yaw drift and a stalled
// v_h during a 14.4 s ramp. Cutting the ramp time is the lowest-risk lever
// available without touching forwardTransition()'s own logic or hoverTrim()'s
// yaw-nulling trim (both previously validated, see report Adim 12/13) --
// less TIME for the same bias to accumulate, not a fix to the bias itself.
static constexpr float FT_FX_RATE = 2.0f;          // N/s, ~6 s to FT_FX_CRUISE (was 14.4 s)
// FT_CRUISE_V: deliberately BELOW the back-transition's BT_RELEASE_V (10.0) so the
// two manoeuvres cannot chase each other around the same speed.
static constexpr float FT_CRUISE_V = 8.0f;         // m/s, "cruise reached"
// FT_ALT_BAND: SAFETY 1, the direct detector for step 29's escape climb (which ran
// 44 m in 35 s). 5 m is far under that and far over a normal transition's altitude
// change (<1 m measured), so it neither misses nor false-alarms.
static constexpr float FT_ALT_BAND = 5.0f;         // m, deviation from entry -> ABORT
// FT_TIMEOUT_S: SAFETY 2, and the AERO-INDEPENDENT one (step 38's lesson: every
// threshold is a measurement OF AN ENVIRONMENT, so pair it with one that is not).
// If the real wing lifts or drags differently the vehicle may never reach
// FT_CRUISE_V and the speed term would never fire. 30 s comfortably covers the
// ~6 s ramp (Adim 64/65's FT_FX_RATE increase) plus acceleration margin --
// left unchanged since it was already generous before that change.
static constexpr float FT_TIMEOUT_S = 30.0f;       // s, RAMP ceiling -> ABORT
static constexpr float FT_MIN_ALT = 20.0f;         // m, refuse to start lower
// FT_ALLOW_ABORT: REQUIREMENT (2026-08-04) -- the mission profile is one piece
// and THE FORWARD TRANSITION DOES NOT CANCEL. Both detectors above still run and
// are reported via `warn_code`; only their action is removed. Full reasoning,
// including what this costs and the zero-margin escape path it removes, is in
// forwardTransition()'s header. Setting this true restores the old behaviour
// EXACTLY -- but close FT_MIN_ALT - FT_ALT_BAND > BT_MIN_ALT first (today
// 20 - 5 = 15 = 15, i.e. zero).
static constexpr bool FT_ALLOW_ABORT = false;
// NOTE: an abort does NOT mean fx = 0. Step 30 tried exactly that and the vehicle
// did not slow down, because Fx is far too weak an objective term to retract the
// tilts. Aborting means REQUESTING THE BACK-TRANSITION.
enum class FtState : int32_t { IDLE = 0, RAMP = 1, CRUISE = 2 };

// ---------------------------------------------------------------------------
// INIS DIZISI (2026-08-31, Adim 153 -- madde B0)
// ---------------------------------------------------------------------------
// NEDEN MODULE TASINDI: profil (kademeli alcalma, flare, temas) PC tarafindaki
// run_mission_test.py icindeydi ve o betik hedefi `px4-mc_indi_tiltrotor
// test_sp` POSIX KABUK ISTEMCISIYLE gonderiyordu. O yol yalnizca SITL'de var
// (indi_sitl_common.py basligi: "external MAVLink clients cannot publish
// arbitrary uORB topics"). Kartta karsiligi yok, ve flight_mode_manager ile
// mc_pos_control bu airframe'de bilerek durduruldugu icin PX4'un kendi land
// modu da yok. Yani kart bugun flash edilse OTONOM INIS OLMAZDI.
//
// SAYILAR PC BETIGINDEN BIREBIR TASINDI, yeniden ayarlanmadi: hepsi olculerek
// bulunmustu ve gerekceleri run_mission_test.py'de yazili.
//   1 m kademe   : 1.5 m kademe inis fazinda 13 BIG_M uretmisti (RUNBOOK (O))
//   1.5 s periyot: irtifa dongusu 1 m'lik bir adimi bundan hizli kapatamiyor
//   FLARE 1.5 m  : altinda hedef YERIN ALTINA surulur
//   TOUCH  0.15 m: 0.5 m COK FAZLAYDI -- temastan sonra da bastirip araci
//                  zemine gommeye calisiyordu (2026-08-29)
enum class LandState : int32_t { IDLE = 0, DESCEND = 1, FLARE = 2, TOUCHDOWN = 3 };

static constexpr float LAND_STEP_M = 1.0f;      // m, her kademede inilecek
static constexpr float LAND_STEP_S = 1.5f;      // s, kademeler arasi bekleme
static constexpr float LAND_FLARE_ALT = 1.5f;   // m AGL, altinda temas komut edilir
static constexpr float LAND_TOUCH_Z = 0.15f;    // m, YER DATUMUNUN ALTI
static constexpr float LAND_DONE_ALT = 0.25f;   // m AGL, altinda inis tamamlandi

// TEMAS OLCUTU: dikey itki agirligin bu kesrinin altindaysa arac yerdedir.
// IRTIFADAN BAGIMSIZ, ve bu bilerek -- yanilan sinyal tam olarak irtifaydi.
// Olculen ayrisma (6 kosum): yerde 5.2/12.3/13.1 N, asili-alcalan
// 34.2/42.3/50.4/50.6 N. Esik 0.5*49.05 = 24.5 N tam ortadan geciyor.
// |vz| KOSULU YOK: denendi ve aracin acikca yerde oldugu bir kosumu kacirdi
// (EKF'in vz'si 0.263 okuyordu) -- vz, yanilan irtifayla ayni kestirimciden
// gelir. Yerine SUREKLILIK: 5.2 N ile havada olsaydi arac 8.8 m/s^2 ile
// duserdi, LAND_TOUCH_DWELL suresince metrelerce yol eder.
static constexpr float LAND_GROUND_THRUST_FRAC = 0.5f;
static constexpr float LAND_TOUCH_DWELL = 1.5f;  // s, kesintisiz temas suresi

// ALCALMA TAKILDI: bu sure boyunca irtifa LAND_STALL_DZ'den az degistiyse
// profil ilerlemiyordur. Modul KENDI DISARM ETMEZ; yalnizca DESCEND'de
// kalir ve durumu yayinlar -- karar disaridadir.
static constexpr float LAND_STALL_DZ = 0.05f;    // m
static constexpr float LAND_STALL_S = 4.5f;      // s (betikteki 3 x 1.5 s)

// Fixed-wing terminal mode (Adim 59-61 MATLAB port). See fixedWingTransition()/
// fixedWingControlLaw() in TiltrotorIndiControl.hpp for the mechanism, and the
// FIXED-WING MODE constants block below (after AERO_WING_A0) for the gains.
// Only reachable from FtState::CRUISE -- a ONE-WAY DOOR going IN, same shape
// as FT itself (see forwardTransition()'s header): GLIDE cuts every rotor to
// zero and slews tilt UP to TILT_MAX before ACTIVE takes over.
//
// RETURN (2026-08-27, step 103) is the MIRROR of GLIDE, going back out: a
// first cut that dumped ACTIVE straight to WLS/INDI at tilt=90 deg/v~16 m/s
// tumbled immediately (roll 173 deg within 1 tick, altitude 64m -> 195m in
// 8s, WLS_LOCKUP_INVESTIGATION_REPORT.md step 102) -- the INDI/LESO state is
// frozen the entire time _fw_state != IDLE, so handing it live control at the
// most extreme tilt/speed this airframe reaches is exactly the wrong moment.
// RETURN reuses GLIDE's own proven answer to "how do you cross the high-tilt
// regime": cut every rotor, hold attitude with surfaces alone (same law), and
// slew tilt back down to TILT_MIN -- only THEN hand off to IDLE/WLS, at the
// same near-zero-tilt, rotors-already-off starting point every normal hover
// arm already starts from.
enum class FwState : int32_t { IDLE = 0, GLIDE = 1, ACTIVE = 2, RETURN = 3 };

// --- CRUISE ENERGY MANAGEMENT (this airframe's TECS), steps 47/49/53 ---------
//
// Ported from cruise_speed_loop.m + cruise_pitch_loop.m + tiltrotor_params.m.
// Every number below is MEASURED in MATLAB; none of it has flown in SITL yet, so
// the first flight must watch fx, tilt and the saturation count. The MATLAB
// derivations are reproduced in the two loop headers in TiltrotorIndiControl.hpp
// -- what follows is only what a reader needs to judge the VALUES.
//
// WHY IT IS NOT "REAL" TECS: classic TECS ties thrust+pitch to total energy and
// energy DISTRIBUTION because in a fixed-wing both channels move both speed and
// altitude. This parametrisation is already more separated -- the outer loop
// commands BODY-AXIS FORCE (fx, Fz) and the WLS decides which actuator makes it,
// and altitude is already closed on Fz (altitudeLoop). The missing degrees of
// freedom were speed (step 47) and the pitch trim (steps 49/53).
static constexpr bool TECS_ENABLE = true;
// v_sp from the measured envelope: in MATLAB fx = 8..14 N ALL settled at
// 16-17 m/s (steep drag slope) and the SITL cruise measured 15.2 m/s. 16.0 is
// inside both and keeps the wing tilt off its 90 deg mechanical stop.
static constexpr float TECS_V_SP = 16.0f;          // m/s, target BODY FORWARD speed
// FT_CRUISE_V_SP: a SEPARATE, LOWER target used ONLY while cruiseSpeedLoop is
// regulating FtState::CRUISE (step 99, 2026-08-26) -- deliberately NOT the same
// constant as TECS_V_SP/FW_V_SP, so this fix cannot touch the already-validated
// fixed-wing GLIDE/ACTIVE cruise speed (FW_V_SP is a compile-time alias of
// TECS_V_SP and stays untouched).
//
// Why: with cruiseSpeedLoop's actual target at TECS_V_SP=16, a live SITL run
// that engages `ft_enable` and never calls `force_fw` (i.e. FtState::CRUISE is
// left running instead of handing off to GLIDE promptly, the realistic shape
// of an autonomous mission where the fixed-wing entry gate has not yet
// triggered) OVERSHOOT past 16 into a 20-25 m/s regime before the loop reins
// it back in -- and at that speed the aero yaw/roll moment needing correction
// grows large enough that the wing-tilt differential trim saturates BOTH
// rotors at once (one at the FtState-only ceiling, `ftceil`/step 98, the other
// at TILT_MIN=0), collapsing differential authority onto thrust and producing
// a violent (self-recovering, but NOT clean) ~15 s oscillation -- roll swung
// -37..+52 deg, altitude excursion 30m -> 64m, WLS_LOCKUP_INVESTIGATION_REPORT.md
// step 99. Capping the FtState::CRUISE target itself well clear of that ~17-20
// m/s onset removes the overshoot's ability to ever reach the unsafe regime,
// rather than trying to survive it after the fact (the ftceil/step-97-98
// approach) -- SITL-validated, see the report.
static constexpr float FT_CRUISE_V_SP = 11.0f;     // m/s, FtState::CRUISE-only target
// Gains from the measured drag slope c = dD/dv ~ 2.3 N/(m/s) (fx 8->10 N moved
// the equilibrium 16.01->16.89 m/s). Kp = 1.0 -> closed-loop tau = m/(c+Kp) =
// 1.5 s, still ~6x slower than the attitude loop. Ki = Kp/5 -> ~5 s integral
// time. Deliberately NOT aggressive: this loop must stay far below the attitude
// loop, and step 51 measured that raising Kp (1/4/8) changes nothing about the
// speed limit anyway -- it was never a gain problem.
static constexpr float TECS_KP = 1.0f;             // N per (m/s)
static constexpr float TECS_KI = 0.2f;             // N per (m/s) per s
// fx_max is a SAFETY CEILING, not a performance number: in the step-46 sweep
// fx = 14 N diverged and 12 N was marginal. It is also what now limits top
// speed -- with the step-53 feed-forward the vehicle no longer diverges at
// v_sp = 26, it SETTLES at 24.9 m/s because this ceiling binds. The limit became
// a choice instead of a failure, and this constant is that choice.
static constexpr float TECS_FX_MAX = 13.0f;        // N
//
// --- the PITCH half (energy distribution), steps 49 + 53 ---
// Step 47 measured the wall: with pitch pinned at zero everywhere, level flight
// is impossible above
//     V_wb = sqrt(W / (0.5*rho*S*cla*a0)) = 16.925 m/s
// (the speed at which the wing carries full weight at theta = 0), and every
// surfaces-off run had been stopping at 16.85-16.90 m/s regardless of fx.
static constexpr bool TECS_PITCH_ENABLE = true;
// Gate: in step 29's regime (~5-6 m/s) nose-up is a CLIMB command and the
// altitude loop cannot fight the wing, so authority must be EXACTLY zero there.
// Smoothstep 13 -> 16 m/s, i.e. full authority just under V_wb.
static constexpr float TECS_PITCH_V_ON = 13.0f;    // m/s
static constexpr float TECS_PITCH_V_FULL = 16.0f;  // m/s
// Ki from the time-scale separation, not from feel: dL/dtheta = qbar*S*cla =
// 841 N/rad at 17 m/s, so tau = 1/(841*Ki). The altitude loop's tau is ~1.7 s
// (Kp_z = 0.6) and this trim must be ~10x slower -> tau ~ 20 s -> Ki = 5.9e-5.
// 5e-5 chosen (tau = 23.8 s @ 17 m/s). Step 53 measured that Ki*4 ALSO removes
// the speed wall, and rejected it: it would cut the separation to 3.5x, while
// the feed-forward below gives the same result for free.
static constexpr float TECS_PITCH_KI = 5e-5f;      // rad / (N*s)
// The required trim range is COMPUTABLE -- theta = W/(qbar*S*cla) - a0 gives
// +3.4 deg at 12 m/s, 0 at 16.9, -1.0 at 20 -- so 6 deg is far above it and
// still a small authority.
static constexpr float TECS_PITCH_MAX = 0.10471976f; // rad (6 deg)
// TARGET actuator vertical load. NOT ZERO, and that is a MEASURED limit rather
// than a preference: step 49's first attempt used 0 ("full wing-borne means
// Fz_sp -> 0"), the law worked perfectly (tilt 41->65 deg, wing 93->106%,
// altitude and speed held, only 0.8 deg of pitch) and the vehicle diverged the
// instant rotor thrust touched 0.00 N. The cause is structural: in the
// effectiveness matrix the tilt column is MULTIPLIED BY THRUST, so a zero-thrust
// tilt rotor has exactly zero control authority, and the Fx channel has no
// surface at all. Measured: -12 N -> T0 ~ 4.95 N (healthy), -10 -> 1.83,
// -8 -> breakdown. Zero thrust is not a target, it is a cliff.
static constexpr float TECS_PITCH_FZ_SP = -12.0f;  // N (FRD, negative = actuators lift)
// Aero constants for the feed-forward ONLY (step 53). These mirror MATLAB's
// p.aero.*, which are read straight from the SDF -- note rho here is the SDF's
// air_density (1.2041), NOT the RHO_AIR = 1.225 used for qbar in the allocator.
// MATLAB uses the same two different densities in the same two places; the
// difference is 0.4% and this port keeps it rather than silently "fixing" it,
// so the two implementations stay bit-comparable.
static constexpr float AERO_RHO = 1.2041f;         // kg/m^3, SDF air_density
static constexpr float AERO_WING_S = 1.0f;         // m^2, both wing halves (2 x 0.5)
static constexpr float AERO_CLA = 4.752798721f;    // 1/rad
static constexpr float AERO_WING_A0 = 0.05984281113f; // rad, wing incidence

// --- FIXED-WING MODE (Adim 59-61 MATLAB port: fixedwing_control_law.m /
// run_full_transition_glide_test.m, sitl/WLS_LOCKUP_INVESTIGATION_REPORT.md
// section Adim 58-61) --------------------------------------------------------
//
// WHY THIS EXISTS AS A SEPARATE LAW, NOT AN EXTENSION OF TECS ABOVE: the
// TECS_PITCH_FZ_SP comment two blocks up already measured the answer in THIS
// codebase, independently of the MATLAB work -- "Zero thrust is not a target,
// it is a cliff": the effectiveness matrix multiplies the tilt column by
// thrust, so a zero-thrust tilt rotor has exactly zero control authority.
// TECS therefore keeps -12 N of actuator lift forever and never reaches true
// wing-borne flight. Adim 58 hit the SAME cliff from the MATLAB side (T=0,
// tilt locked at 90 deg -> INDI/WLS diverges: pitch 84 deg, roll 180 deg) and
// concluded the only way past it is a control law that does not need thrust
// authority at all -- pure control-surface PD for attitude, shared wing-rotor
// thrust for speed only, altitude closed through PITCH (angle of attack), not
// through an Fz channel that no longer exists once the rotors point forward.
static constexpr bool FW_ENABLE = true;

// --- entry gate ---
// Only armed from a COMPLETED forward transition (FtState::CRUISE) -- mirrors
// FT_MIN_ALT/FT_CRUISE_V's shape. GLIDE below cuts EVERY rotor to zero for a
// few seconds, so the altitude margin is generous, not a token check.
//
// FW_TRIGGER_V lowered from an originally-assumed 14.0 (2026-08-21, Adim 66).
// TECS_V_SP=16 above is a MATLAB-derived target this module's own header
// comments already flagged as unvalidated in SITL ("none of it has flown in
// SITL yet"). A 5.5-minute live SITL measurement (Adim 66) found FtState
// reaches CRUISE reliably but cruise_v_fwd then settles into a genuinely
// STABLE equilibrium at 8.0-8.5 m/s under FT_FX_CRUISE=12N (steady for the
// full 3.5 minutes observed, not still climbing) -- well under both the
// TECS target and this constant's old value, so fixed-wing mode could never
// arm via its real gate no matter how long RAMP/CRUISE was given. 7.0 sits
// with a safe margin under the measured floor of that equilibrium band while
// still requiring genuine forward flight, not a near-hover speed.
static constexpr float FW_TRIGGER_V = 7.0f;        // m/s, body-forward speed required to arm GLIDE
static constexpr float FW_MIN_ALT = 30.0f;         // m AGL, refuse to start below this

// COORDINATED-FLIGHT CHECK (2026-08-21, Adim 67). Adim 66's first live
// production-path trigger armed GLIDE at v_fwd=7.0 m/s with no other
// condition -- and happened to fire in the middle of a speed dip/recovery
// (cruise_v_fwd 3.36 -> 0.68 -> 5.55 m/s over the preceding 15 s), i.e. while
// the vehicle was mid-disturbance, not settled. Result: roll pinned at
// 57.6 deg with yaw rotating 160+ deg in the next 10 s -- a graveyard-spiral
// signature -- once ACTIVE relit. GLIDE's bare-P law (no rate term) could not
// arrest whatever roll rate was already present at the moment of entry. This
// is NOT item (N)'s root cause reappearing -- it is THIS gate accepting an
// uncoordinated moment as if it were a settled cruise. A real pilot would not
// retract to a glide mid-upset either; the fix is the same: check the
// vehicle is actually settled, not just fast enough.
static constexpr float FW_MAX_ROLL = 0.17453293f;  // rad (10 deg), |roll| must be under this to arm
static constexpr float FW_MAX_YAW_RATE = 0.17453293f; // rad/s (10 deg/s), |r| must be under this to arm

// --- GLIDE: unpowered (T0=T1=T2=0), tilt free-slews to TILT_MAX ---
// FW_TILT_RAMP_RATE is MATLAB's TILT_RAMP_RATE (Adim 58, 20 deg/s) -- with no
// thrust the tilt servos carry no aerodynamic/gyroscopic load worth rate-
// limiting further, so the ramp is deliberately fast (typically 2-5 s from a
// cruise tilt of 45-70 deg). Gains are the Adim 58 bare-P law, MEASURED stable
// through an unpowered glide+simultaneous full tilt after the aileron sign fix
// (pitch 16.1 deg, roll 0.04 deg): a_ail = +Kp*phi (NOT -Kp*phi -- see
// effectivenessMatrix()'s SURF_K/SURF_CPY derivation, tau_x = -1.2*qbar*a_ail
// for a differential elevon, same sign as MATLAB's effectiveness_matrix.m).
static constexpr float FW_TILT_RAMP_RATE = 0.34906585f; // rad/s, 20 deg/s
static constexpr float FW_GLIDE_KP_ROLL  = 0.15f;
static constexpr float FW_GLIDE_KP_PITCH = 0.15f;
static constexpr float FW_GLIDE_KP_YAW   = 0.05f;
// Rate (D) terms added 2026-08-21 (Adim 67) -- the Adim 58 MATLAB law that
// measured 0.04 deg of roll was validated ENTERING GLIDE from level, un-
// disturbed flight; Adim 66's SITL trigger entered with real residual roll
// rate already present (item N related, see FW_MAX_ROLL/FW_MAX_YAW_RATE's
// note above) and the bare-P law could not damp it before ACTIVE took over.
// The FW_MAX_ROLL/FW_MAX_YAW_RATE gate now keeps GLIDE's ENTRY condition
// small, but a brief unpowered phase still deserves its own damping rather
// than relying on the gate alone -- same margin-in-depth reasoning as any
// other safety check in this module. Roll only: pitch/yaw rate coupling
// during GLIDE was not implicated in Adim 66's failure and MATLAB's own P-
// only law for those two axes remains the validated reference.
static constexpr float FW_GLIDE_KD_ROLL  = 0.12f;  // matches FW_KD_ROLL below

// --- ACTIVE: fixedwing_control_law.m port, T2 permanently 0, tilt = TILT_MAX ---
// Roll/pitch/yaw PD gains are MATLAB's tuned values (Adim 59), validated end to
// end in the Adim 61 60 s test (relight-phase max pitch 12.66 deg, max roll
// 10.16 deg tracking a 10 deg command, vs. 84/180 deg divergence on the old
// INDI/WLS relight path).
static constexpr float FW_KP_ROLL  = 0.35f;
static constexpr float FW_KD_ROLL  = 0.12f;
static constexpr float FW_KP_PITCH = 0.35f;
static constexpr float FW_KD_PITCH = 0.12f;
// FW_KP_YAW/FW_KD_YAW/FW_KI_YAW_RATE/FW_KI_ROLL/FW_KP_ARI (Adim 66-74):
// REMOVED (2026-08-22, Adim 78 C++ port). History, kept because it is the
// reason the architecture below looks the way it does: this module used to
// hold roll_sp at a fixed 0 and drive rudder DIRECTLY off heading error --
// P+D (Adim 66-70) settled into a stable, self-sustaining banked-turn
// equilibrium (roll ~57-63 deg, yaw rotating ~29 deg/s, one full turn every
// 12-13 s); adding an aileron-rudder interconnect (Adim 71) didn't move it
// (rudder pushed to saturation, r unchanged); a much stronger D-term
// (Adim 72) and a roll-angle integrator (Adim 74) each independently BROKE
// the equilibrium but overshot violently PAST wings-level into a worse,
// inverted trim (peak rates ~390-480 deg/s) -- two different axes, same
// failure, not a coincidence. A yaw-rate integrator (Adim 73) was safe but
// proved rudder alone, at ANY magnitude, could not touch r -- direct
// evidence this was never a single-axis disturbance-rejection problem.
// ROOT CAUSE (Adim 75, MATLAB-validated): the architecture itself was wrong,
// not the gains. Real fixed-wing autopilots do not hold heading with rudder
// -- they bank-to-turn (heading error -> commanded ROLL angle) and rudder's
// only job is coordinating that roll (or is left off entirely, Adim 77).
// This is true on ANY symmetric airframe, real or simulated: aileron
// deflection produces adverse yaw (differential drag) as a normal
// aerodynamic fact, not an asymmetry -- decoupled per-axis PID cannot
// coordinate it no matter how each axis's own gains are tuned.
// See fixedWingControlLaw()'s header for the replacement (FW_KP_HDG,
// FW_MAX_BANK) and the report (WLS_LOCKUP_INVESTIGATION_REPORT.md, Adim
// 75-77) for the full MATLAB validation this was ported from.

// Altitude is closed through PITCH (angle of attack), not Fz -- there is no Fz
// channel once the wing rotors point forward. err_z = z - z_sp (NED, down
// positive): this SIGN, not z_sp - z, is Adim 59's critical fix -- the
// inverted version is a positive-feedback loop that pins pitch_corr at
// -FW_PITCH_CORR_MAX exactly when the vehicle needs to climb (measured: -50 m
// drift over 20 s with the wrong sign, 0.15 m with this one). The pitch
// feed-forward re-derives TECS_PITCH_FZ_SP's own closed form
// (theta_ff = (W + Fz_target)/(qbar*S*cla) - a0) with Fz_target = 0 -- the one
// value TECS above cannot use (the cliff, see above) but is exactly correct
// here, since the wing rotors contribute zero Fz by construction at
// TILT_MAX. MATLAB measured the SAME trim empirically via a full nonlinear
// panel sweep (find_fixedwing_trim.m: 1.25 deg at 16 m/s vs. this linear
// formula's ~0.4 deg) -- the gap is expected model error between the linear
// CLA/A0 approximation and the full panel method, and FW_KI_ALT is what closes
// it in flight (same "integrator closes MODEL ERROR" philosophy as
// TECS_PITCH_KI above), not a reason to hardcode MATLAB's number over this
// codebase's own already-validated closed form.
static constexpr float FW_KP_ALT = 0.06f;          // rad per m
static constexpr float FW_KI_ALT = 0.01f;          // rad per (m*s)
static constexpr float FW_PITCH_CORR_MAX = 0.13962634f; // rad, 8 deg
static constexpr float FW_ALT_I_MAX = 200.0f;      // m*s, integrator clamp (MATLAB parity)

// Wing-rotor thrust: pure PI on forward speed, bumpless-seeded at GLIDE->ACTIVE
// (T_FF is MATLAB's measured 4.37 N/rotor at 16 m/s, find_fixedwing_trim.m --
// no drag coefficient is exposed in this file to re-derive it analytically the
// way the pitch trim above was, so it is ported as a feed-forward SEED only;
// FW_KI_V is what actually holds trim against any error in it, same role as
// every other feed-forward in this block).
static constexpr float FW_T_FF = 4.37f;            // N per wing rotor, measured (MATLAB)
static constexpr float FW_KP_V = 1.5f;             // N per (m/s)
static constexpr float FW_KI_V = 0.3f;             // N per (m/s * s)
static constexpr float FW_V_I_MAX = 10.0f;         // N*s, integrator clamp (MATLAB parity)
static constexpr float FW_V_SP = TECS_V_SP;        // m/s, reuse the existing cruise target
static constexpr float FW_V_QBAR_MIN = 8.0f;       // m/s, floor for this law's own qbar only (avoid a near-zero-speed singularity)

// FW_V_FILT_TAU (2026-08-21, Adim 68): first-order lag time constant for the
// v_fwd fed into the pitch feed-forward's qbar ONLY (see fixedWingControlLaw()'s
// header for why). ~1 s sits comfortably above GLIDE's typical 2-3 s dive
// duration (so the reference is genuinely smoothed through the transient) while
// staying short enough not to lag the eventual steady cruise trim noticeably.
static constexpr float FW_V_FILT_TAU = 1.0f;       // s

// --- tail-rotor pitch backstop, ACTIVE only (2026-08-21, Adim 69) ---
// User's proposal after Adim 66-68 all converged on the SAME banked-spiral
// failure (roll pinned ~57-62 deg, large yaw rotation) despite three
// different reference/damping fixes: re-enable T2 once tilt is complete, not
// as a general pitch controller, but as a THRUST-VECTORED SAFETY VALVE against
// the specific mechanism a graveyard spiral runs on -- as bank develops, the
// altitude loop demands more AoA to hold lift, which tightens the turn
// further. The aerodynamic elevator's authority scales with qbar and is by
// definition what was already fighting (and losing) this fight; tail thrust
// at ROTOR_PX[2] = -0.65 m (well aft of CG) is qbar-INDEPENDENT and, kept
// vertical (TILT_MIN) even in ACTIVE, is the SAME mechanism this airframe's
// hover mode already uses for pitch (verified: r x f with f pointing up at a
// negative-X moment arm gives tau_y = PX[2]*T < 0, i.e. MORE tail thrust ->
// NOSE DOWN). Since T2 >= 0 (ROTOR_TMIN), it can only ever push the nose back
// DOWN, never up -- which is exactly what a rising-AoA spiral needs and
// exactly why it cannot be misused to help climb. Below FW_TAIL_PITCH_LIMIT
// it contributes nothing (T2=0, unpowered aerodynamically-controlled cruise
// stays the norm); above it, it engages proportionally as a hard backstop.
static constexpr float FW_TAIL_PITCH_LIMIT = 0.26179939f; // rad (15 deg)
static constexpr float FW_KP_TAIL = 50.0f;          // N per rad (over the limit)

// FW_TAIL_ROLL_GATE (2026-08-22, Adim 81, user's proposal). A banked turn
// needs MORE lift, not less: only the vertical component of lift (L*cos(phi))
// opposes gravity, so a 60 deg bank needs ~2x the level-flight lift -- and
// the elevator's ONLY way to ask for that is more nose-up (higher AoA). Adim
// 78/80's failure traces show pitch reaching 18 deg (past FW_TAIL_PITCH_LIMIT)
// during exactly this kind of high-bank episode -- meaning the tail backstop
// was ACTIVELY PUSHING THE NOSE DOWN at precisely the moment the elevator
// most needed nose-up authority to hold altitude through the turn, fighting
// its own airframe's altitude loop instead of protecting it. The backstop's
// original purpose (Adim 69: arrest a runaway climbing AoA, i.e. a graveyard
// spiral) still applies near wings-level, where extra nose-up genuinely has
// nowhere useful to go -- so it is gated OFF during any real bank rather than
// removed outright. This is a RESTRICTION (makes an existing mechanism LESS
// active), not new/added authority -- deliberately the lower-risk kind of
// change after today's three live-SITL incidents (Adim 72, 74, 80).
static constexpr float FW_TAIL_ROLL_GATE = 0.26179939f; // rad (15 deg), matches FW_TAIL_PITCH_LIMIT

// --- bank-to-turn heading control (Adim 75) + rudder removed (Adim 77) ---
// Heading error commands a ROLL ANGLE (bank), not rudder -- see FW_KP_YAW's
// removal note above for why. MATLAB (Adim 75) validated this against a
// 20 deg heading-change manoeuvre from straight-and-level: peak bank
// ~21 deg, heading captured EXACTLY (20.00 deg), roll returned EXACTLY to
// 0.00 deg once captured -- both the standalone law and the full
// climb->glide->tilt->relight->cruise chain, with Kp_hdg=1.5.
//
// FW_KP_HDG lowered 1.5 -> 0.3 (2026-08-22, Adim 80). The first live SITL
// port (Adim 78) showed a slow, ACCELERATING roll/yaw departure (roll
// -0.8 -> -58.4 deg over 30 s, ratios growing 1.9/1.7/2.8/4.0x each 5 s
// window) -- the signature of a genuine closed-loop stability-margin
// problem, not a disturbance MATLAB's model can reproduce: Adim 79 injected
// a real, PERSISTENT 0.1 N*m yaw disturbance into the MATLAB plant under the
// unchanged (Kp_hdg=1.5) law and it settled at a bounded -0.16 deg roll,
// dead flat from t=10s to t=60s -- proving the law is NOT missing an
// integrator (an integral would not have helped a stability problem, and
// Adim 72/74 already measured that escalating gains on a marginal loop makes
// things WORSE, not better). The mismatch is therefore specifically between
// Kp_hdg=1.5's assumed roll-response speed and the REAL SITL airframe's
// (almost certainly slower/more lagged) actual response -- bank-to-turn
// closes an inherently INTEGRATING relationship (bank angle -> heading rate)
// with a bare P gain, and that construction is only stable with real
// TIME-SCALE SEPARATION from the inner roll loop, the same principle
// TECS_PITCH_KI's derivation states explicitly elsewhere in this file.
// 5x lower is a first, deliberately conservative SITL re-test (Adim 80) --
// not a MATLAB-derived number (MATLAB's own inner loop IS what 1.5 was
// tuned against, so it cannot tell us the right SITL value) -- to be
// increased cautiously ONLY after this is confirmed to hold bounded/stable.
// Reverted to the original MATLAB-validated 1.5 (2026-08-22, Adim 81):
// Adim 80's 0.3 did not help (reached the same failure faster, not slower),
// so there is no evidence 0.3 is safer -- keeping the documented value
// isolates Adim 81's tail-gate change as the only variable under test.
// RESTORED to 1.5 (2026-08-22, Adim 82). Was set to 0.0 as a diagnostic
// (removed the bank-to-turn loop entirely, forcing roll_sp=0 always
// regardless of psi) -- ANSWER: roll STILL reached the same ~57.5 deg
// equilibrium (t+25s=57.5, t+30s=57.5) with ZERO yaw feedback of any kind.
// This proves bank-to-turn/yaw coupling was NEVER the cause -- the problem
// is in the ROLL AXIS ALONE, against a fixed roll_sp=0 target, independent
// of yaw entirely. See fixedWingControlLaw()'s header and the report
// (Adim 82) for the full finding and the Adim 79-style roll-disturbance
// MATLAB test this motivates next. Restored since FW_KP_HDG is exonerated,
// not implicated.
static constexpr float FW_KP_HDG = 1.5f;            // rad roll-sp per rad heading error
static constexpr float FW_MAX_BANK = 0.52359878f;   // rad, 30 deg

// Rudder (Adim 77): REMOVED. MATLAB re-ran BOTH the standalone heading-change
// test and the full end-to-end chain with a_rud forced to 0 -- results were
// indistinguishable from the with-rudder case (peak bank 20.98 vs 21.43 deg,
// final roll -0.03 vs 0.02 deg, etc.). The airframe's own passive
// aerodynamic stability (measured Adim 58, controls frozen: pitch 13.9 deg,
// roll 8.3 deg through a disturbance) plus bank-to-turn roll control is
// sufficient on its own -- one fewer actuator, and the entire adverse-yaw/
// turn-coordination question above is moot if rudder is never used to begin
// with. `surf[4]` (rudder) is written as 0.0f explicitly, not left NAN --
// same "declared, not omitted" convention as T2 = 0 in GLIDE/ACTIVE.

// --- back-transition deceleration (decel_loop.m, 2026-07-29 step 30) ---
// The gate above only PREVENTS the hazard; it does not give the vehicle a way
// home. This does: when position hold is requested above the gate speed, the
// module decelerates first, then engages hold automatically.
//
// Measured failure it is designed around (step 29 ulog): at 15 deg nose-up and
// 9.6 m/s total rotor thrust had fallen to 13 N against a 49.1 N weight -- the
// tail rotor fully off -- i.e. the altitude loop had ALREADY spent all of its
// authority, and the vehicle was being carried by the WING and still climbing
// at 1.1 m/s. So a large nose-up at speed is an unrecoverable climb command,
// and no amount of thrust modulation fixes it.
//
// Design: nose-up proportional to speed, bounded by an ADAPTIVE cap that is
// pulled down whenever the vehicle climbs and recovers slowly otherwise.
// Deliberately MODEL-FREE: computing the cap from aero coefficients
// (alpha_max = a0 + L_max/(q*A*cla)) looks cleaner but cla/area differ between
// MATLAB, Gazebo and the real aircraft -- exactly the controller/plant
// interface mismatch class that caused steps 11, 12, 21 and 27. Reacting to
// measured vz transfers to hardware unchanged.
static constexpr float DECEL_KP = 0.012f;         // rad/(m/s)
static constexpr float DECEL_PITCH_MAX = 0.1396f; // rad (8 deg), well under POS_TILT_MAX
static constexpr float DECEL_VZ_DEAD = 0.3f;      // m/s of climb tolerated before capping
static constexpr float DECEL_CAP_DOWN = 0.60f;    // cap shrink rate per (m/s) of climb
static constexpr float DECEL_CAP_UP = 0.05f;      // rad/s, cap recovery rate

// Fx trim -- report item (P), see positionLoop() in TiltrotorIndiControl.hpp for
// the mechanism. Measured in MATLAB on a +-30 deg yaw step:
//   0   -> overshoot 14.5%/10.9%, asymmetry 1.33x, delta1 on the floor 100%
//   2.9 -> overshoot 13.5%/13.3%, asymmetry 1.02x, delta1 on the floor 0%
//   4.0 -> asymmetry 1.03x but Fx rises to 3.84 N -- marginal gain, higher cost
// 2.9 N is the naturally ACHIEVABLE equilibrium value (measured), not a fitted
// number. Only applied while the horizontal position loop is active.
static constexpr float FX_TRIM = 2.9f;           // N

// --- gain schedule (gain_schedule.m) ---
static constexpr float KP_ATT_HOVER[3]  = { 3.0f, 3.0f, 1.5f };
static constexpr float KP_ATT_CRUISE[3] = { 2.0f, 2.0f, 1.0f };
// KP_RATE_HOVER 4.0 -> 10.0 DENENDI ve GERI ALINDI (2026-08-31, Adim 144).
//
// TESHIS SAGLAM VE SAKLANMAYA DEGER. Ruzgarli pos_hold + esZAMANLI TIRMANIS'ta
// roll 15.5 deg p2p salaniyordu (Adim 90'in pitch limit cevriminin roll
// karsiligi). Olcum zinciri:
//   1. omega_sp ve geri hesaplanan roll_sp, roll ile AYNI frekans (0.459 Hz) ve
//      AYNI genlikte (oran 0.97) -> arac salinan bir KOMUTU sadakatle izliyor;
//      ic INDI/hiz dongusu suclu DEGIL, kaynak pozisyon dongusu.
//   2. Pozisyon dongusu logdan geri kuruldu: uretilen phi_sp 15.08 deg vs
//      olculen 14.82 deg -> model dogrulandi. Hicbir doyum yok (a_max, int_max,
//      tilt_max hepsi %0) -> limit cevrimi DOYUMDAN degil FAZ PAYINDAN.
//   3. Ruzgar bir BOZUCU, yani S = 1/(1+L) uzerinden gelir. S tepesi 0.395 Hz'de
//      1.80: dongu ruzgari 1.8 KAT BUYUTUYOR.
//   4. Olculen |roll/roll_sp| = 1.08 > 1. Birinci mertebe bir kapali cevrim 1'i
//      asamaz -> TUTUM DONGUSU REZONANS YAPIYOR. Sebep: Kp_rate/Kp_att =
//      4.0/3.0 = 1.33x; kaskad 3-5x ayrim ister.
//
// DUZELTME ISE YARADI AMA KABUL EDILEMEZ. Kp_rate 10.0 ile RUZGARLI kosumda:
//   roll p2p 15.5 -> 7.74 deg, |G_att| 1.08 -> 0.881, faz -62.4 -> -47.1 deg,
//   hover |roll|max 9.9 -> 4.96 deg, aktuator cirpinmasi 0.1697 -> 0.1431 N.
// AMA SAKIN kosumda ARAC KACTI: son v_h 8.19 m/s (olcut < 1.0), doyum %17.9,
// BIG_M 39079 (GUI). Headless sakin da kil payi kaldi (BIG_M 1). Kp_rate 8.0
// ise sakin HEADLESS'ta felaket verdi (v_h 8.08, doyum %14.07, BIG_M 29402)
// ama ayni gain ruzgarli kosumu GECMISTI.
// YORUM: yuksek hiz kazanci ZAMANLAMA SARSINTISINA dayanikliligi yok ediyor;
// ariza geri geciste (frenleme yapamiyor) cikiyor ve kosumdan kosuma degisken.
// Gercek kartta jitter HENUZ OLCULMEDI (madde H7) -- yani orada durum SITL'den
// daha kotu olabilir. Ruzgarda 8 derecelik kazanc, "arac kaciyor" riskine
// degmez. 4.0'a donuldu.
//
// SIRADAKI ADIM (knob degil, arastirma): teshis pozisyon dongusunun faz payini
// isaret ediyor. Kp_v/Ki_v yariya indirme Adim 91-92'de zaten curutuldu,
// Kp_rate yukseltmek de burada curutuldu. Geriye GECIKMENIN KENDISINI azaltmak
// kaliyor (Td = 112 ms olculdu, Adim 93b'nin aktuator dinamigiyle tutarli):
// ya aktuator modelini tahsisata dahil etmek ya da pozisyon dongusune
// gecikmeyi telafi eden bir kestirim (lead/Smith) koymak.
static constexpr float KP_RATE_HOVER[3]  = { 4.0f, 4.0f, 2.0f };

// AYRI VE ACIK ARIZA (2026-08-31, Adim 144): ARALIKLI INIS SICRAMASI.
// 5 kosumun 1'inde arac yere degip KENDINI GERI KALDIRDI. Imza (gercek yer
// yuksekligi lp.dist_bottom'dan, EKF datumundan bagimsiz):
//   t+0 s  dist_bottom 0.29 m  -> temas
//   t+3 s  kuyruk motoru 0.256 -> 0.000 (KAPANIYOR)
//   t+6..24 s  on motorlar 0.654 -> 0.767 TIRMANIYOR, kuyruk 0'da (ornekl. %42)
//   sonuc  dist_bottom 0.29 -> 1.18 m  (0.84 m SICRAMA)
// ON ROTORLAR x=+0.27'de: on artip kuyruk sifirlanmasi = BURUN YUKARI + toplam
// itki artisi. Yani arac inisi kendi iptal ediyor.
// Diger 4 kosumda (142 sakin, 143a sakin, 143b sakin, 143b ruzgar) sicrama
// -0.03..+0.007 m, kuyruk hic sifirlanmiyor -> SISTEMATIK DEGIL, ARALIKLI.
//
// ONEMLI IPUCU: lp.dist_bottom GERCEK yer yuksekligini dogru veriyor (yerde
// 0.38 m okur, sensor govdede) AMA dist_bottom_valid = 0, yani kontrolcu onu
// KULLANAMIYOR ve datum tabanli AGL'ye mahkum. Datum tabanli AGL bu kosumda
// gercekten 0.29 m'deyken 0.60 m gosterdi. Adim 117'nin datum duzeltmesi ilk
// ofseti coemdi ama sinyal hala dolayli. dist_bottom'i gecerli kilmak
// (SDF'e menzil sensoru + EKF2_RNG_* ) bu arizayi ve Adim 116-118'in tum
// sinifini kokunden kaldirabilir -- HITL oncesi en yuksek getirili is bu.

static constexpr float KP_RATE_CRUISE[3] = { 3.0f, 3.0f, 1.5f };
// WU_TILT_HOVER=3.0f (was 8.0f, 2026-07-26): at hover trim thrust
// (Tw~=18.32N) tilt is ~3.665x more effective than thrust for roll torque
// (|dtau_ddelta0|~=0.916 vs |dtau_dT0|=0.25), but the old weight made tilt
// look ~2x more *expensive per unit torque* than thrust anyway (Wu_i/|G_i|
// comparison: 8.0/0.916 ~= 8.73 vs 1.0/0.25 = 4.0). WLS then closed roll
// error by draining rotor thrust to zero instead of tilting -- a
// contributing factor in the SITL actuator lockup (see
// tiltrotor_Matlab files/sitl/RUNBOOK.md S4). 3.0 stays below the ~3.665
// break-even point (wu_thrust * dtau_ddelta0/dtau_dT0) so tilt is
// preferred for small roll/pitch corrections; see gain_schedule.m for the
// full derivation. NOTE (2026-07-26 SITL validation): this alone did NOT
// fix the lockup -- it relocated to a different rotor (T1 instead of T0)
// with the same runaway signature. Root cause still open, see RUNBOOK.md
// "Aday cozum 2": likely nu_des itself growing persistently one-signed
// (LESO d_hat drift vs hoverTrim() mismatch), not the Wu ratio itself.
static constexpr float WU_TILT_HOVER = 3.0f;
static constexpr float WU_TILT_CRUISE = 1.5f;
static constexpr float WU_TAIL_PENALTY = 3.0f;

// --- WLS weights (indi_attitude_controller.m) ---
static constexpr float WS_ROLL = 200.0f;
static constexpr float WS_PITCH = 200.0f;
// NOTE (2026-07-26): 6.0f was tried and REVERTED -- see sitl/RUNBOOK.md
// "Adim 7". Did not help yaw convergence, made Fx demand and roll worse,
// T0 still collapsed to 0 by the end of a 26s SITL test.
static constexpr float WS_YAW = 3.0f;

// LAND_YAW_FREE_ALT -- DENENDI, GERI ALINDI (2026-08-29).
// Hipotez: inis sirasinda yere yakin yaw referansini birakmak (yaw_sp =
// olculen yaw, 4 m altinda) tahsisatin kanat itki farkini buyutmesini ve
// bir motoru tabana yapistirmasini onler. Mekanizma olculmustu ve GERCEK:
// yaw RMS yere yakin 20.1x artiyor (roll 4.1x, pitch 3.1x), kilit 1.92 m'de
// basliyor, motorlar raylar arasinda isaret degistirerek salaniyor.
// AMA DUZELTME ISE YARAMADI: 3 GUI kosumunda kilitlenme 2/3 cikti, degisiklik
// yokken 1/3 idi. Yani en iyi ihtimalle etkisiz. Uc kosum istatistiksel olarak
// ayirt edici degil, ama FAYDA GOSTERILEMEDI ve ucus yazilimina risk eklemek
// icin gerekce yok. Kalan supheli: yere yakinda tahsisatin itki payi -- yaw
// TALEBI degil, TOPLAM itkinin alt payinin tukenmesi.


static constexpr float WS_FX = 0.05f;
static constexpr float WS_FZ = 20.0f;

// Outer attitude-P-loop rate setpoint saturation, PER AXIS (2026-07-27,
// step 13 -- was a single scalar 3.0f for all three axes).
// Roll and pitch can follow 3 rad/s comfortably; yaw physically cannot. Yaw
// torque on this airframe comes only from differential wing tilt, which is
// slew-rate limited (TILT_RATE_MAX), so the measured achievable yaw torque
// increment is ~0.05 Nm per step (step 12g). With the old 3.0 limit, once the
// vehicle was rotating the wrapped yaw error swept the full +-pi range, so
// omega_sp(2) kept flipping between +-3 rad/s (measured: -1.01, +1.01, +2.07,
// -3.00, +3.00 ...). Half the time that made the rate loop ACCELERATE the
// spin instead of damping it -- the outer loop was defeating the inner one.
// Keeping the yaw limit BELOW the observed spin rates (1-3.5 rad/s) makes the
// sign of the rate error follow the measured rate, so the inner loop always
// damps. 0.5 rad/s (~29 deg/s) still slews 90 deg in ~3 s.
// Mirrors p.ctrl.rate_sp_limit in tiltrotor_params.m / sf_indi_rate_law.m.
static constexpr float RATE_SP_LIMIT[3] = { 3.0f, 3.0f, 0.5f }; // rad/s, roll/pitch/yaw

// --- GRADED FAILSAFE (blocker B1, 2026-07-29 step 32) ---
//
// What this replaces. Run() used to answer EVERY degraded input the same way:
//
//     if (!ekf_ok || !att_ok || !lpos_ok) { publishDisarmed(); return; }
//
// publishDisarmed() writes NaN to all motors and servos. On the ground that is
// correct; in the air it is **cutting the motors**, which is not a failsafe --
// it converts a degraded estimate into a guaranteed crash. Worse, the condition
// is an OR of three unrelated signals with very different consequences, and one
// of them (lpos) had no freshness check at all, so a *stopped* EKF read as
// healthy forever while a *flickering* one cut power mid-flight.
//
// The key structural fact that makes graded degradation possible here:
// **the INDI rate loop needs only omega and omega_dot.** Both come from
// vehicle_angular_velocity -- the topic whose callback SCHEDULES this WorkItem.
// So if Run() is executing at all, the signal the inner loop needs is present
// by construction. vehicle_attitude feeds ONLY the outer attitude P loop, and
// vehicle_local_position feeds ONLY the altitude/position outer loops. Losing
// an outer-loop input costs you that loop, not the aircraft.
//
// Hence the levels, each dropping exactly the loop whose input is missing:
//
//   NONE       all inputs valid -- unchanged behaviour, the flown configuration
//   NO_POS     xy estimate lost -> position loop off, back-transition aborted,
//              roll/pitch to zero. Altitude hold still works.
//   NO_ALT     z/vz estimate lost -> altitude loop off, Fz held open-loop at
//              FS_FZ_OPENLOOP of hover weight. Attitude hold still works.
//              ONE behaviour, not two: a middle "z gone but vz survives, close
//              the loop on the rate" branch was removed in step 36 because EKF2
//              cannot produce that state (EKF2.cpp:1588-1590 derives z_valid and
//              v_z_valid from a single OR, under a TODO acknowledging that
//              consumers mishandle them differing). Making it reachable would
//              have meant inventing a distinction the estimator declines to
//              make -- an untested failsafe assumption, which is precisely what
//              RATE_ONLY turned out to cost.
//
// GRADING STOPS AT THE OUTER LOOPS. Attitude is a HARD PREREQUISITE, not a
// degradable input -- see the RETRACTION note below.
//
// --- RETRACTED: FsLevel::RATE_ONLY (step 35, 2026-07-30) ---
//
// There used to be a fourth level. Attitude lost -> outer attitude P loop off,
// omega_sp = 0 so the inner INDI loop becomes pure rate damping, Fz open-loop,
// descend. Its justification was written here as "it is strictly better than
// motors off". **That claim was measured and is false. The level is gone.**
//
// It was never observable in SITL: commander enters NAVIGATION_STATE_TERMINATION
// automatically ~50 ms after attitude goes invalid (modeCanRun() fails and this
// airframe has no fallback mode, because its .post script stops
// flight_mode_manager/mc_pos_control), which raises actuator_armed.lockdown, so
// terminationCommanded() below cut the output before the branch could run for
// more than one tick. Step 34 measured exactly that: 34.84 m -> 0.10 m in 2.7 s,
// i.e. free fall. So the claim was settled where there IS no commander --
// in MATLAB, run_rate_only_test.m, which reproduces this branch exactly
// (att_sp = att <=> omega_sp = 0, Fz = -m*g*FS_FZ_OPENLOOP, fx_sp = 0).
//
// Three scenarios from 35 m, all three inverted and hit the ground:
//
//   scenario            >30 deg bank   max |roll|   yaw swept   impact vz
//   clean hover          45.3 s         179.6 deg    +287 deg    11.7 m/s
//   gust + roll dist.    12.2 s         154.2 deg    +208 deg    23.7 m/s
//   entered at 10 deg     1.7 s         122.1 deg    +391 deg    19.6 m/s
//
// (time-to-30-deg is measured from the moment of degradation, not from t=0.)
//
// (free fall from 35 m for comparison: 2.67 s, 26.2 m/s.) Even the best case --
// no disturbance, entered wings-level -- rolls past 90 deg and arrives at
// 11.7 m/s. The mechanism is the one steps 29/30 already found: rate damping is
// not attitude hold. Residual torques integrate into attitude without bound;
// as the vehicle banks, the open-loop Fz stops opposing gravity; speed builds,
// wing moments engage, and the divergence accelerates.
//
// One part of the design DID work and is kept: the vertical policy. In the clean
// scenario FS_FZ_OPENLOOP = 0.97 held the vehicle to 0.71 m/s mean descent over
// 49.5 s -- it very nearly held altitude. What failed is omega_sp = 0. Open-loop
// thrust is a fine answer to "z is gone"; rate damping is not an answer to
// "attitude is gone".
//
// Consequence, stated plainly: **attitude loss now cuts the output** (see the
// hard-prerequisite check in Run()). The cost of the retraction is that a
// TRANSIENT attitude gap which would have recovered no longer gets a rate-damped
// bridge. That cost is accepted because (a) commander terminates on the same
// signal ~50 ms later regardless, so the module is agreeing with its enclosing
// authority rather than racing it, and (b) the measurement says the bridge does
// not lead anywhere: there is no control law for this airframe without attitude.
//
// Termination (NaN out) therefore has three causes, and only ONE of them is a
// human: manual_lockdown (the kill switch). actuator_armed.lockdown and
// flag_control_termination_enabled are both derived from commander's own
// automatic nav_state, and attitude loss is now a third, local cause. The
// earlier comment claiming all of them were "commanded, not inferred" was
// wrong -- see terminationCommanded() in the .cpp.
//
// NOT a staleness timeout on tiltrotor_indi_setpoint. That topic is
// event-driven by design -- `test_sp` publishes ONE message and the module
// flies it for the rest of the run (sitl/indi_sitl_common.py set_setpoint) --
// so "stale" is its normal state and a timeout there would fire on every
// existing test while detecting nothing. Control-link loss is a different
// signal and PX4 already computes it: failsafe_flags.manual_control_signal_lost
// (see the RC path, step 33).
// Sole remaining user (step 36): the CONTROL-LINK-LOSS path, where every
// estimate is still good and there is simply nobody at the sticks -- so the
// z setpoint is walked down at this rate and the normal altitude loop flies it.
// It no longer serves a degraded-estimate branch; that branch is gone.
static constexpr float FS_DESCENT_VZ = 0.7f;     // m/s, commanded descent rate on link loss
// Open-loop thrust when there is no vz to close the loop on: slightly under
// hover weight, so the vehicle sinks slowly instead of holding or climbing on
// an unknown mass. 0.97 gives ~0.29 m/s^2 net down (~1.7 m in the first 3 s).
// MEASURED and kept (step 35, run_rate_only_test.m): with attitude held level
// this gave 0.71 m/s mean descent over 49.5 s. This constant is the part of the
// old RATE_ONLY design that worked; it survives as the !alt_ok answer.
static constexpr float FS_FZ_OPENLOOP = 0.97f;   // fraction of MASS*GRAVITY
// Freshness windows. att already used 50 ms inline; lpos had NO check at all,
// which is the more dangerous half -- uORB::Subscription::copy() keeps
// returning the last sample forever, so a dead estimator looked healthy.
static constexpr uint64_t FS_ATT_TIMEOUT_US = 50000;   // 50 ms
static constexpr uint64_t FS_LPOS_TIMEOUT_US = 200000; // 200 ms (published at 30-50 Hz)

// Value 3 is deliberately NOT reused: it was RATE_ONLY, and every log, plot and
// analysis script written before step 35 reads a 3 as "attitude lost, still
// flying". Nothing should ever mean that again.
enum class FsLevel : uint8_t {
	NONE = 0,
	NO_POS = 1,
	NO_ALT = 2,
};

// --- PILOT INPUT (blocker B1, 2026-07-29 step 33) ---
//
// Before this, the only publisher of tiltrotor_indi_setpoint was the `test_sp`
// debug console command, so there was no way for a human to influence the
// vehicle in flight at all. This maps manual_control_setpoint onto the same
// internal setpoints the test path drives.
//
// Why sticks are read HERE and not through a FlightTask. The airframe's .post
// script (4023_gz_tiltrotor_indi.post) stops flight_mode_manager and
// mc_pos_control, because this module replaces the whole attitude + allocation
// + position stack. Routing the pilot through FlightTask would mean restarting
// that chain and taking trajectory_setpoint from mc_pos_control -- whose
// position law is the plain-multicopter theta_sp = -atan2(ax_b, g) that step 29
// MEASURED as wrong for this airframe (above ~5-6 m/s it commands a climb, not
// a brake). Keeping the pilot path inside the module leaves one owner of
// roll/pitch/Fx and lets the vehicle's own measured limits (POS_ENGAGE_V_MAX,
// the tilt ceiling, the back-transition states) arbitrate.
//
// Manual input is IGNORED unless vehicle_control_mode.flag_control_manual_enabled
// is set and a fresh, valid manual_control_setpoint exists. With no RC/joystick
// -- every SITL run in this report -- that topic is never published, so the
// `test_sp` path is bit-for-bit unaffected.

// Same 15 deg the position loop saturates at (POS_TILT_MAX). Not a coincidence
// and not a placeholder: that is the ONLY roll/pitch magnitude this airframe has
// ever been flown at, and a manual mode is the wrong place to exceed a limit the
// automatic loop respects.
static constexpr float MAN_TILT_MAX = POS_TILT_MAX;   // rad

// Yaw stick is a RATE command integrated into the heading setpoint, and both
// numbers are set by yaw's measured weakness rather than by feel. The rate must
// stay under RATE_SP_LIMIT[2] = 0.5 rad/s, otherwise the pilot walks the
// setpoint away faster than the vehicle can turn and the heading error grows
// without bound -- which is precisely the divergence step 13 fixed. The leash
// is the second half of the same guard: the setpoint may never lead the actual
// heading by more than this, so releasing the stick always leaves a bounded
// error no matter how long it was held.
static constexpr float MAN_YAW_RATE = 0.35f;          // rad/s (70% of RATE_SP_LIMIT[2])
static constexpr float MAN_YAW_LEASH = 0.35f;         // rad (~20 deg)

// Throttle stick is a climb-RATE command with a centre deadband, integrated
// into z_sp -- the same leash logic as yaw, for the same reason: z_sp must not
// run away from an altitude the vehicle is still climbing towards.
static constexpr float MAN_VZ_MAX = 1.5f;             // m/s (under ALT_VZ_MAX = 2.0)
static constexpr float MAN_Z_LEASH = 2.0f;            // m
static constexpr float MAN_STICK_DEAD = 0.05f;        // stick units, centre deadband

// Freshness of manual_control_setpoint. Losing it is NOT the same as a degraded
// estimate: the vehicle still knows exactly where it is, it just has nobody
// flying it -- so the response is to hold position and descend, not to degrade.
static constexpr uint64_t MAN_TIMEOUT_US = 300000;    // 300 ms

} // namespace tiltrotor_indi
