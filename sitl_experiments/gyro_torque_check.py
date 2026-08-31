#!/usr/bin/env python3
"""
Adim 94: dönen rotorların HIZLA tilt edilmesinden doğan jiroskopik presesyon
tepki torkunu, gerçek yakalanan joint_state verisinden hesaplar ve itki-kaynakli
(pasif olmayan, "aktif") momentle oranlar. Hicbir plant/WLS modelinde bu terim
yok -- kontrol edelim ne kadar buyuk.

tau_gyro_i (govdeye tepki) = - I_spin_i * w_i * ddelta_i * [cos(delta_i), 0, sin(delta_i)]
(dir_i = [sin(delta),0,-cos(delta)], d(dir_i)/dt = ddelta*[cos(delta),0,sin(delta)],
 rotor spin ekseni Y etrafinda tilt ediliyor -- yalnizca KANAT rotorleri, 0 ve 1,
 kuyruk rotoru (2) de ayni sekilde ama farkli SDF konumunda.)
"""
import numpy as np
import pickle

with open('/tmp/claude-1000/-home-omer/2ebc1a5b-f0c9-4937-8214-37f5d48fdea3/scratchpad/joint_state_parsed.pkl', 'rb') as f:
    js = pickle.load(f)

I_SPIN = 0.000167604  # kg*m^2, rotor izz (spin axis), same for all 3 (identical SDF blocks)
SLOWDOWN = 20.0
ROTOR_KF = 2.0e-5

t = js['t']
dt = np.gradient(t)

results = {}
for i in [0,1,2]:
    delta = js[f'motor_{i}_joint_pos']
    w = js[f'rotor_{i}_joint_vel'] * SLOWDOWN
    ddelta = np.gradient(delta) / dt
    # smooth ddelta lightly -- raw gradient of a position signal at this log
    # rate is noisy, same issue as Adim 93's omega differentiation
    kernel = np.ones(9)/9
    ddelta_s = np.convolve(ddelta, kernel, mode='same')

    tau_gyro_x = -I_SPIN * w * ddelta_s * np.cos(delta)
    tau_gyro_z = -I_SPIN * w * ddelta_s * np.sin(delta)
    results[i] = (tau_gyro_x, tau_gyro_z, ddelta_s, w, delta)

tau_gyro_x_total = sum(results[i][0] for i in [0,1,2])
tau_gyro_z_total = sum(results[i][1] for i in [0,1,2])  # yaw component

mask = (t >= 30) & (t <= 90)
print("Jiroskopik tepki torku (govdeye), 30-90s penceresi:")
print(f"  roll bileseni (X): RMS={np.sqrt(np.mean(tau_gyro_x_total[mask]**2)):.4f} Nm, "
      f"peak={np.max(np.abs(tau_gyro_x_total[mask])):.4f} Nm")
print(f"  yaw bileseni  (Z): RMS={np.sqrt(np.mean(tau_gyro_z_total[mask]**2)):.4f} Nm, "
      f"peak={np.max(np.abs(tau_gyro_z_total[mask])):.4f} Nm")

for i in [0,1,2]:
    tgx, tgz, dds, w, delta = results[i]
    print(f"  rotor {i}: ddelta RMS={np.sqrt(np.mean(dds[mask]**2)):.3f} rad/s "
          f"(max {np.max(np.abs(dds[mask])):.3f}), w RMS={np.sqrt(np.mean(w[mask]**2)):.1f} rad/s")

# --- karsilastirma: itki-kaynakli (Adim 93b'nin tau_pred, pitch) ile ---
# Adim 93b'nin urettigi buyuklukler: RMS qdot_pred(smoothed pitch)=1.87 rad/s^2 ->
# tau (pitch) = Iyy*qdot = 0.25*1.87 = 0.4675 Nm RMS (yaklasik, referans icin)
tau_active_pitch_rms_approx = 0.25 * 1.872
print(f"\nKiyasla: Adim 93b'nin itki-kaynakli PITCH momenti (yaklasik) RMS ~= {tau_active_pitch_rms_approx:.3f} Nm")
print(f"Jiroskopik YAW torku / itki-kaynakli PITCH momenti orani (RMS) = "
      f"{np.sqrt(np.mean(tau_gyro_z_total[mask]**2))/tau_active_pitch_rms_approx:.2f}")

# --- ikinci pasif etki: tilt eklemini HIZLA dondurmenin reaksiyon torku ---
# (motor+rotor kutlesinin kendi acisal ivmesi -- jiroskopik presesyon DEGIL,
# duz D'Alembert reaksiyonu -- ama AYNI SEKILDE hicbir modelde yok)
I_TILT = 0.0166704 + 0.000166704  # motor.iyy + rotor.iyy, Y ekseni (tilt ekseni)
print(f"\nI_TILT (motor+rotor, tilt/pitch ekseni) = {I_TILT:.5f} kg*m^2")
for i in [0,1]:  # sadece kanat rotorleri (tail rotor farkli konumda, ayni analiz)
    ddelta_s = results[i][2]
    ddelta_dot = np.gradient(ddelta_s) / dt
    kernel = np.ones(9)/9
    ddelta_dot_s = np.convolve(ddelta_dot, kernel, mode='same')
    tau_tilt_reaction = I_TILT * ddelta_dot_s
    rms = np.sqrt(np.mean(tau_tilt_reaction[mask]**2))
    peak = np.max(np.abs(tau_tilt_reaction[mask]))
    print(f"  rotor {i} tilt-reaksiyon torku (pitch ekseni): RMS={rms:.4f} Nm, peak={peak:.4f} Nm, "
          f"aktif-momente oran(RMS)={rms/tau_active_pitch_rms_approx:.3f}")
