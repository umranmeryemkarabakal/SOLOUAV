#!/bin/bash
# gz joint_state (text protobuf) -> CSV
#   sim_time_s, d0,d1,d2 (tilt, rad), wr0,wr1,wr2 (rotor eklem hizi, rad/s)
#
# Adim 18: tilt eklemleri (motor_N_joint konumu) — golge modelin tilt
#          kanallariyla karsilastirmak icin.
# Adim 20: rotor eklemleri (rotor_N_joint HIZI) de eklendi. GERCEK itki:
#            w_gercek = wrN * rotorVelocitySlowdownSim   (SDF: 20)
#            T        = motorConstant * w_gercek^2       (SDF: 2e-5)
#          Bu onemli cunku yaw torkunun bir bileseni rotor reaksiyon torkudur
#          (km*T), ve yapisal bir suphe var: gz'nin timeConstantUp/Down filtresi
#          ROTOR HIZINA (w) uygulanirken PX4'un golge modeli ayni zaman sabitini
#          ITKIYE (T) uyguluyor; T = kf*w^2 oldugundan buyuk gecislerde ayrisirlar.
#
# Model.sdf yalnizca 3 eklem yayinliyorsa (Adim 18 oncesi) rotor sutunlari 0 olur.
TOPIC=/world/default/model/tiltrotor_indi_0/joint_state
gz topic -e -t "$TOPIC" 2>/dev/null | awk '
  function emit(s, ns,   i) {
      printf "%.6f", s + ns/1e9
      for (i = 0; i < 6; i++) printf ",%s", (v[i] == "" ? "0" : v[i])
      printf "\n"
  }
  /^ *sec:/  { sec  = $2 }
  /^ *nsec:/ { nsec = $2 }
  # Her mesaj "name: \"tiltrotor...\"" ile baslar. Bu satiri gorunce BIR ONCEKI
  # mesaji (kendi zaman damgasiyla) yaz, sonra tamponu sifirla. Boylece model
  # 3 ya da 6 eklem yayinlasa da dogru calisir.
  /name: "tiltrotor/ {
      if (have) emit(psec, pnsec)
      psec = sec; pnsec = nsec; have = 1; cur = -1
      for (i = 0; i < 6; i++) v[i] = ""
  }
  /name: "motor_0_joint"/ { cur = 0 }
  /name: "motor_1_joint"/ { cur = 1 }
  /name: "motor_2_joint"/ { cur = 2 }
  /name: "rotor_0_joint"/ { cur = 3 }
  /name: "rotor_1_joint"/ { cur = 4 }
  /name: "rotor_2_joint"/ { cur = 5 }
  # tilt eklemi -> KONUM, rotor eklemi -> HIZ
  /^ *position: / { if (cur >= 0 && cur < 3) { v[cur] = $2; cur = -1 } }
  /^ *velocity: / { if (cur >= 3)            { v[cur] = $2; cur = -1 } }
  { fflush() }
  END { if (have) emit(psec, pnsec) }
'
