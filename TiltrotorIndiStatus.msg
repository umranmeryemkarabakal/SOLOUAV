# Diagnostics for mc_indi_tiltrotor, mirrors indi_attitude_controller.m's
# `diagn` struct plus back/forward-transition and failsafe state.

uint64 timestamp

float32[3] omega_sp
float32[3] d_hat

float32[11] du
bool[11] sat_flag
float32[11] u_actual

float32[5] nu_des

float32 qbar
float32 gain_schedule_smooth
float32 avg_tilt_rad

bool ekf_healthy
bool shadow_u_actual_valid
uint8 failsafe_level

uint8 bt_state
float32 bt_tilt_ceil
float32 bt_tilt_floor
bool pos_hold_active

uint8 ft_state
float32 fz_sp
float32 cruise_fx_cmd
float32 cruise_pitch_sp
float32 cruise_v_fwd

uint8 fw_state
float32 fw_tilt

# GOLGE CODEGEN FARKI (Adim 131). MATLAB Coder'in urettigi sf_wls_alloc her
# tikte C++ wlsAllocate'in YANINDA kosar; bu alan ikisinin du(0..5) farkinin
# sonsuz-normudur. UCUSU SURMEZ -- yalnizca olcum. Adim 130 ayni karsilastirmayi
# CEVRIMDISI tekrar oynatmayla yapmisti (ort 0.042 N); bu alan onu GERCEK ucus
# dongusunde, kapali cevrim girdileriyle olcer.
float32 codegen_du_diff

# INIS DURUM MAKINESI (Adim 153): 0=IDLE 1=DESCEND 2=FLARE 3=TOUCHDOWN.
# TOUCHDOWN, "temas olustu, disarm edilebilir" demektir -- modul KENDI
# DISARM ETMEZ. Bu bilerek: PX4'un kendi mimarisinde de land_detector
# bildirir, commander karar verir. Profili moduile tasimak, arming
# yetkisini de ona vermek anlamina gelmemeli.
uint8 land_state
float32 land_z_cmd

# GOREV DIZICISI (Adim 154): 0=IDLE 1=CLIMB 2=HOVER 3=FWD 4=CRUISE
# 5=FW 6=FW_CRUISE 7=BACK 8=SETTLE 9=LAND 10=DONE
uint8 mission_state
