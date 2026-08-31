# Setpoint input for mc_indi_tiltrotor (INDI+LESO+WLS tiltrotor controller).
# Manual/scripted test helper, mirrors indi_attitude_controller.m's
# att_sp/F_sp/leso_axes_enable inputs plus the two mission-phase flags
# from forwardtrans_loop.m / backtrans_loop.m.

uint64 timestamp

float32 roll_sp
float32 pitch_sp
float32 yaw_sp
float32 fx_sp
float32 z_sp

bool leso_enable_roll
bool leso_enable_pitch
bool leso_enable_yaw

bool pos_hold_enable
bool bt_enable
bool ft_enable
bool fw_enable
