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

# INIS DIZISI (2026-08-31, Adim 153 -- madde B0). Digerleriyle AYNI KALIP:
# tek bir bayrak, diziyi MODUL yurutur. Oncesinde inis profili (kademeli
# alcalma, flare, temas) PC tarafindaki run_mission_test.py icindeydi ve o
# betik hedefi POSIX kabuk istemcisiyle gonderiyordu -- gercek kartta
# olmayan bir yol. Bayrak kalkinca modul z_sp'yi KENDI uretir; disaridan
# gelen z_sp yok sayilir.
bool land_enable

# TAM GOREV (2026-08-31, Adim 154 -- madde B0). Bu bayrak kalkinca dizici
# butun evreleri KENDI yurutur: tirmanis, ileri gecis, seyir, sabit kanat,
# geri gecis, oturma, inis. Digerleri (ft/bt/fw/land) o sirada YOK SAYILIR
# -- onlari da dizici uretir. Tek tek bayraklar elle sinama icin kalir.
bool mission_enable
