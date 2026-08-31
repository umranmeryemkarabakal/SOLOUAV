function [z_cmd, state_out] = landing_sequence(enable, z, z_datum, ctz, state_in, p)
%LANDING_SEQUENCE  Inis dizisi durum makinesi (MATLAB REFERANSI).
%
% Uc uygulamanin referans ayagi. Digerleri:
%   codegen : sf_landing_sequence.m
%   PX4 C++ : TiltrotorIndiControl.hpp landingSequence()
% Ucu de AYNI matematigi tasimali (safe-control-change).
%
% NEDEN VAR (Adim 153/160): profil (kademeli alcalma, flare, temas) PC
% tarafindaki run_mission_test.py icindeydi ve o betik hedefi POSIX KABUK
% ISTEMCISIYLE gonderiyordu -- gercek kartta olmayan bir yol (madde B0).
%
% DURUMLAR: 0=IDLE 1=DESCEND 2=FLARE 3=TOUCHDOWN
% DISARM ETMEZ: TOUCHDOWN yalnizca "temas olustu" der; karar disaridadir.
% Gerekcelerin tamami sf_landing_sequence.m ve TiltrotorIndiParams.hpp'de.

st          = state_in(1);
step_timer  = state_in(2);
touch_dwell = state_in(3);
z_step      = state_in(4);

if ~enable || ~isfinite(z) || ~isfinite(z_datum)
    z_cmd = z;  state_out = [0; 0; 0; z];  return;
end

agl = z_datum - z;                       % DATUMA GORE, ham -z DEGIL (Adim 117)

% Temas olcutu IRTIFADAN BAGIMSIZ (Adim 150): |vz| kosulu YOK.
low_thrust = isfinite(ctz) && (ctz < p.ctrl.land_ground_thrust_frac * p.m * p.g);
if low_thrust
    touch_dwell = touch_dwell + p.Ts_ctrl;
else
    touch_dwell = 0;
end
contact = (touch_dwell >= p.ctrl.land_touch_dwell) || (agl < p.ctrl.land_done_alt);

z_cmd = z_step;

if st == 0
    st = 1;  z_step = z;  step_timer = 0;  z_cmd = z_step;
elseif st == 1
    step_timer = step_timer + p.Ts_ctrl;
    if step_timer >= p.ctrl.land_step_s
        step_timer = 0;  z_step = z_step + p.ctrl.land_step_m;
    end
    if agl < p.ctrl.land_flare_alt; st = 2; end
    if contact; st = 3; end
    z_cmd = z_step;
elseif st == 2
    if contact; st = 3; end
    z_cmd = z_datum + p.ctrl.land_touch_z;
else
    z_cmd = z_datum + p.ctrl.land_touch_z;
end

state_out = [st; step_timer; touch_dwell; z_step];
end
