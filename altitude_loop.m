function [Fz_sp, state_out] = altitude_loop(z_sp, z, vz, state_in, p)
%ALTITUDE_LOOP  Basit irtifa dis donguyu: P (pozisyon) + PI (hiz, anti-windup).
%
% F_sp = [Fx_sp;Fz_sp]'in Fz bileseni sabit -m*g yerine bu fonksiyondan gelir.
% Bu, hover_gust/transition testlerinde gozlenen "uzun sureli hover'da irtifa
% kaybi" sorununu cozer — nedeni, sabit Fz_sp'nin tilt kaynakli dikey itki
% kaybini (veya baska model belirsizligini) telafi edecek hicbir geri besleme
% icermemesiydi.
%
% state_in/out: [integral_vz] (1x1) — cok hafif, LESO'nun 1/8'i kadar bile
% degil (bkz. sohbet: dis donguyu eklemenin maliyeti, zaten calisan
% WLS/effectiveness-matrix yukune kiyasla ihmal edilebilir).
%
% Ts_pos ile (varsayilan 50 Hz) rate loop'tan (400 Hz) cok daha seyrek
% cagirilmasi onerilir; ama fonksiyonun kendisi her cagrida integral'i
% Ts_pos kadar ilerletir (caller decimasyonu yonetir, bkz. run_*.m).

vz_max = 2.0;      % m/s, tirmanma/inis hiz limiti
Kp_z   = 0.6;       % 1/s
Kp_vz  = 4.0;        % (m/s^2)/(m/s)
Ki_vz  = 1.5;        % (m/s^2)/(m/s * s)
int_max = 3.0;       % m/s^2, anti-windup clamp

integral_vz = state_in(1);

err_z  = z_sp - z;
vz_sp  = max(min(Kp_z*err_z, vz_max), -vz_max);
err_vz = vz_sp - vz;

integral_vz = max(min(integral_vz + err_vz*p.Ts_pos, int_max), -int_max);

az_corr = Kp_vz*err_vz + Ki_vz*integral_vz;   % NED-z ivme duzeltmesi (m/s^2)
% GUVENLIK KELEPCESI (2026-08-16, PX4 C++ portundan geri taşındı --
% TiltrotorIndiParams.hpp ALT_FZ_MIN/ALT_FZ_MAX_CLAMP, Adim 57). Proportional
% terim err_vz uzerinden SINIRSIZDIR (vz_sp kirpili ama olculen vz degil);
% dikey kanal otoritesi kaybolup vz sapinca Fz_sp de sinirsizca patliyordu.
Fz_sp = p.m*(az_corr - p.g);
Fz_sp = max(min(Fz_sp, 20.0), -110.0);

state_out = integral_vz;

end
