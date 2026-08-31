function [z1, z2] = leso_axis_update(z1, z2, y_meas, u_applied, beta1, beta2, Ts)
%LESO_AXIS_UPDATE  Tek eksen icin 2. derece Lineer Genisletilmis Durum Gozlemcisi.
%
% Model (rate-loop, goreli derece 1):  omega_dot = u_applied + d(t)
%   z1 -> omega tahmini,  z2 -> toplam (kumulatif) bozucu acisal ivme tahmini d_hat
%
%   z1_dot = z2 + u_applied + beta1*(y_meas - z1)
%   z2_dot =              beta2*(y_meas - z1)
%
% beta1 = 2*wo, beta2 = wo^2  (kritik-sonumlu gozlemci kutuplari, -wo,-wo)
% wo = gozlemci bant genisligi (rad/s) — bkz. leso_bandwidth_gains.m
%
% Ts, LESO'nun kendi guncelleme periyodu (rate loop'tan daha dusuk frekansta
% calistirilabilir — sohbette onerildigi gibi CPU yukunu azaltmak icin
% 100-250 Hz araliginda tutulmasi yeterli).
%
% Bu fonksiyon "saf" (state persistent tutmaz) — z1,z2 caginin disinda
% (indi_attitude_controller icindeki ctrl_state struct'inda) saklanir, boylece
% ayni kod birden fazla paralel simulasyonda/eksende yeniden kullanilabilir.

e = y_meas - z1;

z1 = z1 + Ts*(z2 + u_applied + beta1*e);
z2 = z2 + Ts*(beta2*e);

end
