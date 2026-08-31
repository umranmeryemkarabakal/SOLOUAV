function [Fz_sp, state_out] = sf_altitude_loop(z_sp, z, vz, state_in)
%SF_ALTITUDE_LOOP  MATLAB Function blok icerigi: basit irtifa dis dongusu
%(P pozisyon + PI hiz, anti-windup) — altitude_loop.m ile ayni mantik,
%persistent yerine SAF fonksiyon + disaridan Unit Delay durum geri beslemesi
%(bkz. sf_indi_rate_law.m'deki ayni desen).
%#codegen
%
% state_in/out (1x1): integral_vz
%
% NOT: pure-MATLAB run_*.m betiklerinde bu dongu Ts_pos=1/50s'de decimasyonlu
% cagrilir (gercekci coklu-hiz mimarisi icin); burada MATLAB Function blogunun
% ayrik ornekleme-zamani ozelligini programatik ayarlamanin guvenilir bir yolu
% bulunamadigi icin (bkz. proje notlari — sf_indi_rate_law'daki persistent
% sorunuyla ayni kok neden), blok her rate-loop adiminda (Ts_ctrl=400 Hz)
% cagriliyor ve Ts asagida ona gore kucultuldu. Matematiksel olarak esdeger
% (integral, ornekleme hizindan bagimsizdir) — sadece CPU'da (ihmal edilebilir
% olsa da) 8 kat daha sik calisir.

m = 5.0; g = 9.81;         % tiltrotor_params.m: p.m, p.g

vz_max = 2.0;
Kp_z   = 0.6;
Kp_vz  = 4.0;
Ki_vz  = 1.5;
int_max = 3.0;

integral_vz = state_in(1);

err_z  = z_sp - z;
vz_sp  = max(min(Kp_z*err_z, vz_max), -vz_max);
err_vz = vz_sp - vz;

Ts_pos = 0.0025; % s, 400 Hz (bkz. yukaridaki not — Ts_ctrl ile ayni cagri hizi)
integral_vz = max(min(integral_vz + err_vz*Ts_pos, int_max), -int_max);

az_corr = Kp_vz*err_vz + Ki_vz*integral_vz;
Fz_sp = m*(az_corr - g);

state_out = integral_vz;

end
