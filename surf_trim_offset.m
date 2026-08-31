function s0 = surf_trim_offset(p, qbar)
%SURF_TRIM_OFFSET  Fiziksel yuzeylerin trim ofseti (rad, 5x1).
%
%   surf_cmd = surf_virtual_map(p) * a_virt + surf_trim_offset(p, qbar)
%
% Adim 52 (2026-08-04). Su an yalnizca ELEVATOR ofseti var; gerekce ve turetme
% tiltrotor_params.m'de (p.surf.ele_trim). Ozet: kuyruk panelleri SDF'te
% a0 = -0.2 ile kurulu ve hiz karesiyle buyuyen kalici bir ASAGI kuvvet
% uretiyorlar (25 m/s'de 34 N = agirligin %70'i). Kontrolcu bu kuvveti
% GORMEZ, dolayisiyla tahsisat onu iptal etmeyi hic denemez ve yuk sessizce
% ROTORLARA kalir.
%
% NEDEN AYRI BIR FONKSIYON: ofset iki yerde birden gerekiyor ve ikisinin
% AYNI olmasi sart --
%   (1) komut uretilirken  : surf_cmd = Mv*a + s0
%   (2) olcumden sanal koordinata geri projeksiyon yapilirken : a = pinv(Mv)*(s - s0)
% Ikisi ayrisirsa kontrolcu kendi trimini bir hata sanip surekli kovalar. Bu
% projede ayni sinif bir ayrisma (kontrolcu modeli ile plant fizigi arasinda)
% en pahali hatalardi (Adim 11/21/27), o yuzden tek kaynak.

% HIZ KAPISI -- ve bunu bir REGRESYON TESTI zorunlu kildi (Adim 52).
% Ofset ilk yazimda kosulsuzdu; run_surface_effectiveness_test'in E2 denetimi
% ("hover'da yuzey komutu TAM sifir") o yuzden DUSTU: hover'da elevator
% -4.54 deg'de duruyordu. Aerodinamik olarak zararsizdi (qbar ~ 0) ama
% NOTRLUK GARANTISI bu projede bir sozdur -- hover davranisinin bit duzeyinde
% degismedigini soyler ve tam da o yuzden her degisiklikte sinanir.
%
% Kapi FIZIKSEL OLARAK BEDAVADIR: iptal ettigi kuvvet de, iptal eden kuvvet de
% qbar ile olceklenir, yani oranlari sabittir; qbar = 0'da ikisi de sifirdir ve
% ofsetin degeri onemsizdir. Dolayisiyla dusuk hizda sifire cekmek hicbir sey
% kaybettirmez, yalnizca garantiyi korur.
% Band 3 -> 8 m/s: ust ucu FT_CRUISE_V ile ayni (kanat orada tasimaya baslar),
% alt ucu POS_ENGAGE_V_MAX ile ayni (hover sayilan hiz).
if nargin < 2 || isempty(qbar)
    qbar = 0.0;
end
q_on   = 0.5 * p.rho * 3.0^2;
q_full = 0.5 * p.rho * 8.0^2;
s = max(0, min(1, (qbar - q_on) / (q_full - q_on)));
w = 3*s^2 - 2*s^3;                 % smoothstep, gain_schedule.m ile ayni bicim

s0 = zeros(p.surf.n, 1);
s0(3) = w * p.surf.ele_trim;   % sol elevator
s0(4) = w * p.surf.ele_trim;   % sag elevator

end
