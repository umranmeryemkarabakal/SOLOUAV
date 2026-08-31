function [pitch_sp, state_out] = decel_loop(v_h, vz, state_in, p)
%DECEL_LOOP  Seyirden hover'a geri donus frenleme yasasi.
%
% !!! DENENDI VE GERI ALINDI (2026-07-29, Adim 30). KULLANILMIYOR. !!!
% Ayni fikrin korlemesine tekrar denenmemesi icin kayit olarak birakildi.
%
% NEDEN BASARISIZ OLDU (SITL'de olculdu): arac hic yavaslamadi (hiz ~27 s
% boyunca 11.7-19.2 m/s'de kaldi) ve sonunda dustu. Log sebebi gosteriyor ve
% sebep pitch yasasi DEGIL: deneme boyunca TILT'LER ILERIYE KACTI --
% tilt0 43 -> 80 deg, tilt1 36 -> 69 deg. fx_sp = 0 vermek tilt'leri geri
% cekmiyor, cunku Fx cok zayif bir WLS amaci (Ws_Fx = 0.05, ve bu BILINCLI:
% yaw'i trimleyen diferansiyel tilt ayni zamanda Fx uretir, Fx'i roll/pitch
% mertebesinde agirliklandirmak yaw'i bozar), ustelik gain schedule tilt
% buyudukce onu DAHA UCUZ yapiyor (wu_tilt 3.0 -> 1.5). Bu arada uyarlanabilir
% tavan gorevini yapip pitch komutunu ~0-4 dereceye indirdi ve geriye hic
% frenleme otoritesi kalmadi.
%
% SONUC: bir tiltrotor'da geri gecisin frenleme otoritesi TILT'tir, pitch
% attitude'u degil. Gercek bir back-transition tilt'i DOGRUDAN komut etmeli
% (bir tilt setpoint'i, ya da manevra boyunca Ws_Fx'i yukari planlamak);
% tahsisatin bunu fx_sp'den cikarmasini ummak yetmiyor. Bu bir ayar degil,
% bilincli bir tasarim degisikligi -- ve onerilen sonraki adim budur.
% O zamana kadar pos_hold, POS_ENGAGE_V_MAX ustunde sadece REDDEDIYOR.
%
% NEDEN VAR (2026-07-29, Adim 30): Adim 29'da olculdu ki bu aracin seyirden
% hover'a donmek icin GUVENLI BIR YOLU YOKTU. position_loop.m yalnizca hover
% icindir (theta_sp = -atan2(ax_b,g) duz multikopter varsayimi); 14.5 m/s'de
% devreye alindiginda pitch POS_TILT_MAX'te doydu ve arac 35 s boyunca
% tirmandi. Adim 29 buna bir KAPI koydu (POS_ENGAGE_V_MAX = 3 m/s) ama kapi
% yalnizca tehlikeyi engeller, yetenegi eklemez. Bu dosya yetenegi ekler.
%
% ARIZANIN OLCULEN MEKANIZMASI (SITL ulog, Adim 29):
%   15 deg burun yukari + 9.6 m/s'de toplam rotor itkisi 13 N'a kadar
%   dusmustu (agirlik 49.1 N, kuyruk rotoru TAMAMEN kapali) -- yani irtifa
%   dongusu itki otoritesini SONUNA KADAR kullanmisti ve arac buna ragmen
%   1.1 m/s tirmaniyordu. Aracin tamamini KANAT tasiyordu.
%   Sonuc: hizliyken buyuk burun yukari komutu bir frenleme degil, bir
%   TIRMANIS komutudur ve itkiyle telafi edilemez.
%
% TASARIM: hiza orantili burun yukari, AMA "tirmaniyorsan pitch'i kis"
% seklinde UYARLANABILIR bir tavanla sinirli.
%
%   pitch_talep = Kp_dec * v_h
%   pitch_sp    = min(pitch_talep, pitch_cap)
%   pitch_cap:  tirmanista (vz < -VZ_DEAD) asagi cekilir, aksi halde
%               yavasca PITCH_DEC_MAX'e geri toparlanir
%
% NEDEN MODEL-BAGIMSIZ (bilerek): tavani aero katsayilarindan hesaplamak
% (alpha_max = a0 + L_max/(q*A*cla)) daha "temiz" gorunur ama cla/alan
% MATLAB, Gazebo ve gercek arac arasinda FARKLI -- bu projede tam bu sinif
% (kontrolcu/plant arayuz uyusmazligi) dort kez arizaya yol acti (Adim 11,
% 12, 21, 27). vz geri beslemesi olculen davranisa tepki verir, varsayilan
% bir modele degil, ve donanima oldugu gibi aktarilabilir.
%
% !! MATLAB BU ARIZAYI YAPISAL OLARAK URETEMEZ (olculdu, Adim 30):
% MATLAB plant'i TEK bir boylamsal yuzey kullaniyor (p.aero.area = 0.5 m^2),
% 12 m/s ve 15 deg'de tasima yalnizca ~25 N -- 49 N agirligi kaldiramaz, yani
% burada kacis tirmanisi olusmaz. Gazebo modelinde BES lift-drag yuzeyi var.
% Bu yuzden bu yasa MATLAB'da DOGRULANAMAZ, yalnizca SITL'de. MATLAB'daki
% varligi senkron ve regresyon icindir. (Projenin kendi dersi, Adim 21d: bir
% ortam ancak hedeflenen mekanizma orada AKTIFSE bir sey kanitlar.)
%
% Girisler:
%   v_h      (m/s)  yatay hiz buyuklugu
%   vz       (m/s)  dikey hiz, NED (negatif = TIRMANIS)
%   state_in (rad)  pitch_cap (ilk cagrida PITCH_DEC_MAX ile baslatin)
%   p               tiltrotor_params()
% Ciktilar:
%   pitch_sp (rad)  burun yukari attitude setpoint'i (>= 0)
%   state_out(rad)  guncellenmis pitch_cap

Kp_dec       = 0.012;          % rad/(m/s), hiza orantili burun yukari
PITCH_DEC_MAX = deg2rad(8);    % rad, mutlak tavan (POS_TILT_MAX'in cok altinda)
VZ_DEAD      = 0.3;            % m/s, bu tirmanma hizina kadar tepki verilmez
CAP_DOWN     = 0.60;           % rad/s per (m/s) tirmanis, tavani kisma hizi
CAP_UP       = 0.05;           % rad/s, tavanin toparlanma hizi

pitch_cap = state_in(1);

% --- uyarlanabilir tavan: tirmaniyorsan kis, degilse yavasca ac ---
climb = max(0, -vz - VZ_DEAD);          % m/s, olu bandin uzerindeki tirmanis
if climb > 0
    pitch_cap = pitch_cap - CAP_DOWN * climb * p.Ts_pos;
else
    pitch_cap = pitch_cap + CAP_UP * p.Ts_pos;
end
pitch_cap = max(0, min(pitch_cap, PITCH_DEC_MAX));

% --- hiza orantili talep, tavanla sinirli ---
pitch_sp = min(Kp_dec * max(0, v_h), pitch_cap);

state_out = pitch_cap;

end
