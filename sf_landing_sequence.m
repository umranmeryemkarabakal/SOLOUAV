function [z_cmd, state_out] = sf_landing_sequence(enable, z, z_datum, ctz, state_in, dt)
%SF_LANDING_SEQUENCE  MATLAB Function blok icerigi: inis dizisi durum makinesi.
%#codegen
%
% TiltrotorIndiControl.hpp landingSequence()'in codegen-guvenli portu
% (Adim 160). AYNI matematik, ayni sabitler -- tek fark struct yerine literal
% sabitler ve state_in/state_out ile disaridan gelen durum (MATLAB Function
% blogu `persistent` kullanamaz).
%
% NEDEN VAR: profil (kademeli alcalma, flare, temas) PC tarafindaki
% run_mission_test.py icindeydi ve o betik hedefi POSIX KABUK ISTEMCISIYLE
% gonderiyordu -- gercek kartta olmayan bir yol (madde B0). Adim 153'te PX4
% C++'a tasindi; bu dosya onu CODEGEN yoluna da tasir, cunku HITL uretilen
% kodu kosar ve o yolda dizi olmadan otonom inis olmaz.
%
% Girisler:
%   enable  (bool)  inis dizisi bayragi
%   z       (m,NED) olculen z (asagi pozitif)
%   z_datum (m,NED) YER referansi -- disarm'da yakalanan deger
%   ctz     (N)     gerceklesen DIKEY itki toplami (temas olcutu)
%   state_in(4x1)   [durum; kademe_zamani; temas_beklemesi; z_kademe]
%   dt      (s)
% Cikislar:
%   z_cmd     (m,NED) irtifa dongusune verilecek hedef
%   state_out (4x1)
%
% DURUMLAR: 0=IDLE 1=DESCEND 2=FLARE 3=TOUCHDOWN
% MODUL DISARM ETMEZ: TOUCHDOWN yalnizca "temas olustu" der.

% --- SABITLER: TiltrotorIndiParams.hpp LAND_* ile SENKRON KALMALI ---
step_m      = 1.0;          % m,  LAND_STEP_M   (1.5 m 13 BIG_M uretmisti)
step_s      = 1.5;          % s,  LAND_STEP_S
flare_alt   = 1.5;          % m,  LAND_FLARE_ALT
touch_z     = 0.15;         % m,  LAND_TOUCH_Z (yer datumunun ALTI)
done_alt    = 0.25;         % m,  LAND_DONE_ALT
thrust_frac = 0.5;          % LAND_GROUND_THRUST_FRAC
touch_dwell_s = 1.5;        % s,  LAND_TOUCH_DWELL
mass        = 5.0;          % kg
g           = 9.81;

st          = state_in(1);
step_timer  = state_in(2);
touch_dwell = state_in(3);
z_step      = state_in(4);

if ~enable || ~isfinite(z) || ~isfinite(z_datum)
    z_cmd = z;
    state_out = [0; 0; 0; z];
    return;
end

% AGL DATUMA GORE (Adim 117): ham -z DEGIL. Olculen datum ofseti kosumdan
% kosuma -1.0 .. +0.9 m arasinda degisiyor; mutlak esik guvenilmez.
agl = z_datum - z;

% TEMAS OLCUTU IRTIFADAN BAGIMSIZ (Adim 150): dikey itki agirligin yarisinin
% altindaysa araci tutan sey zemindir. |vz| kosulu YOK -- olculdu, vz yanilan
% irtifayla ayni kestirimciden geliyor ve aracin yerde oldugu bir kosumu
% kacirdi (5.2 N itkideyken vz 0.263 okuyordu).
low_thrust = isfinite(ctz) && (ctz < thrust_frac*mass*g);
if low_thrust
    touch_dwell = touch_dwell + dt;
else
    touch_dwell = 0;
end
contact = (touch_dwell >= touch_dwell_s) || (agl < done_alt);

z_cmd = z_step;

if st == 0                                   % IDLE
    st = 1;
    z_step = z;
    step_timer = 0;
    z_cmd = z_step;

elseif st == 1                               % DESCEND
    % Surekli rampa DENENDI VE GERI ALINDI (Adim 134): inis hizini dusurdu
    % ama salinim frekansi degismedi (0.417 -> 0.419 Hz) ve pitch tepe-tepe
    % 7.96 -> 11.00 deg KOTULESTI. Salinim profilin uyarmasi degil, sistemin
    % kendi modu.
    step_timer = step_timer + dt;
    if step_timer >= step_s
        step_timer = 0;
        z_step = z_step + step_m;
    end
    if agl < flare_alt
        st = 2;
    end
    if contact
        st = 3;
    end
    z_cmd = z_step;

elseif st == 2                               % FLARE
    % Hedef YERIN ALTINA surulur: "0.30 m'de asili kal" demek, aracin yer
    % etkisinde takilmasi demekti (2026-08-28 olcumu).
    if contact
        st = 3;
    end
    z_cmd = z_datum + touch_z;

else                                         % TOUCHDOWN
    z_cmd = z_datum + touch_z;
end

state_out = [st; step_timer; touch_dwell; z_step];
end
