function [pitch_sp, state_out] = sf_cruise_pitch_loop(v_fwd, Fz_sp, state_in)
%SF_CRUISE_PITCH_LOOP  MATLAB Function blok icerigi: seyir pitch trim'i
%(analitik ileri besleme + model-hatasi integrali).
%#codegen
%
% cruise_pitch_loop.m'in codegen-guvenli yeniden yazimi (Adim 129).
%
% Girisler:
%   v_fwd    (m/s) govde ileri hizi
%   Fz_sp    (N)   irtifa dongusunun urettigi govde-z kuvvet komutu (FRD,
%                  pozitif = ASAGI)
%   state_in [I]   integrator (rad)
% Ciktilar:
%   pitch_sp  (rad)
%   state_out [I]

% --- SABITLER: cruise_pitch_loop.m / tiltrotor_params.m ile SENKRON ---
PITCH_ENABLE  = true;        % p.tecs.pitch_enable
PITCH_V_ON    = 13.0;        % p.tecs.pitch_v_on     (m/s)
PITCH_V_FULL  = 16.0;        % p.tecs.pitch_v_full   (m/s)
PITCH_MAX     = 6.0*pi/180;  % p.tecs.pitch_max = deg2rad(6)
PITCH_KI      = 5e-5;        % p.tecs.pitch_ki
PITCH_FZ_SP   = -12.0;       % p.tecs.pitch_fz_sp    (N)
RHO           = 1.2041;      % p.aero.rho
AREA1         = 0.5;         % p.aero.area(1)        (m^2, tek kanat yarisi)
CLA           = 4.752798721; % p.aero.cla  (1/rad, TAM deger)
A0_1          = 0.05984281113;  % p.aero.a0(1)  (rad, TAM deger)
MASS          = 5.0;         % p.m
G             = 9.81;        % p.g
TS_POS        = 1/50;        % p.Ts_pos

I = state_in(1);

if ~PITCH_ENABLE
    pitch_sp  = 0.0;
    state_out = 0.0;
    return;
end

% --- KAPI: yalnizca kanat-tasimali rejimde ---
% smoothstep, gain_schedule ile ayni bicim: ayri bir durum makinesi degil,
% surekli bir karisim. v_on altinda otorite TAM SIFIRDIR (Adim 29'un rejimi).
s = (v_fwd - PITCH_V_ON) / (PITCH_V_FULL - PITCH_V_ON);
s = max(0, min(1, s));
w = 3*s^2 - 2*s^3;

if w <= 0
    % Kapi kapali: integratoru de sifira cek ki kapi acildiginda birikmis bir
    % gecmisle baslamasin (bumpless giris).
    pitch_sp  = 0.0;
    state_out = 0.0;
    return;
end

lim = PITCH_MAX;

% --- ILERI BESLEME: ANALITIK TRIM (Adim 53) ---
% theta_ff = (W + Fz_hedef)/(qbar*S*cla) - a0.
% Hiz tabani kapinin alt ucudur: w > 0 zaten v_fwd > v_on demek, ama bolme
% guvenligi ifadenin KENDISINDE dursun (kapi ileride degisirse burasi sessizce
% sonsuza gitmesin).
v_ff    = max(v_fwd, PITCH_V_ON);
qbar_ff = 0.5 * RHO * v_ff^2;
dLdth   = qbar_ff * (2*AREA1) * CLA;          % N/rad, iki kanat yarisi
th_ff   = (MASS*G + PITCH_FZ_SP)/dLdth - A0_1;
th_ff   = max(min(th_ff, lim), -lim);

% --- INTEGRAL TRIM: yalnizca MODEL HATASI icin ---
err    = Fz_sp - PITCH_FZ_SP;
I_cand = I - PITCH_KI * err * TS_POS;

% Anti-windup: kosullu integrasyon, ve olcut TOPLAM komuttur (th_ff + I) --
% integratorun kendisi degil. Adim 53'ten once ileri besleme yoktu ve ikisi
% ayni seydi; simdi ayrilar ve dogru olcut fiilen komut edilen degerdir.
th_tot = th_ff + I_cand;
if th_tot > lim || th_tot < -lim
    pushing_out = (th_tot >  lim && err < 0) || ...
                  (th_tot < -lim && err > 0);
    if pushing_out
        I_cand = I;
    end
end
% Sert kirpma da TOPLAMA gore yazilir (I icin [-lim-th_ff, lim-th_ff]), yani
% kirpma ile kosullu integrasyon AYNI sinira bakar. Ikisi ayri sinira bakarsa
% (orn. I dogrudan +-lim'e kirpilirsa) ileri besleme ters isaretliyken sert
% kirpma once bagladigi icin toplam sinira HIC ULASAMAZ -- otorite sessizce
% |th_ff| kadar kisilir. 20 m/s'de bu 6.0 yerine 4.4 derece demekti.
I = max(min(I_cand, lim - th_ff), -lim - th_ff);

pitch_sp  = w * max(min(th_ff + I, lim), -lim);
state_out = I;

end
