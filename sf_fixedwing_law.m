function [T_wing, surf_cmd, state_out] = sf_fixedwing_law( ...
        hdg_sp, z_sp, v_sp, att, omega, z, v_fwd, qbar, state_in)
%SF_FIXEDWING_LAW  MATLAB Function blok icerigi: sabit kanat modu icin
%YUZEY-merkezli otopilot (bank-to-turn).
%#codegen
%
% fixedwing_control_law.m'in codegen-guvenli yeniden yazimi (Adim 129).
% `surf_virtual_map` ve `surf_trim_offset` cagrilari ICERI ACILDI (ikisi de
% `p` struct'i alir); Mv sabit bir 5x3 matris oldugu icin literal yazildi.
%
% Kanat rotorleri tilt KILITLI (90 deg, ileri iten pervaneler), kuyruk KAPALI
% -- INDI/WLS'in rotor-tilt otoritesi varsayimi burada GECERSIZ (Adim 58).
% Bunun yerine konvansiyonel bir sabit kanat otopilotu.
%
% MIMARI (Adim 75): roll_sp SABIT 0 tutulup yaw_sp dogrudan rudder ile
% "tutulmaya" calisilmasi -- iki eksen bagimsiz -- SITL'de kararli, kendini
% surduren bir "bankali donus" dengesine (roll ~57-63 deg) yol aciyordu.
% Dogrusu bank-to-turn: heading hatasi ROLL setpoint'i uretir.
%
% Girisler:
%   hdg_sp (rad) hedef yon        z_sp (m, NED) hedef irtifa
%   v_sp   (m/s) hedef hiz        att  (3x1 rad) [phi;theta;psi]
%   omega  (3x1 rad/s)            z    (m, NED)  olculen irtifa
%   v_fwd  (m/s) govde ileri hizi qbar (Pa)      dinamik basinc
%   state_in [integral_v; integral_alt]
% Ciktilar:
%   T_wing (N) kanat rotoru basina itki, surf_cmd (5x1 rad), state_out

% --- SABITLER: fixedwing_control_law.m ile SENKRON ---
Kp_roll  = 0.35;  Kd_roll  = 0.12;
Kp_pitch = 0.35;  Kd_pitch = 0.12;
Kp_hdg   = 1.5;                 % rad roll_sp / rad heading hatasi
MAX_BANK = 30.0*pi/180;
% find_fixedwing_trim.m ile OLCULDU (V=16 m/s): denge hucum acisi 1.25 deg,
% gereken itki rotor basina 4.37 N. Eski tahminler (15 deg, 17 N) COK yanlisti
% ve irtifa/hiz dongulerinin birbirini itmesinin asil sebebi buydu.
THETA_FF = 1.25*pi/180;
T_FF     = 4.37;                % N, rotor basina
Kp_alt   = 0.06;   Ki_alt = 0.01;
PITCH_CORR_MAX = 8.0*pi/180;
Kp_v     = 1.5;    Ki_v   = 0.3;
TS       = 0.0025;              % p.Ts_ctrl
ROTOR_TMAX = 45.0;              % p.rotor.Tmax
SURF_MAX = [0.78; 0.78; 0.52; 0.52; 0.52];   % p.surf.max
RHO      = 1.225;               % p.rho
ELE_TRIM = -0.0792;             % p.surf.ele_trim

integral_v   = state_in(1);
integral_alt = state_in(2);

% z NED (asagi pozitif): irtifa kaybedildiginde z ARTAR.
% err_z = z - z_sp > 0  <=>  hedeften daha ASAGIDAYIZ  <=>  DAHA FAZLA burun
% yukari (pitch_corr > 0) gerekir. Onceki isaret (z_sp - z) TERSTI: irtifa
% kaybi sirasinda pitch_corr'u NEGATIFE itip alcalmayi BESLIYORDU (pozitif
% geri besleme, doygunlukta kaliyordu) -- Adim 58 sonrasi bulunan asil sebep.
err_z = z - z_sp;
integral_alt = max(min(integral_alt + err_z*TS, 200.0), -200.0);
pitch_corr = max(min(Kp_alt*err_z + Ki_alt*integral_alt, PITCH_CORR_MAX), -PITCH_CORR_MAX);
pitch_sp = THETA_FF + pitch_corr;

phi = att(1); theta = att(2); psi = att(3);
p_rate = omega(1); q_rate = omega(2);

% --- bank-to-turn: heading hatasi -> roll_sp ---
e_hdg = atan2(sin(hdg_sp - psi), cos(hdg_sp - psi));
roll_sp = max(min(Kp_hdg*e_hdg, MAX_BANK), -MAX_BANK);

e_roll  = roll_sp  - phi;
e_pitch = pitch_sp - theta;

% ISARETLER effectiveness_matrix.m'den: tau_x = -1.2*qbar*a_ail,
% tau_y = +0.806*qbar*a_ele. Duzeltme yonu icin e_roll > 0 (daha fazla roll_sp
% gerek) -> tau_x > 0 -> a_ail < 0.
a_ail = -(Kp_roll*e_roll   - Kd_roll*p_rate);
a_ele =  (Kp_pitch*e_pitch - Kd_pitch*q_rate);
% Rudder KALICI OLARAK KAPALI (Adim 77). Test edildi: acik/kapali sonuclar
% ozdes denecek kadar ayniydi -- pasif aerodinamik stabilite (Adim 58) +
% bank-to-turn tek basina yeterli. Bir aktuator daha az, ARI capraz-besleme
% karmasikligi tamamen kalkti.
a_rud = 0.0;

% surf_virtual_map(p) ICERI ACILDI -- sabit 5x3, sanal [a_ail;a_ele;a_rud]
% -> fiziksel [s0..s4]. Simetrik elevon Mv'nin GORUNTU UZAYINDA DEGILDIR,
% yani hicbir sekilde bir "flap" olarak kullanilamaz (Adim 45/46).
%   s0 sol elevon   = +a_ail       s1 sag elevon   = -a_ail
%   s2 sol elevator = +a_ele       s3 sag elevator = +a_ele
%   s4 rudder       = +a_rud
surf_cmd = [ a_ail; -a_ail; a_ele; a_ele; a_rud ];

% surf_trim_offset(p, qbar) ICERI ACILDI: sabit elevator trim ofseti, dusuk
% hizda sifire cekilir. Band 3 -> 8 m/s; ust uc FT_CRUISE_V (kanat orada
% tasimaya baslar), alt uc POS_ENGAGE_V_MAX (hover sayilan hiz).
q_on   = 0.5 * RHO * 3.0^2;
q_full = 0.5 * RHO * 8.0^2;
s_tr   = max(0, min(1, (qbar - q_on) / (q_full - q_on)));
w_tr   = 3*s_tr^2 - 2*s_tr^3;          % smoothstep, gain_schedule ile ayni
surf_cmd(3) = surf_cmd(3) + w_tr * ELE_TRIM;
surf_cmd(4) = surf_cmd(4) + w_tr * ELE_TRIM;

surf_cmd = max(min(surf_cmd, SURF_MAX), -SURF_MAX);

err_v = v_sp - v_fwd;
integral_v = max(min(integral_v + err_v*TS, 10.0), -10.0);
T_wing = T_FF + Kp_v*err_v + Ki_v*integral_v;
T_wing = max(min(T_wing, ROTOR_TMAX), 0);

state_out = [integral_v; integral_alt];

end
