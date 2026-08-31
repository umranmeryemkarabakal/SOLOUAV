function [T_wing, surf_cmd, state_out] = fixedwing_control_law( ...
    hdg_sp, z_sp, v_sp, att, omega, z, vz, v_fwd, qbar, state_in, p)
%FIXEDWING_CONTROL_LAW  Sabit kanat modu icin surface-merkezli otopilot.
%
% Kanat rotorleri tilt KILITLI (90 deg, ileri iten pervaneler), kuyruk
% KAPALI -- INDI/WLS'in rotor-tilt otoritesi varsayimi burada gecersiz
% (Adim 58). Bunun yerine KONVANSIYONEL bir sabit kanat otopilotu.
%
% MIMARI DUZELTMESI (Adim 75, 2026-08-22): PX4 C++ portunda (Adim 66-74)
% roll_sp'yi SABIT 0'da tutup yaw_sp'yi dogrudan rudder ile "tutmaya"
% calismak -- iki eksen birbirinden BAGIMSIZ -- SITL'de kararli, kendini
% surduren bir "bankali donus" dengesine (roll ~57-63 deg, yaw surekli
% donuyor) yol acti. Dort farkli tek-eksenli duzeltme (ARI, agresif D,
% roll/yaw integralleri) denendi -- ikisi (agresif D, roll integrali)
% ACIKCA TEHLIKELI sonuclar uretti (araci ters donmus/baş asagi bir dengeye
% firlatti). Kok neden: bu, gercek sabit kanatli ucaklarin hepsinde var olan
% NORMAL bir lateral-directional kenetlenme (adverse yaw: aileron sapmasi
% kendi basina fazladan surtunmeden dolayi ters yonde yaw uretir) --
% "simetrik frame" olmasi bunu ORTADAN KALDIRMAZ, cunku bu asimetriden degil
% AERODINAMIKTEN kaynaklanir. Gercek sabit kanat otopilotlari heading'i
% RUDDER ile degil BANKA ACISIYLA (bank-to-turn) kontrol eder; rudder'in
% tek isi roll'e ESLIK ETMEK (donus koordinasyonu) ve yaw rate'i SONUMLEMEK
% (yaw damper) -- heading hatasina DOGRUDAN tepki vermez.
%
%   heading hatasi -> roll_sp (banka acisi komutu, sinirlanmis)
%   roll  <- aileron (PD, roll_sp'yi takip eder -- artik sabit 0 DEGIL)
%   pitch <- elevator (PD, theta + q), theta_sp irtifa hatasindan turer
%   yaw   <- rudder: SADECE sonum (Kd*r) + donus koordinasyonu (K_ARI*a_ail)
%            -- heading hatasina ARTIK DOGRUDAN TEPKI VERMEZ
%   hiz   <- iki kanat rotorunun ORTAK itkisi (P, v_fwd hatasi)
%
% hdg_sp = hedef yon (rad) -- eskiden [roll_sp;yaw_sp] olan att_sp_rp'nin
% yerini aldi, cunku roll_sp artik DISARIDAN komut edilen bir sey degil,
% BU FONKSIYONUN KENDI URETTIGI bir ic degisken (bank-to-turn).
% state_in/out: [integral_v; integral_alt] (hiz P+I ve irtifa PI icin)

Kp_roll = 0.35;  Kd_roll = 0.12;
Kp_pitch = 0.35; Kd_pitch = 0.12;
Kd_yaw = 0.08;   % SADECE sonum -- P terimi YOK (Adim 75, yukarida aciklandi)

% Heading -> banka acisi (bank-to-turn). Kp_hdg ve MAX_BANK gercek hafif
% sabit-kanat otopilotlarinin tipik degerleridir (orn. ArduPilot NAVL1,
% PX4 fw_pos_control varsayilanlari benzer mertebede) -- ilk deneme, MATLAB'da
% dogrulanip ayarlanacak.
Kp_hdg = 1.5;              % rad roll-sp per rad heading hatasi
MAX_BANK = deg2rad(30);    % banka aci siniri

% Donus koordinasyonu: aileron sapmasinin urettigi ters-yaw'i iptal eder.
% Isaret PX4 tarafinda (Adim 71) dogrulandi: a_ail>0 sustained ile r>0
% sustained birlikte gozlemlendi, yani ayni isaretli ekleme doguru yonde.
K_ARI = 0.5;

% find_fixedwing_trim.m ile OLCULDU (V=16 m/s): denge hucum acisi 1.25 deg,
% gereken itki rotor basina 4.37 N -- eski tahminler (15 deg, 17N) COK
% yanlisti, irtifa/hiz dongulerinin birbirini itmesinin asil sebebi buydu.
THETA_FF = deg2rad(1.25);    % feedforward trim (olculmus, V_SP=16 icin)
T_FF     = 4.37;             % N, rotor basina feedforward itki (olculmus)

Kp_alt = 0.06;                % irtifa hatasi -> pitch_sp DUZELTMESI (ff'in USTUNE)
Ki_alt = 0.01;                 % irtifa hizi (vz) uzerinden kucuk integral, kalici hatayi kapatir
pitch_corr_max = deg2rad(8); % duzeltme payi sinirlai (ff ayri, sinirsiz degil ama kucuk)

Kp_v = 1.5;   % N per (m/s)
Ki_v = 0.3;   % N per (m/s * s)
Ts = p.Ts_ctrl;

integral_v   = state_in(1);
integral_alt = state_in(2);

% z NED (asagi pozitif): irtifa kaybedildiginde (asagi sarktiginda) z ARTAR.
% err_z = z - z_sp > 0  <=>  hedeften daha ASAGIDAYIZ  <=>  DAHA FAZLA burun
% yukari (pitch_corr > 0) gerekir. Onceki isaret (z_sp - z) TERSTI: irtifa
% kaybi sirasinda pitch_corr'u NEGATIFE itip alcalmayi besliyordu (pozitif
% geri besleme, doygunlukta kaliyordu) -- Adim 58 sonrasi bulunan asil sebep.
err_z = z - z_sp;
integral_alt = max(min(integral_alt + err_z*Ts, 200.0), -200.0);
pitch_corr = max(min(Kp_alt*err_z + Ki_alt*integral_alt, pitch_corr_max), -pitch_corr_max);
pitch_sp = THETA_FF + pitch_corr;

phi = att(1); theta = att(2); psi = att(3);
p_rate = omega(1); q_rate = omega(2); r_rate = omega(3);

% --- bank-to-turn: heading hatasi -> roll_sp ---
e_hdg = atan2(sin(hdg_sp - psi), cos(hdg_sp - psi));
roll_sp = max(min(Kp_hdg*e_hdg, MAX_BANK), -MAX_BANK);

e_roll  = roll_sp  - phi;
e_pitch = pitch_sp - theta;

% ISARETLER effectiveness_matrix.m'den: tau_x=-1.2*qbar*a_ail,
% tau_y=+0.806*qbar*a_ele, tau_z=-0.142*qbar*a_rud. Duzeltme yonu icin
% e_roll>0 (daha fazla roll_sp gerek) -> tau_x>0 -> a_ail<0.
a_ail = -(Kp_roll*e_roll  - Kd_roll*p_rate);
a_ele =  (Kp_pitch*e_pitch - Kd_pitch*q_rate);
% Rudder KALICI OLARAK KAPALI (Adim 77). Test edildi: acik/kapali sonuclar
% ozdes denecek kadar aynıydı (pitch/roll/heading-yakalama farkı yok) --
% pasif aerodinamik stabilite (Adim 58: kontrolsuz birakildiginda pitch
% 13.9, roll 8.3 deg) + roll/banka-ile-donus (bank-to-turn) kontrolu tek
% basina yeterli. Basitlestirme: bir aktuator daha az, ARI capraz-besleme
% karmasikligi tamamen kalkti. Eski satir (Adim 75): a_rud = Kd_yaw*r_rate + K_ARI*a_ail;
a_rud = 0;

a_virt = [a_ail; a_ele; a_rud];
Mv = surf_virtual_map(p);
surf_cmd = Mv * a_virt + surf_trim_offset(p, qbar);
surf_cmd = max(min(surf_cmd, p.surf.max(:)), -p.surf.max(:));

err_v = v_sp - v_fwd;
integral_v = max(min(integral_v + err_v*Ts, 10.0), -10.0);
T_wing = T_FF + Kp_v*err_v + Ki_v*integral_v;
T_wing = max(min(T_wing, p.rotor.Tmax), 0);

state_out = [integral_v; integral_alt];

end
