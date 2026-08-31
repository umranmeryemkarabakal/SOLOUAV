function xdot = tiltrotor_plant_deriv(x, T_cmd, delta_cmd, p, wind_ned, ext_moment, surf_cmd)
%TILTROTOR_PLANT_DERIV  3 tilt-rotorlu VTOL icin tam nonlineer 6-DOF + aktuator
%gecikmesi + basit boylamsal aero "bozucu" plant modeli.
%
% Durum x (19x1, +5 surf istege bagli, +3 tilt-reaksiyon istege bagli):
%   x(1:3)   pos_ned      [x;y;z]        (m, NED)
%   x(4:6)   vel_body     [u;v;w]        (m/s, govde eksen)
%   x(7:10)  quat         [q0;q1;q2;q3]  (body->earth, Hamilton)
%   x(11:13) omega_body   [p;q;r]        (rad/s)
%   x(14:16) T_actual     [T0;T1;T2]     (N)   — rotor itkisi, 1. derece gecikmeli
%   x(17:19) delta_actual [d0;d1;d2]     (rad) — tilt servo, 1. derece gecikmeli
%   x(20:24) surf_actual  [s0..s4]       (rad) — KONTROL YUZEYLERI (istege bagli,
%                                         bkz. asagisi), 1. derece + hiz limitli
%   x(son-2:son) ddelta_filt [f0;f1;f2] (rad/s) — tilt HIZININ alcak-gecirgen
%                                         suzulmus kopyasi (istege bagli, Adim 96),
%                                         reaksiyon torku icin ivme tahmincisi
%
% Girisler:
%   T_cmd, delta_cmd   kontrolcuden gelen aktuator komutlari (lag'in hedefi)
%   wind_ned  (3x1)    ruzgar hizi (NED) — LESO/INDI testi icin disturbance kaynagi
%   ext_moment (3x1)   ek dis moment (Nm, govde ekseninde) — gust/darbe testleri icin
%   surf_cmd  (5x1)    ISTEGE BAGLI kontrol yuzeyi komutu (rad)
%
% GERIYE UYUMLULUK (Adim 46, Adim 96): x 19 elemanli verilirse yuzeyler VE
% tilt-reaksiyon yoktur, xdot da 19 elemanlidir — yuzeyleri/reaksiyonu
% kullanmayan tum mevcut cagiricilar (run_hover_gust_test, run_yaw_step_test,
% run_poshold_climb_test, ...) DEGISMEDEN calisir. x 24 elemanli -> +yuzeyler.
% x 22 (yuzeysiz) veya 27 (yuzeyli) elemanli -> +tilt-reaksiyon (Adim 96).
%
% ADIM 96 -- TILT-REAKSIYON TORKU (D'Alembert, Adim 94'un SITL'de olculdugu,
% hicbir modelde bulunmayan terimi): motor+rotor montaji HIZLA tilt edilirken
% (delta_dot degisirken), bu kutleyi acisal olarak ivmelendirmenin Newton'un
% 3. yasasi geregi govdeye binen tepki torku -M_TILT*delta_dotdot (motor_N_joint
% ekseni Y = pitch, ucu rotor de ayni eksende -- SDF'den dogrulandi). delta_dotdot
% dogrudan (algebraik ddelta'nin turevi) tanimsiz/impuls-benzeri oldugundan
% (ddelta, delta_cmd her kontrol tick'inde SIcRAdikca sicramali degisiyor),
% GERCEKCI, nedensel bir ivme tahmini icin ddelta'nin kendisi kucuk bir
% zaman sabitiyle (TAU_DIFF) alcak-gecirgen suzuluyor (3 yeni durum) ve
% delta_dotdot = (ddelta - ddelta_filt)/TAU_DIFF olarak tahmin ediliyor --
% gercek, sonlu-bantgenislikli bir sensorun/sistemin yapacagi gibi.
M_TILT   = 0.0166704 + 0.0001667;  % kg*m^2, motor.iyy+rotor.iyy (SDF, Y ekseni)
% TAU_DIFF 0.01 -> 0.004 (2026-08-25, Adim 96b): 0.01s, SITL'in gercek 4ms
% kontrol tick'inden 2.5x YAVAS -- ddelta'nin tick-basi isaret degistirmesini
% (Adim 90-94'un imzasi) yeterince keskin yakalayamiyor, ilk deneme (Adim 96)
% SITL'in siddetini yeniden uretemedi. TS_BOX (=1/250) ile esitlendi.
TAU_DIFF = 0.004;                  % s, turev-tahmin filtresi zaman sabiti

pos   = x(1:3);
vel_b = x(4:6);
q     = x(7:10);  q = q/norm(q);
omega = x(11:13);
T     = x(14:16);
delta = x(17:19);

has_surf = numel(x) >= 19 + p.surf.n;
if has_surf
    surf = x(20:19+p.surf.n);
    if nargin < 7 || isempty(surf_cmd)
        surf_cmd = zeros(p.surf.n,1);
    end
else
    surf = zeros(p.surf.n,1);
end

n_before_reaction = 19 + has_surf*p.surf.n;
has_reaction = numel(x) >= n_before_reaction + 3;
if has_reaction
    ddelta_filt = x(n_before_reaction+1 : n_before_reaction+3);
else
    ddelta_filt = [];  % asagida hesaplanmayacak, M_reaction=0 kalacak
end

R_eb = quat_to_dcm(q);   % v_earth = R_eb*v_body

%% --- Aktuator dinamigi (1. derece gecikme) ---
Tdot = zeros(3,1);
for i = 1:3
    if T_cmd(i) >= T(i)
        tau = p.rotor.tau_up;
    else
        tau = p.rotor.tau_down;
    end
    Tdot(i) = (T_cmd(i) - T(i)) / tau;
end
ddelta_raw = (delta_cmd - delta) / p.tilt.tau;
ddelta = max(min(ddelta_raw, p.tilt.rate_max), -p.tilt.rate_max);

%% --- Tilt-reaksiyon torku (Adim 96, istege bagli) ---
if has_reaction
    ddelta_dot_est = (ddelta - ddelta_filt) / TAU_DIFF;
    ddelta_filt_dot = ddelta_dot_est;  % = xdot(ddelta_filt), tanimca
    % Ucu rotorun de tilt ekseni Y (pitch) -- SDF'den dogrulandi (motor_{0,1,2}_joint
    % hepsi <axis><xyz>0 1 0</xyz>). Newton'un 3. yasasi: montaji ivmelendirmenin
    % govdeye reaksiyonu -M_TILT*delta_dotdot, Y ekseninde (pitch).
    M_reaction = [0; -M_TILT * sum(ddelta_dot_est); 0];
else
    ddelta_filt_dot = [];
    M_reaction = zeros(3,1);
end

%% --- Rotor kuvvet/moment (tam nonlineer geometri) ---
F_thrust = zeros(3,1);
M_thrust = zeros(3,1);
for i = 1:3
    s = sin(delta(i)); c = cos(delta(i));
    dir_i = [s; 0; -c];
    f_i = T(i) * dir_i;
    m_i = p.rotor.km(i) * T(i) * dir_i;
    F_thrust = F_thrust + f_i;
    M_thrust = M_thrust + cross(p.rotor.pos(:,i), f_i) + m_i;
end

%% --- Aero: bes LiftDrag paneli (kontrolcuden gizli, "bozucu" olarak) ---
% Adim 46: eski tek-panelli, TERS ISARETLI, sabit-Cd'li yaklasim yerine gz'nin
% gercek yasasi. Turetme ve olculen hata icin bkz. aero_panels.m.
wind_b = R_eb' * wind_ned;
v_rel  = vel_b - wind_b;

[F_aero, M_aero] = aero_panels(v_rel, omega, surf, p);

F_body = F_thrust + F_aero + R_eb' * [0; 0; p.m*p.g];
M_body = M_thrust + M_aero + M_reaction + ext_moment(:);

veldot   = F_body/p.m - cross(omega, vel_b);
omegadot = p.Iinv * (M_body - cross(omega, p.I*omega));
qdot     = quat_deriv(q, omega);
posdot   = R_eb * vel_b;

xdot = [posdot; veldot; qdot; omegadot; Tdot; ddelta];

%% --- Kontrol yuzeyi servolari (yalnizca genisletilmis durumda) ---
% Tilt servolariyla ayni yapi: 1. derece gecikme + sert hiz limiti.
if has_surf
    dsurf_raw = (surf_cmd(:) - surf) / p.surf.tau;
    dsurf = max(min(dsurf_raw, p.surf.rate_max), -p.surf.rate_max);
    % Eklem limiti: gz'de JointPositionController hedefi eklem sinirinda
    % durdurulur, yani limitin OTESINE giden hareket fiziksel olarak olmaz.
    smax = p.surf.max(:);
    dsurf(surf >=  smax & dsurf > 0) = 0;
    dsurf(surf <= -smax & dsurf < 0) = 0;
    xdot = [xdot; dsurf];
end

%% --- Tilt-reaksiyon filtre durumu (yalnizca genisletilmis durumda, Adim 96) ---
if has_reaction
    xdot = [xdot; ddelta_filt_dot];
end

end
