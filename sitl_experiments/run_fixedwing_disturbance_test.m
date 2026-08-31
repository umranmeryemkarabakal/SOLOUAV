%% RUN_FIXEDWING_DISTURBANCE_TEST (Adim 79)
% SITL'de goruilen yavas, hizlanan roll/yaw sapmasinin nedenini ayirt eder:
% (a) kalici, kucuk bir bozucu tork (item-N benzeri) mi -- boyle olsaydi
%     P-only bank-to-turn dongusu SABIT (buyumeyen) bir denge bankasinda
%     kalirdi, cunku bir P dongusu sabit bir bozucuya karsi sabit bir kalici
%     hatada durur, BUYUMEZ; ya da
% (b) Kp_hdg'nin gercek roll tepki gecikmesine gore fazla agresif olup
%     dongunun kendisinin KARARSIZ olmasi mi -- boyle olsaydi KUCUK bir
%     bozucu bile USTEL sekilde buyuyen bir sapma baslatirdi (SITL'de
%     gozlenen desen: oranlar 1.9, 1.7, 2.8, 4.0 -- hizlanan).
%
% Kucuk, KALICI bir yaw bozucu tork (ext_m) eklenip mevcut (integral'siz)
% mimari 60 s boyunca gozlemleniyor. hdg_sp=0 sabit (maniivra yok).

clear; clc;
addpath(fileparts(mfilename('fullpath')));

p = tiltrotor_params();

x0 = zeros(24,1);
x0(3)     = -80;
x0(4)     = 16.0;
x0(7:10)  = quat_from_euler(0,deg2rad(1.25),0);
x0(14:16) = [4.37; 4.37; 0];
x0(17:19) = [p.tilt.max; p.tilt.max; 0];

z_sp = x0(3);
V_SP = 16.0;
Tsim = 60;
dt_ctrl = p.Ts_ctrl;
n_sub = 5;
dt_phys = dt_ctrl/n_sub;
N = round(Tsim/dt_ctrl);

% KALICI bozucu yaw torku -- buyuklugu, SITL'in "item N" turu etkilerinde
% (Adim 63-64) olculen mertebeyle (birkac N'lik kalici kuvvetin ~0.5 m'lik
% kol ile urettigi tork) kabaca ayni mertebede, 0.1 N*m.
EXT_YAW_TORQUE = 0.1; % N*m, surekli

x = x0;
state_v = [0; 0];

log.t = zeros(N,1);
log.att = zeros(3,N);
log.omega = zeros(3,N);

for k = 1:N
    t = (k-1)*dt_ctrl;

    wind_ned = [0;0;0];
    ext_m = [0; 0; EXT_YAW_TORQUE];   % kalici, sabit yaw bozucu tork

    att = quat_to_euler(x(7:10));
    omega = x(11:13);
    vel_b = x(4:6);
    V = norm(vel_b);
    qbar = 0.5 * p.rho * V^2;

    hdg_sp = 0;   % sabit -- maniivra yok, saf duz-ucus-tutma testi

    [T_wing, surf_cmd, state_v] = fixedwing_control_law( ...
        hdg_sp, z_sp, V_SP, att, omega, x(3), x(6), vel_b(1), qbar, state_v, p);

    T_cmd = [T_wing; T_wing; 0];
    delta_cmd = [p.tilt.max; p.tilt.max; 0];

    for s = 1:n_sub
        x = rk4_step(@(xx) tiltrotor_plant_deriv(xx, T_cmd, delta_cmd, p, wind_ned, ext_m, surf_cmd), x, dt_phys);
    end

    log.t(k) = t;
    log.att(:,k) = att;
    log.omega(:,k) = omega;
end

fprintf('=== Kalici %.2f N*m yaw bozucusu altinda, hdg_sp=0 sabit ===\n', EXT_YAW_TORQUE);
% Ilk/orta/son pencerelerdeki roll'u karsilastir -- BUYUYOR mu (kararsizlik)
% yoksa SABIT bir degere mi YERLESIYOR (P-controller'in beklenen davranisi)?
idx10 = find(log.t>=10,1); idx20 = find(log.t>=20,1);
idx30 = find(log.t>=30,1); idx45 = find(log.t>=45,1); idx60 = N;
fprintf('Roll  @10s=%.2f  @20s=%.2f  @30s=%.2f  @45s=%.2f  @60s=%.2f deg\n', ...
    rad2deg(log.att(1,idx10)), rad2deg(log.att(1,idx20)), rad2deg(log.att(1,idx30)), ...
    rad2deg(log.att(1,idx45)), rad2deg(log.att(1,idx60)));
fprintf('Yaw   @10s=%.2f  @20s=%.2f  @30s=%.2f  @45s=%.2f  @60s=%.2f deg\n', ...
    rad2deg(log.att(3,idx10)), rad2deg(log.att(3,idx20)), rad2deg(log.att(3,idx30)), ...
    rad2deg(log.att(3,idx45)), rad2deg(log.att(3,idx60)));
fprintf('Son omega norm: %.4f rad/s\n', norm(log.omega(:,end)));
