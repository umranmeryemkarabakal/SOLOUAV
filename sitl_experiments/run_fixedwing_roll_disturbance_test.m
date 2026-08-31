%% RUN_FIXEDWING_ROLL_DISTURBANCE_TEST (Adim 82's follow-up)
% Adim 79'un YAW eksenine uyguladigi testin ROLL eksenindeki karsiligi.
% SITL'de (Adim 82) roll_sp=0 sabit tutulup yaw geri beslemesi TAMAMEN
% kaldirilsa bile roll'un yine ~57.5 dereceye ulastigi bulundu -- yani sorun
% roll ekseninin KENDISINDE. Burada MATLAB'in kendi modelinde AYNI sey
% denenip denenmedigini goruyoruz: kalici bir ROLL bozucu toru altinda
% mevcut (integral'siz) P+D roll yasasi SABIT mi kaliyor (o zaman SITL'e ozgu
% bir model uyusmazligi) yoksa MATLAB'da da mi buyuyor (o zaman gercek bir
% kontrol-yasasi/otorite eksikligi).

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

% KALICI bozucu ROLL torku -- Adim 79'un yaw testiyle AYNI buyuklukte (0.1 N*m)
% dogrudan karsilastirilabilir olmasi icin.
EXT_ROLL_TORQUE = 0.1; % N*m, surekli

x = x0;
state_v = [0; 0];

log.t = zeros(N,1);
log.att = zeros(3,N);
log.omega = zeros(3,N);

for k = 1:N
    t = (k-1)*dt_ctrl;

    wind_ned = [0;0;0];
    ext_m = [EXT_ROLL_TORQUE; 0; 0];   % kalici, sabit ROLL bozucu tork (X ekseni)

    att = quat_to_euler(x(7:10));
    omega = x(11:13);
    vel_b = x(4:6);
    V = norm(vel_b);
    qbar = 0.5 * p.rho * V^2;

    hdg_sp = 0;   % sabit -- Adim 82'nin SITL testiyle tutarli olsun diye
                  % roll_sp de dolayli olarak sifira yakin kalacak (kucuk
                  % heading hatasi disinda)

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

fprintf('=== Kalici %.2f N*m ROLL bozucusu altinda, hdg_sp=0 sabit ===\n', EXT_ROLL_TORQUE);
idx10 = find(log.t>=10,1); idx20 = find(log.t>=20,1);
idx30 = find(log.t>=30,1); idx45 = find(log.t>=45,1); idx60 = N;
fprintf('Roll  @10s=%.2f  @20s=%.2f  @30s=%.2f  @45s=%.2f  @60s=%.2f deg\n', ...
    rad2deg(log.att(1,idx10)), rad2deg(log.att(1,idx20)), rad2deg(log.att(1,idx30)), ...
    rad2deg(log.att(1,idx45)), rad2deg(log.att(1,idx60)));
fprintf('Yaw   @10s=%.2f  @20s=%.2f  @30s=%.2f  @45s=%.2f  @60s=%.2f deg\n', ...
    rad2deg(log.att(3,idx10)), rad2deg(log.att(3,idx20)), rad2deg(log.att(3,idx30)), ...
    rad2deg(log.att(3,idx45)), rad2deg(log.att(3,idx60)));
fprintf('Son omega norm: %.4f rad/s\n', norm(log.omega(:,end)));
