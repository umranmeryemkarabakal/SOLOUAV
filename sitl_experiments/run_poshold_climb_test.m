%% RUN_POSHOLD_CLIMB_TEST
% Adim 87'nin SITL bulgusunu MATLAB'da yeniden uretme denemesi: pos_hold
% (position_loop.m) AKTIFKEN, ayni anda irtifa 0'dan -30'a TIRMANIRKEN
% (yani run_station_keeping_test.m'in HICBIR ZAMAN test etmedigi kombinasyon
% -- o testte z_sp = x0(3), yani irtifa hep sabit) yaw davranisini olcer.
%
% SITL'de bu kombinasyonla (test_sp ... pos_hold=1, z_sp kademeli 0->-30)
% yaw surekli, sonmeyen bir spin'e giriyordu (r -2.5 rad/s'e kadar). Bu test
% AYNI kombinasyonu saf MATLAB'da (aero yaw sonumlemesi YOK, en kotu durum)
% calistirip ayni spin'in burada da olup olmadigini olcer.
%
% Kullanim: run_poshold_climb_test

clear; clc;
addpath(fileparts(mfilename('fullpath')));

p = tiltrotor_params();

Tsim    = 40.0;
leso_axes = [true; true; false];

dt_ctrl = p.Ts_ctrl; n_sub = 5; dt_phys = dt_ctrl/n_sub;
N = round(Tsim/dt_ctrl);

u_trim = hover_trim(p);
x0 = zeros(19,1);
x0(3)     = 0;                       % yerden basla (SITL'deki gibi)
x0(7:10)  = quat_from_euler(0,0,deg2rad(90));  % SITL spawn heading ~90 deg
x0(14:19) = u_trim;

x = x0;
ctrl_state = init_ctrl_state();
omega_dot_filt = zeros(3,1);
alt_state = 0; alt_accum = 0; pos_state = zeros(2,1);
Fz_sp = -p.m*p.g;
att_sp_xy = [0;0];
fx_sp = 0;

pos_sp = x0(1:2);
z_sp   = 0;              % SITL'deki kademeli tirmanmayi taklit et
z_targets = [-3 -10 -18 -25 -30];
z_step_t  = [5 10 15 20 25];   % s, her hedefe gecis zamani (SITL'deki 5s araliklarla eslesir)

log.t     = zeros(N,1);
log.pos   = zeros(2,N);
log.vh    = zeros(N,1);
log.att   = zeros(3,N);
log.omega = zeros(3,N);
log.z     = zeros(N,1);

for k = 1:N
    t = (k-1)*dt_ctrl;

    zi = find(t >= z_step_t, 1, 'last');
    if ~isempty(zi), z_sp = z_targets(zi); end

    att   = quat_to_euler(x(7:10));
    omega = x(11:13);
    u_actual = x(14:19);

    xdot_now = tiltrotor_plant_deriv(x, u_actual(1:3), u_actual(4:6), p, [0;0;0], [0;0;0]);
    omega_dot_filt = omega_dot_filt + 0.3*(xdot_now(11:13) - omega_dot_filt);

    alt_accum = alt_accum + dt_ctrl;
    if alt_accum >= p.Ts_pos - 1e-12
        [Fz_sp, alt_state] = altitude_loop(z_sp, x(3), xdot_now(3), alt_state, p);
        [att_sp_xy, fx_sp, pos_state] = position_loop(pos_sp, x(1:2), xdot_now(1:2), ...
                                                      att(3), mean(u_actual(4:6)), pos_state, p);
        alt_accum = alt_accum - p.Ts_pos;
    end

    psi_sp = deg2rad(90);   % SITL'deki gibi yaw_sp sabit, hic adim yok
    att_sp = [att_sp_xy(1); att_sp_xy(2); psi_sp];

    [T_cmd, delta_cmd, ctrl_state, ~] = indi_attitude_controller( ...
        att_sp, att, omega, omega_dot_filt, [fx_sp; Fz_sp], u_actual, ctrl_state, p, leso_axes);

    for s = 1:n_sub
        x = rk4_step(@(xx) tiltrotor_plant_deriv(xx, T_cmd, delta_cmd, p, [0;0;0], [0;0;0]), x, dt_phys);
    end

    log.t(k)     = t;
    log.pos(:,k) = x(1:2);
    log.vh(k)    = norm(xdot_now(1:2));
    log.att(:,k) = att;
    log.omega(:,k) = omega;
    log.z(k)     = x(3);
end

fprintf('\n=== pos_hold AKTIF + ESZAMANLI TIRMANMA (0 -> -30) testi ===\n');
fprintf('%6s %8s %8s %8s %8s\n', 't(s)', 'yaw(deg)', 'r(rad/s)', 'alt(m)', 'vh(m/s)');
for tt = 0:2:Tsim
    [~, idx] = min(abs(log.t - tt));
    fprintf('%6.1f %8.2f %8.4f %8.2f %8.3f\n', log.t(idx), rad2deg(log.att(3,idx)), ...
            log.omega(3,idx), -log.z(idx), log.vh(idx));
end

r_tail = log.omega(3, log.t >= Tsim-10);
yaw_tail = rad2deg(log.att(3, log.t >= Tsim-10));
fprintf('\nSon 10s: yaw_rate RMS=%.5f rad/s, max|r|=%.4f rad/s, yaw araligi=[%.2f, %.2f] deg\n', ...
        sqrt(mean(r_tail.^2)), max(abs(r_tail)), min(yaw_tail), max(yaw_tail));
