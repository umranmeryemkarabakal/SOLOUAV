%% RUN_TRANSITION_TEST
% Hover -> cruise gecis testi: disaridaki (guidance) dongu ileri govde-kuvveti
% (Fx_sp) istegini kademeli olarak artirir. WLS bunu karsilamanin en verimli
% yolunun rotorlari one yatirmak oldugunu "kendiliginden" bulur (Fx sadece
% tilt ile uretilebilir, T dikeyken Fx katkisi sifirdir). Ortalama tilt acisi
% arttikca gain_schedule.m INDI kazanclarini ve WLS aktuator tercihini
% (basit, dogrusal-rampali) kademeli olarak degistirir.
%
% Bu script mimarinin "basit gain-scheduling" ayagini gosterir: ayri bir
% durum makinesi yok, sadece olculen tilt acisina bagli surekli interpolasyon.

clear; clc;
addpath(fileparts(mfilename('fullpath')));

p = tiltrotor_params();
u_trim = hover_trim(p);

x0 = zeros(19,1);
x0(3)     = -80;
x0(7:10)  = quat_from_euler(0,0,0);
x0(14:19) = u_trim;

att_sp = [0;0;0];
z_sp   = x0(3);      % irtifayi baslangic degerinde tutmaya calis (bkz. altitude_loop.m)
Fx_final = 10;      % N, cruise'a dogru hedeflenen ileri govde-kuvveti (limitli — bkz. dosya notu)
t_ramp   = 12;       % s, rampanin tamamlanma suresi

% NOT: Bu betik yalnizca INDI/WLS/gain-scheduling'in DEGISEN tilt acisina
% nasil tepki verdigini gosterir; tam bir hover->cruise gecis kontrolcusu
% icin ayrica bir hiz/irtifa dis dongusu (bu depoda kapsam disi) gerekir.
% Fx_final ve Tsim, ic dongunun kararli kaldigi bolgede tutulacak sekilde
% sinirlandirilmistir.
Tsim = 14;
dt_ctrl = p.Ts_ctrl;
n_sub = 5;
dt_phys = dt_ctrl/n_sub;
N = round(Tsim/dt_ctrl);

leso_axes = [true;true;false];

x = x0;
ctrl_state = init_ctrl_state();
omega_dot_filt = zeros(3,1);
alt_state = 0;
alt_accum = 0;
Fz_sp = -p.m*p.g;

log.t = zeros(N,1);
log.att = zeros(3,N);
log.omega = zeros(3,N);
log.delta = zeros(3,N);
log.T = zeros(3,N);
log.delta_bar = zeros(N,1);
log.Kp_rate = zeros(3,N);
log.wu_tilt = zeros(3,N);
log.vel_body = zeros(3,N);
log.Fx_sp = zeros(N,1);
log.pos = zeros(3,N);

for k = 1:N
    t = (k-1)*dt_ctrl;
    Fx_sp = Fx_final * min(1, t/t_ramp);

    wind_ned = [0;0;0];
    ext_m = [0;0;0];

    att = quat_to_euler(x(7:10));
    omega = x(11:13);
    u_actual = x(14:19);

    xdot_now = tiltrotor_plant_deriv(x, u_actual(1:3), u_actual(4:6), p, wind_ned, ext_m);
    omega_dot_raw = xdot_now(11:13);
    omega_dot_filt = omega_dot_filt + 0.3*(omega_dot_raw - omega_dot_filt);

    % --- Irtifa dis dongusu (decimasyonlu, Ts_pos = 1/50 s) ---
    alt_accum = alt_accum + dt_ctrl;
    if alt_accum >= p.Ts_pos - 1e-12
        vz_ned = xdot_now(3);
        [Fz_sp, alt_state] = altitude_loop(z_sp, x(3), vz_ned, alt_state, p);
        alt_accum = alt_accum - p.Ts_pos;
    end
    F_sp = [Fx_sp; Fz_sp];

    [T_cmd, delta_cmd, ctrl_state, diagn] = indi_attitude_controller( ...
        att_sp, att, omega, omega_dot_filt, F_sp, u_actual, ctrl_state, p, leso_axes);

    for s = 1:n_sub
        x = rk4_step(@(xx) tiltrotor_plant_deriv(xx, T_cmd, delta_cmd, p, wind_ned, ext_m), x, dt_phys);
    end

    log.t(k) = t;
    log.att(:,k) = att;
    log.omega(:,k) = omega;
    log.delta(:,k) = u_actual(4:6);
    log.T(:,k) = u_actual(1:3);
    log.delta_bar(k) = mean(u_actual(4:6));
    log.Kp_rate(:,k) = diagn.sched.Kp_rate;
    log.wu_tilt(:,k) = diagn.sched.wu_tilt;
    log.vel_body(:,k) = x(4:6);
    log.Fx_sp(k) = Fx_sp;
    log.pos(:,k) = x(1:3);
end

fprintf('Son ortalama tilt: %.1f deg,  ileri hiz (govde u): %.2f m/s,  irtifa degisimi: %.2f m\n', ...
    rad2deg(log.delta_bar(end)), log.vel_body(1,end), -log.pos(3,end)-80);
fprintf('Simulasyon boyunca max |omega| = %.4f rad/s (kararlilik kontrolu)\n', max(vecnorm(log.omega)));

%% --- Grafikler ---
fig = figure('Position',[100 100 1100 900]);

subplot(3,2,1); hold on; grid on;
plot(log.t, rad2deg(log.delta(1,:)), 'LineWidth',1.3, 'DisplayName','\delta_0 (sag kanat)');
plot(log.t, rad2deg(log.delta(2,:)), 'LineWidth',1.3, 'DisplayName','\delta_1 (sol kanat)');
plot(log.t, rad2deg(log.delta(3,:)), 'LineWidth',1.3, 'DisplayName','\delta_2 (kuyruk)');
ylabel('tilt (deg)'); legend('Location','best'); title('Rotor tilt acilari (WLS kendiliginden secti)');

subplot(3,2,2); hold on; grid on;
plot(log.t, log.T(1,:), 'LineWidth',1.1, 'DisplayName','T_0');
plot(log.t, log.T(2,:), 'LineWidth',1.1, 'DisplayName','T_1');
plot(log.t, log.T(3,:), 'LineWidth',1.1, 'DisplayName','T_2');
ylabel('itki (N)'); legend('Location','best'); title('Rotor itkileri');

subplot(3,2,3); hold on; grid on;
plot(log.t, log.vel_body(1,:), 'LineWidth',1.3, 'DisplayName','u (ileri hiz)');
plot(log.t, log.Fx_sp/p.m, 'k--', 'DisplayName','Fx_{sp}/m (referans ivme)');
ylabel('m/s'); legend('Location','best'); title('Govde-eksen ileri hiz');

subplot(3,2,4); hold on; grid on;
plot(log.t, rad2deg(log.att(1,:)), 'LineWidth',1.1, 'DisplayName','\phi (roll)');
plot(log.t, rad2deg(log.att(2,:)), 'LineWidth',1.1, 'DisplayName','\theta (pitch)');
plot(log.t, rad2deg(log.att(3,:)), 'LineWidth',1.1, 'DisplayName','\psi (yaw)');
ylabel('deg'); legend('Location','best'); title('Attitude (referans = 0)');

subplot(3,2,5); hold on; grid on;
plot(log.t, log.Kp_rate(1,:), 'LineWidth',1.3, 'DisplayName','Kp_{rate,roll}');
plot(log.t, log.Kp_rate(3,:), 'LineWidth',1.3, 'DisplayName','Kp_{rate,yaw}');
yyaxis right
plot(log.t, rad2deg(log.delta_bar), 'k--', 'LineWidth',1.3, 'DisplayName','ortalama tilt (deg)');
ylabel('tilt (deg)');
yyaxis left; ylabel('Kp_{rate}'); xlabel('t (s)');
legend('Location','best'); title('Gain-scheduling: Kp_{rate} vs ortalama tilt');

subplot(3,2,6); hold on; grid on;
plot(log.t, log.wu_tilt(1,:), 'LineWidth',1.3, 'DisplayName','Wu_{tilt,kanat}');
plot(log.t, log.wu_tilt(3,:), 'LineWidth',1.3, 'DisplayName','Wu_{tilt,kuyruk}');
ylabel('Wu agirligi'); xlabel('t (s)'); legend('Location','best');
title('Gain-scheduling: WLS tilt tercih agirligi');

saveas(fig, fullfile(fileparts(mfilename('fullpath')), 'transition_test.png'));
fprintf('\nGrafik kaydedildi: transition_test.png\n');
