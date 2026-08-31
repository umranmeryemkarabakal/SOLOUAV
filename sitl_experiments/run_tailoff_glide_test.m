%% RUN_TAILOFF_GLIDE_TEST
% Kullanicinin onerisi: ileri hiz belli bir esigi (aerodinamik ruzgar gulu
% etkisinin SITL'de olculdugu esik, Adim 16-20: ~2 m/s) gectikten sonra
% KUYRUK ROTORUNU (T2) ZORLA KAPAT (fiziksel motor arizasi/kapanmasi gibi)
% ve WLS/plant'in kapali dongude buna nasil tepki verdigini olc.
%
% Bu test SADECE PITCH sorusunu cevaplar (MATLAB'da yanal aero yok, bkz.
% dosya notu asagida) -- yaw sorusu yalnizca SITL'de test edilebilir.
%
% NOT: T2 zorla sifirlanirken delta2 (kuyruk tilt) kontrolcunun komutuna
% birakilir -- gercekci senaryo "motor calismiyor ama servo hala hareket
% edebiliyor" (ya da tam tersi onemli degil, T=0 iken tilt'in Fz/tau
% katkisi zaten sifir).

clear; clc;
addpath(fileparts(mfilename('fullpath')));

p = tiltrotor_params();
u_trim = hover_trim(p);

x0 = zeros(19,1);
x0(3)     = -80;
x0(7:10)  = quat_from_euler(0,0,0);
x0(14:19) = u_trim;

att_sp = [0;0;0];
z_sp   = x0(3);
Fx_final = 10;
t_ramp   = 12;
V_TRIGGER = 2.0;   % m/s, govde ileri hizi -- SITL'de olculen aero-yaw-sonumleme esigi

Tsim = 20;         % Adim 15'ten daha uzun -- kapanmadan sonraki davranisi gormek icin
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
tail_off = false;
t_tailoff = NaN;

log.t = zeros(N,1);
log.att = zeros(3,N);
log.omega = zeros(3,N);
log.delta = zeros(3,N);
log.T = zeros(3,N);
log.vel_body = zeros(3,N);
log.pos = zeros(3,N);
log.tail_off = false(N,1);

for k = 1:N
    t = (k-1)*dt_ctrl;
    Fx_sp = Fx_final * min(1, t/t_ramp);

    wind_ned = [0;0;0];
    ext_m = [0;0;0];

    att = quat_to_euler(x(7:10));
    omega = x(11:13);
    u_actual = x(14:19);
    v_fwd = x(4);   % govde ileri hizi (u)

    if ~tail_off && v_fwd >= V_TRIGGER
        tail_off = true;
        t_tailoff = t;
        fprintf('t=%.2f s: v_fwd=%.2f m/s esigi gecti -- KUYRUK MOTORU KAPATILDI\n', t, v_fwd);
    end

    xdot_now = tiltrotor_plant_deriv(x, u_actual(1:3), u_actual(4:6), p, wind_ned, ext_m);
    omega_dot_raw = xdot_now(11:13);
    omega_dot_filt = omega_dot_filt + 0.3*(omega_dot_raw - omega_dot_filt);

    alt_accum = alt_accum + dt_ctrl;
    if alt_accum >= p.Ts_pos - 1e-12
        vz_ned = xdot_now(3);
        [Fz_sp, alt_state] = altitude_loop(z_sp, x(3), vz_ned, alt_state, p);
        alt_accum = alt_accum - p.Ts_pos;
    end
    F_sp = [Fx_sp; Fz_sp];

    [T_cmd, delta_cmd, ctrl_state, diagn] = indi_attitude_controller( ...
        att_sp, att, omega, omega_dot_filt, F_sp, u_actual, ctrl_state, p, leso_axes);

    if tail_off
        T_cmd(3) = 0;   % motor fiziksel olarak kapali -- WLS'in komutu ne olursa olsun
    end

    for s = 1:n_sub
        x = rk4_step(@(xx) tiltrotor_plant_deriv(xx, T_cmd, delta_cmd, p, wind_ned, ext_m), x, dt_phys);
    end

    log.t(k) = t;
    log.att(:,k) = att;
    log.omega(:,k) = omega;
    log.delta(:,k) = u_actual(4:6);
    log.T(:,k) = u_actual(1:3);
    log.vel_body(:,k) = x(4:6);
    log.pos(:,k) = x(1:3);
    log.tail_off(k) = tail_off;
end

fprintf('\nKuyruk kapanma zamani: t=%.2f s\n', t_tailoff);
fprintf('Kapanmadan sonraki max |pitch|: %.2f deg\n', max(abs(rad2deg(log.att(2, log.tail_off)))));
fprintf('Kapanmadan sonraki max |roll|:  %.2f deg\n', max(abs(rad2deg(log.att(1, log.tail_off)))));
fprintf('Son irtifa degisimi: %.2f m\n', -log.pos(3,end)-80);
fprintf('Son govde ileri hiz: %.2f m/s\n', log.vel_body(1,end));
fprintf('Simulasyon boyunca max |omega| = %.4f rad/s\n', max(vecnorm(log.omega)));

%% --- Grafikler ---
fig = figure('Position',[100 100 1100 900]);

subplot(3,2,1); hold on; grid on;
plot(log.t, rad2deg(log.att(1,:)), 'LineWidth',1.3, 'DisplayName','\phi (roll)');
plot(log.t, rad2deg(log.att(2,:)), 'LineWidth',1.3, 'DisplayName','\theta (pitch)');
xline(t_tailoff, 'r--', 'DisplayName','kuyruk kapandi');
ylabel('deg'); legend('Location','best'); title('Attitude (referans = 0)');

subplot(3,2,2); hold on; grid on;
plot(log.t, log.T(1,:), 'LineWidth',1.1, 'DisplayName','T_0');
plot(log.t, log.T(2,:), 'LineWidth',1.1, 'DisplayName','T_1');
plot(log.t, log.T(3,:), 'LineWidth',1.1, 'DisplayName','T_2 (kuyruk)');
xline(t_tailoff, 'r--');
ylabel('itki (N)'); legend('Location','best'); title('Rotor itkileri');

subplot(3,2,3); hold on; grid on;
plot(log.t, rad2deg(log.delta(1,:)), 'LineWidth',1.1, 'DisplayName','\delta_0');
plot(log.t, rad2deg(log.delta(2,:)), 'LineWidth',1.1, 'DisplayName','\delta_1');
plot(log.t, rad2deg(log.delta(3,:)), 'LineWidth',1.1, 'DisplayName','\delta_2');
xline(t_tailoff, 'r--');
ylabel('tilt (deg)'); legend('Location','best'); title('Rotor tilt acilari');

subplot(3,2,4); hold on; grid on;
plot(log.t, log.vel_body(1,:), 'LineWidth',1.3, 'DisplayName','u (ileri)');
plot(log.t, log.vel_body(3,:), 'LineWidth',1.3, 'DisplayName','w (dikey, govde)');
xline(t_tailoff, 'r--'); yline(V_TRIGGER, 'k:', 'DisplayName','esik');
ylabel('m/s'); legend('Location','best'); title('Govde hizlari');

subplot(3,2,5); hold on; grid on;
plot(log.t, -log.pos(3,:)-80, 'LineWidth',1.3);
xline(t_tailoff, 'r--');
ylabel('irtifa sapmasi (m)'); xlabel('t (s)'); title('Irtifa (80m referansindan sapma)');

subplot(3,2,6); hold on; grid on;
plot(log.t, rad2deg(log.omega(1,:)), 'LineWidth',1.1, 'DisplayName','p (roll rate)');
plot(log.t, rad2deg(log.omega(2,:)), 'LineWidth',1.1, 'DisplayName','q (pitch rate)');
xline(t_tailoff, 'r--');
ylabel('deg/s'); xlabel('t (s)'); legend('Location','best'); title('Acisal hizlar');

saveas(fig, fullfile(fileparts(mfilename('fullpath')), 'tailoff_glide_test.png'));
fprintf('\nGrafik kaydedildi: tailoff_glide_test.png\n');
