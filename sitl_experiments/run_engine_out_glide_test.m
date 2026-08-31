%% RUN_ENGINE_OUT_GLIDE_TEST
% Kullanicinin onerisi: belirli bir ileri hizdan sonra TUM ROTORLERI KAPAT
% (T=0), suzulme sirasinda tilt'i tamamla (T=0 iken tilt'in kuvvet/tork
% etkisi zaten sifir, yani "bedava" mekanik yeniden konumlandirma), sonra
% motorlari yeniden calistir.
%
% KRITIK FIZIK: effectiveness_matrix.m'de dtau/ddelta ve dtau/dT ikisi de
% T ile CARPIMLI -- T=0 iken TUM rotor sutunlari (G'nin ilk 6 sutunu) SIFIR
% olur. Yani motor kapaliyken attitude kontrolu TAMAMEN yuzeylere kalir.
% Bu test o varsayimin tutup tutmadigini olcer.
%
% Uc faz:
%   1) GUCLU GECIS  (t < t_glide_start): normal WLS, T ve yuzeyler aktif,
%      ileri hiz esige kadar birikir
%   2) SUZULME      (t_glide_start .. t_glide_start+T_GLIDE): T=0 zorlanir,
%      delta_cmd/surf_cmd WLS'ten gelmeye devam eder (ama T=0 oldugu icin
%      rotor sutunlari G'de sifir -- WLS attitude'u YALNIZCA yuzeylerle
%      tutmaya calisir)
%   3) YENIDEN CALISTIRMA (t > glide sonu): T normale doner

clear; clc;
addpath(fileparts(mfilename('fullpath')));

p = tiltrotor_params();
u_trim = hover_trim(p);

x0 = zeros(24,1);
x0(3)     = -80;
x0(7:10)  = quat_from_euler(0,0,0);
x0(14:19) = u_trim;

V_GLIDE_TRIGGER = 12.0;   % m/s, suzulmenin baslayacagi ileri hiz
T_GLIDE = 4.0;            % s, motorlarin kapali kalacagi sure

att_sp = [0;0;0];
z_sp   = x0(3);
Fx_final = 12;
t_ramp   = 12;
Tsim = 30;
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
a_virt = zeros(3,1);
gliding = false;
t_glide_start = NaN;
surf_cmd_frozen = nan(5,1);
delta_cmd_frozen = nan(3,1);

log.t = zeros(N,1);
log.att = zeros(3,N);
log.omega = zeros(3,N);
log.T = zeros(3,N);
log.delta = zeros(3,N);
log.surf = zeros(5,N);
log.vel_body = zeros(3,N);
log.pos = zeros(3,N);
log.phase = zeros(N,1);   % 1=powered, 2=glide, 3=relight

for k = 1:N
    t = (k-1)*dt_ctrl;
    Fx_sp = Fx_final * min(1, t/t_ramp);

    wind_ned = [0;0;0];
    ext_m = [0;0;0];

    att = quat_to_euler(x(7:10));
    omega = x(11:13);
    T_act = x(14:16);
    delta_act = x(17:19);
    vel_b = x(4:6);
    V = norm(vel_b);
    qbar = 0.5 * p.rho * V^2;

    if ~gliding && vel_b(1) >= V_GLIDE_TRIGGER
        gliding = true;
        t_glide_start = t;
        fprintf('t=%.2f s: v_fwd=%.2f m/s esigi gecti -- TUM MOTORLAR KAPATILDI (suzulme baslar)\n', t, vel_b(1));
    end
    in_glide = gliding && (t < t_glide_start + T_GLIDE);
    relit = gliding && ~in_glide;
    if relit && log.phase(max(k-1,1)) == 2
        fprintf('t=%.2f s: motorlar YENIDEN CALISTIRILDI\n', t);
    end

    u_actual9 = [T_act; delta_act; a_virt];

    xdot_now = tiltrotor_plant_deriv(x, T_act, delta_act, p, wind_ned, ext_m, zeros(p.surf.n,1));
    omega_dot_raw = xdot_now(11:13);
    omega_dot_filt = omega_dot_filt + 0.3*(omega_dot_raw - omega_dot_filt);

    alt_accum = alt_accum + dt_ctrl;
    if alt_accum >= p.Ts_pos - 1e-12
        vz_ned = xdot_now(3);
        [Fz_sp, alt_state] = altitude_loop(z_sp, x(3), vz_ned, alt_state, p);
        alt_accum = alt_accum - p.Ts_pos;
    end
    F_sp = [Fx_sp; Fz_sp];

    % WLS DUZELTMESI: T=0'i sonradan zorlamak yerine, WLS'e BASTAN soyle --
    % Tmax=0 kutu kisitiyla, itki kanalinin gercekten yok oldugunu bilerek
    % tahsis yapsin (dtau/dT, T'ye bagli DEGIL -- WLS T=0'da bile "itkiden
    % tork uretebilirim" saniyordu, sonradan override edince plani tutarsiz
    % kaliyordu).
    p_ctrl = p;
    if in_glide
        p_ctrl.rotor.Tmax = 0;
    end

    [T_cmd, delta_cmd, ctrl_state, diagn, surf_cmd] = indi_attitude_controller( ...
        att_sp, att, omega, omega_dot_filt, F_sp, u_actual9, ctrl_state, p_ctrl, leso_axes, [], qbar);

    if gliding && t < t_glide_start + 0.5
        fprintf('  t=%.3f att=[%.2f %.2f %.2f]deg omega=[%.3f %.3f %.3f] surf_cmd=[%.1f %.1f %.1f %.1f %.1f]deg nu_des=[%.2f %.2f %.2f %.2f %.2f]\n', ...
            t, rad2deg(att), omega, rad2deg(surf_cmd), diagn.nu_des);
    end

    a_virt = diagn.a_cmd;

    for s = 1:n_sub
        x = rk4_step(@(xx) tiltrotor_plant_deriv(xx, T_cmd, delta_cmd, p, wind_ned, ext_m, surf_cmd), x, dt_phys);
    end

    log.t(k) = t;
    log.att(:,k) = att;
    log.omega(:,k) = omega;
    log.T(:,k) = T_act;
    log.delta(:,k) = delta_act;
    log.surf(:,k) = x(20:24);
    log.vel_body(:,k) = vel_b;
    log.pos(:,k) = x(1:3);
    log.phase(k) = 1 + gliding + relit;
end

idx_glide = log.phase == 2;
fprintf('\nSuzulme baslangici: t=%.2f s, bitisi: t=%.2f s\n', t_glide_start, t_glide_start+T_GLIDE);
fprintf('Suzulme SIRASINDA max |pitch|: %.2f deg, max |roll|: %.2f deg\n', ...
    max(abs(rad2deg(log.att(2,idx_glide)))), max(abs(rad2deg(log.att(1,idx_glide)))));
fprintf('Suzulme sirasinda irtifa kaybi: %.2f m\n', ...
    (-log.pos(3,find(idx_glide,1,'last'))) - (-log.pos(3,find(idx_glide,1,'first'))));
fprintf('Genel max |pitch|: %.2f deg, max |roll|: %.2f deg\n', max(abs(rad2deg(log.att(2,:)))), max(abs(rad2deg(log.att(1,:)))));
fprintf('Genel max |omega| = %.4f rad/s\n', max(vecnorm(log.omega)));
fprintf('Son durum: ileri hiz=%.2f m/s, irtifa sapmasi=%.2f m\n', log.vel_body(1,end), -log.pos(3,end)-80);

fig = figure('Position',[100 100 1100 900]);
subplot(3,2,1); hold on; grid on;
plot(log.t, rad2deg(log.att(1,:)), 'DisplayName','\phi'); plot(log.t, rad2deg(log.att(2,:)), 'DisplayName','\theta');
xline(t_glide_start,'r--'); xline(t_glide_start+T_GLIDE,'g--');
ylabel('deg'); legend('Location','best'); title('Attitude (kirmizi=suzulme baslar, yesil=yeniden calisir)');

subplot(3,2,2); hold on; grid on;
plot(log.t, log.T(1,:)); plot(log.t, log.T(2,:)); plot(log.t, log.T(3,:));
xline(t_glide_start,'r--'); xline(t_glide_start+T_GLIDE,'g--');
ylabel('N'); title('Rotor itkileri');

subplot(3,2,3); hold on; grid on;
plot(log.t, rad2deg(log.delta(1,:))); plot(log.t, rad2deg(log.delta(2,:))); plot(log.t, rad2deg(log.delta(3,:)));
xline(t_glide_start,'r--'); xline(t_glide_start+T_GLIDE,'g--');
ylabel('deg'); title('Tilt acilari');

subplot(3,2,4); hold on; grid on;
plot(log.t, rad2deg(log.surf(1,:)), 'DisplayName','elevon'); plot(log.t, rad2deg(log.surf(3,:)), 'DisplayName','elevator');
plot(log.t, rad2deg(log.surf(5,:)), 'DisplayName','rudder');
xline(t_glide_start,'r--'); xline(t_glide_start+T_GLIDE,'g--');
ylabel('deg'); legend('Location','best'); title('Yuzey sapmalari');

subplot(3,2,5); hold on; grid on;
plot(log.t, log.vel_body(1,:));
xline(t_glide_start,'r--'); xline(t_glide_start+T_GLIDE,'g--'); yline(V_GLIDE_TRIGGER,'k:');
ylabel('m/s'); xlabel('t (s)'); title('Ileri hiz');

subplot(3,2,6); hold on; grid on;
plot(log.t, -log.pos(3,:)-80);
xline(t_glide_start,'r--'); xline(t_glide_start+T_GLIDE,'g--');
ylabel('irtifa sapmasi (m)'); xlabel('t (s)'); title('Irtifa');

saveas(fig, fullfile(fileparts(mfilename('fullpath')), 'engine_out_glide_WLSFIX_test.png'));
fprintf('\nGrafik kaydedildi: engine_out_glide_WLSFIX_test.png\n');
