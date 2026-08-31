%% RUN_HANDOFF_TRANSIENT_TEST
% Kullanicinin plani, adim 2 (izole test): belirli bir hizda/irtifada
% GECIS KOMUTU gelince (a) kuyruk motoru (T2) kapatilir, (b) on iki kanat
% rotorunun tilt acisi ESITLENIR (o anki ortalamalarina). T0/T1 ve yuzeyler
% MEVCUT WLS/INDI dongusunde serbest kalir -- soru: bu ani gecis kendi
% basina takla attiriyor mu, yoksa mevcut kapali dongu (T0/T1 farki +
% yuzeyler) bunu tolere edebiliyor mu.
%
% Onceki testlerden farki: T2=0 olsa da T0,T1 SIFIRLANMIYOR -- yani rotor
% tabanli otorite TAMAMEN kaybolmuyor, sadece kuyruk kanali ve tilt'in
% BAGIMSIZ iki-eksenli kontrolu kayboluyor (artik ikisi TEK bir sayi).

clear; clc;
addpath(fileparts(mfilename('fullpath')));

p = tiltrotor_params();
u_trim = hover_trim(p);

x0 = zeros(24,1);
x0(3)     = -80;
x0(7:10)  = quat_from_euler(0,0,0);
x0(14:19) = u_trim;

V_HANDOFF = 12.0;      % m/s, gecis tetikleyici
TILT_RAMP_RATE = deg2rad(0.5);  % rad/s, gecis sonrasi senkron ileri tilt hizi

att_sp = [0;0;0];
z_sp   = x0(3);
Fx_final = 12;
t_ramp   = 12;
Tsim = 100;
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
handoff = false;
t_handoff = NaN;
delta_eq = NaN;   % esitlenen tilt acisi (kilitlenir)

log.t = zeros(N,1);
log.att = zeros(3,N);
log.omega = zeros(3,N);
log.T = zeros(3,N);
log.delta = zeros(3,N);
log.surf = zeros(5,N);
log.vel_body = zeros(3,N);
log.pos = zeros(3,N);

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

    if ~handoff && vel_b(1) >= V_HANDOFF
        handoff = true;
        t_handoff = t;
        delta_eq = mean(delta_act(1:2));
        fprintf('t=%.2f s: v_fwd=%.2f m/s -- GECIS: kuyruk kapatildi, on tilt esitlendi (%.1f deg)\n', ...
            t, vel_b(1), rad2deg(delta_eq));
    end
    if handoff
        % Esitlenmis ekseni sabit tutmak yerine, ikisini birlikte (senkron)
        % yataya dogru rampala -- TILT_RAMP_RATE ile, p.tilt.max'i asmadan.
        delta_eq = min(delta_eq + TILT_RAMP_RATE*dt_ctrl, p.tilt.max);
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

    [T_cmd, delta_cmd, ctrl_state, diagn, surf_cmd] = indi_attitude_controller( ...
        att_sp, att, omega, omega_dot_filt, F_sp, u_actual9, ctrl_state, p, leso_axes, [], qbar);

    if handoff
        T_cmd(3) = 0;                    % kuyruk kapali
        delta_cmd(1:2) = [delta_eq; delta_eq];  % on tilt esitlenmis, senkron
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
end

idx_post = log.t >= t_handoff;
fprintf('\nGecis ani: t=%.2f s\n', t_handoff);
fprintf('Gecis SONRASI max |pitch|: %.2f deg, max |roll|: %.2f deg\n', ...
    max(abs(rad2deg(log.att(2,idx_post)))), max(abs(rad2deg(log.att(1,idx_post)))));
fprintf('Gecis sonrasi max |omega| = %.4f rad/s\n', max(vecnorm(log.omega(:,idx_post))));
fprintf('Son durum: ileri hiz=%.2f m/s, irtifa sapmasi=%.2f m\n', log.vel_body(1,end), -log.pos(3,end)-80);

fig = figure('Position',[100 100 1100 900]);
subplot(3,2,1); hold on; grid on;
plot(log.t, rad2deg(log.att(1,:)), 'DisplayName','\phi'); plot(log.t, rad2deg(log.att(2,:)), 'DisplayName','\theta');
xline(t_handoff,'r--'); ylabel('deg'); legend('Location','best'); title('Attitude (kirmizi=gecis ani)');

subplot(3,2,2); hold on; grid on;
plot(log.t, log.T(1,:)); plot(log.t, log.T(2,:)); plot(log.t, log.T(3,:));
xline(t_handoff,'r--'); ylabel('N'); title('Rotor itkileri');

subplot(3,2,3); hold on; grid on;
plot(log.t, rad2deg(log.delta(1,:))); plot(log.t, rad2deg(log.delta(2,:))); plot(log.t, rad2deg(log.delta(3,:)));
xline(t_handoff,'r--'); ylabel('deg'); title('Tilt acilari');

subplot(3,2,4); hold on; grid on;
plot(log.t, rad2deg(log.surf(1,:)), 'DisplayName','elevon'); plot(log.t, rad2deg(log.surf(3,:)), 'DisplayName','elevator');
plot(log.t, rad2deg(log.surf(5,:)), 'DisplayName','rudder');
xline(t_handoff,'r--'); ylabel('deg'); legend('Location','best'); title('Yuzey sapmalari');

subplot(3,2,5); hold on; grid on;
plot(log.t, log.vel_body(1,:)); xline(t_handoff,'r--'); ylabel('m/s'); xlabel('t (s)'); title('Ileri hiz');

subplot(3,2,6); hold on; grid on;
plot(log.t, -log.pos(3,:)-80); xline(t_handoff,'r--'); ylabel('irtifa sapmasi (m)'); xlabel('t (s)'); title('Irtifa');

saveas(fig, fullfile(fileparts(mfilename('fullpath')), 'handoff_ramp_slowrate_test.png'));
fprintf('\nGrafik kaydedildi: handoff_ramp_slowrate_test.png\n');
