%% RUN_SURF_ENABLED_TRANSITION_TEST
% Kullanicinin onerisi: kuyruk motorunu HIC kapatmadan, kontrol yuzeylerini
% (elevator ozellikle) WLS'e ek kanal olarak acip, hiz arttikca kuyruk
% yukunun kendiliginden azalip azalmadigini olcer. run_transition_test.m'in
% aynisi ama:
%   - plant 24-durumlu (fiziksel yuzey aktuatorleri dahil, aero_panels.m
%     onlari GERCEKTEN hesaba katar)
%   - kontrolcuye 9 elemanli u_actual + qbar verilir (surf yolu acilir)
%   - "golge" sanal aktuator durumu (a_ail,a_ele,a_rud) mevcut mimarideki
%     T/delta golge modeliyle AYNI felsefeyle ayrica izlenir (Adim 18/24'un
%     dersi: kapali cevrimde gercek servoya baglanmak kilitlenmeye yol
%     acabiliyordu -- acik cevrim golge guvenli varsayilan)

clear; clc;
addpath(fileparts(mfilename('fullpath')));

p = tiltrotor_params();
u_trim = hover_trim(p);

x0 = zeros(24,1);          % 19 + 5 (fiziksel yuzeyler)
x0(3)     = -80;
x0(7:10)  = quat_from_euler(0,0,0);
x0(14:19) = u_trim;
% x0(20:24) = 0 (yuzeyler notrde basliyor)

att_sp = [0;0;0];
z_sp   = x0(3);
Fx_final = 10;
t_ramp   = 12;
Tsim = 20;
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
a_virt = zeros(3,1);   % golge sanal yuzey durumu [a_ail;a_ele;a_rud]

log.t = zeros(N,1);
log.att = zeros(3,N);
log.omega = zeros(3,N);
log.T = zeros(3,N);
log.delta = zeros(3,N);
log.surf = zeros(5,N);
log.vel_body = zeros(3,N);
log.pos = zeros(3,N);
log.qbar = zeros(N,1);

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

    a_virt = diagn.a_cmd;   % golge sanal durumu ilerlet (acik cevrim, T/delta ile ayni desen)

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
    log.qbar(k) = qbar;
end

fprintf('Son ortalama tilt: %.1f deg, ileri hiz: %.2f m/s, irtifa sapmasi: %.2f m\n', ...
    rad2deg(mean(log.delta(:,end))), log.vel_body(1,end), -log.pos(3,end)-80);
fprintf('Son kuyruk itkisi T2: %.2f N (baslangic hover trim T2 = %.2f N)\n', log.T(3,end), u_trim(3));
fprintf('Son elevator sapmasi (s2,s3): %.2f, %.2f deg\n', rad2deg(log.surf(3,end)), rad2deg(log.surf(4,end)));
fprintf('Max |pitch|: %.2f deg, max |roll|: %.2f deg\n', max(abs(rad2deg(log.att(2,:)))), max(abs(rad2deg(log.att(1,:)))));
fprintf('Max |omega| = %.4f rad/s\n', max(vecnorm(log.omega)));

fig = figure('Position',[100 100 1100 900]);

subplot(3,2,1); hold on; grid on;
plot(log.t, log.T(1,:), 'DisplayName','T_0'); plot(log.t, log.T(2,:), 'DisplayName','T_1');
plot(log.t, log.T(3,:), 'LineWidth',1.5, 'DisplayName','T_2 (kuyruk)');
ylabel('N'); legend('Location','best'); title('Rotor itkileri (yuzeyler ACIK)');

subplot(3,2,2); hold on; grid on;
plot(log.t, rad2deg(log.surf(3,:)), 'DisplayName','elevator (s2)');
plot(log.t, rad2deg(log.surf(1,:)), 'DisplayName','elevon/aileron (s0)');
plot(log.t, rad2deg(log.surf(5,:)), 'DisplayName','rudder (s4)');
ylabel('deg'); legend('Location','best'); title('Yuzey sapmalari');

subplot(3,2,3); hold on; grid on;
plot(log.t, rad2deg(log.att(1,:)), 'DisplayName','\phi'); plot(log.t, rad2deg(log.att(2,:)), 'DisplayName','\theta');
ylabel('deg'); legend('Location','best'); title('Attitude');

subplot(3,2,4); hold on; grid on;
plot(log.t, log.vel_body(1,:), 'DisplayName','u'); ylabel('m/s'); legend('Location','best'); title('Ileri hiz');

subplot(3,2,5); hold on; grid on;
plot(log.t, log.qbar, 'DisplayName','qbar'); ylabel('Pa'); xlabel('t (s)'); title('Dinamik basinc');

subplot(3,2,6); hold on; grid on;
plot(log.t, -log.pos(3,:)-80); ylabel('irtifa sapmasi (m)'); xlabel('t (s)'); title('Irtifa');

saveas(fig, fullfile(fileparts(mfilename('fullpath')), 'surf_enabled_transition_test.png'));
fprintf('\nGrafik kaydedildi: surf_enabled_transition_test.png\n');
