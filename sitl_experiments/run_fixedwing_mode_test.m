%% RUN_FIXEDWING_MODE_TEST
% Yeni fixedwing_control_law.m'i, gecisten BAGIMSIZ olarak, dogrudan sabit
% kanat konfigurasyonunda (kanat rotorleri 90 deg tilt kilitli, kuyruk
% kapali, 16 m/s baslangic hizi) test eder. Once duz ucus tutabiliyor mu,
% sonra kucuk bir roll/yaw komutuyla tepkisini olcer.

clear; clc;
addpath(fileparts(mfilename('fullpath')));

p = tiltrotor_params();

x0 = zeros(24,1);
x0(3)     = -80;
x0(4)     = 16.0;                    % 16 m/s ileri hizla basla (zaten cruise)
x0(7:10)  = quat_from_euler(0,deg2rad(1.25),0);  % trim aciyla basla
x0(14:16) = [4.37; 4.37; 0];          % olculmus trim itkisiyle basla
x0(17:19) = [p.tilt.max; p.tilt.max; 0];  % kanat tilt kilitli 90 deg, kuyruk dik (guc yok)

z_sp = x0(3);
V_SP = 16.0;
Tsim = 30;   % Adim 75: uzatildi, banka-donus sonrasi duz ucusa donusu de gormek icin
dt_ctrl = p.Ts_ctrl;
n_sub = 5;
dt_phys = dt_ctrl/n_sub;
N = round(Tsim/dt_ctrl);

x = x0;
state_v = [0; 0];

log.t = zeros(N,1);
log.att = zeros(3,N);
log.omega = zeros(3,N);
log.T = zeros(3,N);
log.surf = zeros(5,N);
log.vel_body = zeros(3,N);
log.pos = zeros(3,N);

for k = 1:N
    t = (k-1)*dt_ctrl;

    wind_ned = [0;0;0];
    ext_m = [0;0;0];

    att = quat_to_euler(x(7:10));
    omega = x(11:13);
    vel_b = x(4:6);
    V = norm(vel_b);
    qbar = 0.5 * p.rho * V^2;

    % Adim 75: MIMARI DUZELTMESI -- artik dogrudan roll_sp komut edilmiyor
    % (bkz. fixedwing_control_law.m basligi). Maniivra testi artik bir
    % HEADING DEGISIKLIGI komut ederek yapiliyor -- ucak bunu banka yaparak
    % (bank-to-turn) yakalamali, sonra kanatlari duzeltip yeni headingde
    % duz ucusa donmeli.
    if t >= 8
        hdg_sp = deg2rad(20);
    else
        hdg_sp = 0;
    end

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
    log.T(:,k) = x(14:16);
    log.surf(:,k) = x(20:24);
    log.vel_body(:,k) = vel_b;
    log.pos(:,k) = x(1:3);
end

fprintf('Duz ucus fazi (t<8s) max |pitch|: %.2f deg, max |roll|: %.2f deg\n', ...
    max(abs(rad2deg(log.att(2, log.t<8)))), max(abs(rad2deg(log.att(1, log.t<8)))));
fprintf('Heading komutu (20 deg, t=8s+) sirasindaki max |roll| (banka): %.2f deg\n', ...
    max(abs(rad2deg(log.att(1, log.t>=8)))));
fprintf('Son yaw: %.2f deg (hedef 20), son roll: %.2f deg (0''a donmus olmali)\n', ...
    rad2deg(log.att(3,end)), rad2deg(log.att(1,end)));
fprintf('Genel max |pitch|: %.2f deg, max |roll|: %.2f deg\n', max(abs(rad2deg(log.att(2,:)))), max(abs(rad2deg(log.att(1,:)))));
fprintf('Genel max |omega| = %.4f rad/s\n', max(vecnorm(log.omega)));
fprintf('Son hiz: %.2f m/s, irtifa sapmasi: %.2f m\n', log.vel_body(1,end), -log.pos(3,end)-80);

fig = figure('Position',[100 100 1100 900]);
subplot(3,2,1); hold on; grid on;
plot(log.t, rad2deg(log.att(1,:)), 'DisplayName','\phi'); plot(log.t, rad2deg(log.att(2,:)), 'DisplayName','\theta');
ylabel('deg'); legend('Location','best'); title('Attitude');

subplot(3,2,2); hold on; grid on;
plot(log.t, log.T(1,:)); plot(log.t, log.T(2,:));
ylabel('N'); title('Kanat rotor itkileri');

subplot(3,2,3); hold on; grid on;
plot(log.t, rad2deg(log.surf(1,:)), 'DisplayName','elevon'); plot(log.t, rad2deg(log.surf(3,:)), 'DisplayName','elevator');
plot(log.t, rad2deg(log.surf(5,:)), 'DisplayName','rudder');
ylabel('deg'); legend('Location','best'); title('Yuzey sapmalari');

subplot(3,2,4); hold on; grid on;
plot(log.t, log.vel_body(1,:)); ylabel('m/s'); title('Ileri hiz');

subplot(3,2,5); hold on; grid on;
plot(log.t, -log.pos(3,:)-80); ylabel('irtifa sapmasi (m)'); xlabel('t (s)'); title('Irtifa');

subplot(3,2,6); hold on; grid on;
plot(log.t, rad2deg(log.omega(1,:)), 'DisplayName','p'); plot(log.t, rad2deg(log.omega(2,:)), 'DisplayName','q');
ylabel('deg/s'); xlabel('t (s)'); legend('Location','best'); title('Acisal hizlar');

saveas(fig, fullfile(fileparts(mfilename('fullpath')), 'fixedwing_mode_TRIMMED_test.png'));
fprintf('\nGrafik kaydedildi: fixedwing_mode_TRIMMED_test.png\n');
