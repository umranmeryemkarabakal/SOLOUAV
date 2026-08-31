%% RUN_HOVER_GUST_TEST
% Hover'da INDI + WLS + LESO testi: t=4s'de (a) roll ekseninde yavas degisen
% sentetik bir bozucu moment (orn. kutle asimetrisi/rotor performans farki)
% ve (b) bir ruzgar esintisi (pitch ekseninde aerodinamik bozucu, kontrolcunun
% BILMEDIGI SDF-tabanli aero modelinden geliyor) uygulanir. LESO acik/kapali
% iki senaryo karsilastirilarak roll/pitch rate hatasindaki iyilesme gosterilir.

clear; clc;
addpath(fileparts(mfilename('fullpath')));

p = tiltrotor_params();

%% --- Hover trim (bkz. hover_trim.m) ---
u_trim = hover_trim(p);

x0 = zeros(19,1);
x0(3)     = -50;                          % 50 m irtifa (NED, z=-50)
x0(7:10)  = quat_from_euler(0,0,0);
x0(14:19) = u_trim;

att_sp = [0;0;0];
z_sp   = x0(3);                            % irtifayi baslangic degerinde tut
Fx_sp  = 0;

Tsim   = 12;
dt_ctrl = p.Ts_ctrl;
n_sub   = 5;
dt_phys = dt_ctrl/n_sub;
N = round(Tsim/dt_ctrl);

configs = struct('name', {'LESO acik (roll+pitch)', 'LESO kapali'}, ...
                  'leso_axes', {[true;true;false], [false;false;false]});

results = struct();

for cfg = 1:numel(configs)
    x = x0;
    ctrl_state = init_ctrl_state();
    omega_dot_filt = zeros(3,1);
    leso_axes = configs(cfg).leso_axes;
    alt_state = 0;          % irtifa donguesu integral durumu
    alt_accum = 0;          % Ts_pos decimasyon birikici
    Fz_sp = -p.m*p.g;       % ilk deger (trim)

    log.t     = zeros(N,1);
    log.att   = zeros(3,N);
    log.omega = zeros(3,N);
    log.dhat  = zeros(3,N);
    log.Tcmd  = zeros(3,N);
    log.delta = zeros(3,N);
    log.extm  = zeros(3,N);

    for k = 1:N
        t = (k-1)*dt_ctrl;

        % --- Bozucu programı (gust 1 s icinde rampa ile devreye girer) ---
        if t < 4
            wind_ned = [0;0;0];
            ext_m = [0;0;0];
        else
            ramp = min(1, (t-4)/1.0);
            wind_ned = ramp * [-5; 0; -0.8];                          % basit gust (bkz. dosya notu)
            ext_m = [ramp*(0.4 + 0.15*sin(2*pi*0.3*(t-4))); 0; 0];    % yavas degisen roll bozucusu
        end

        att   = quat_to_euler(x(7:10));
        omega = x(11:13);
        u_actual = x(14:19);

        % --- "Sensor" tarafi: anlik omega_dot (gercek turev + tek kutuplu filtre) ---
        xdot_now = tiltrotor_plant_deriv(x, u_actual(1:3), u_actual(4:6), p, wind_ned, ext_m);
        omega_dot_raw = xdot_now(11:13);
        omega_dot_filt = omega_dot_filt + 0.3*(omega_dot_raw - omega_dot_filt);

        % --- Irtifa dis dongusu (decimasyonlu, Ts_pos = 1/50 s) ---
        alt_accum = alt_accum + dt_ctrl;
        if alt_accum >= p.Ts_pos - 1e-12
            vz_ned = xdot_now(3);   % = (R_eb*vel_body)_z, dogrudan pos_dot(3)
            [Fz_sp, alt_state] = altitude_loop(z_sp, x(3), vz_ned, alt_state, p);
            alt_accum = alt_accum - p.Ts_pos;
        end
        F_sp = [Fx_sp; Fz_sp];

        [T_cmd, delta_cmd, ctrl_state, diagn] = indi_attitude_controller( ...
            att_sp, att, omega, omega_dot_filt, F_sp, u_actual, ctrl_state, p, leso_axes); %#ok<ASGLU>

        for s = 1:n_sub
            x = rk4_step(@(xx) tiltrotor_plant_deriv(xx, T_cmd, delta_cmd, p, wind_ned, ext_m), x, dt_phys);
        end

        log.t(k)       = t;
        log.att(:,k)   = att;
        log.omega(:,k) = omega;
        log.dhat(:,k)  = ctrl_state.d_hat;
        log.Tcmd(:,k)  = T_cmd;
        log.delta(:,k) = delta_cmd;
        log.extm(:,k)  = ext_m;
    end

    results.(sprintf('cfg%d', cfg)) = log;
    fprintf('[%s] tamamlandi.\n', configs(cfg).name);
end

%% --- Metrikler: bozucu penceresinde (t>=4) RMS roll/pitch rate hatasi ---
for cfg = 1:numel(configs)
    log = results.(sprintf('cfg%d', cfg));
    idx = log.t >= 4;
    rms_p = sqrt(mean(log.omega(1,idx).^2));
    rms_q = sqrt(mean(log.omega(2,idx).^2));
    fprintf('%-28s  RMS p = %.4f rad/s   RMS q = %.4f rad/s\n', configs(cfg).name, rms_p, rms_q);
end

%% --- Grafikler ---
fig = figure('Position',[100 100 1100 800]);
labels = {'p (roll rate)','q (pitch rate)','r (yaw rate)'};
colors = {[0.85 0.2 0.2],[0.2 0.4 0.8]};
for ax = 1:2
    subplot(3,1,ax); hold on; grid on;
    for cfg = 1:numel(configs)
        log = results.(sprintf('cfg%d', cfg));
        plot(log.t, log.omega(ax,:), 'Color', colors{cfg}, 'LineWidth', 1.4, ...
             'DisplayName', configs(cfg).name);
    end
    xline(4,'k--','gust baslangici','HandleVisibility','off');
    ylabel([labels{ax} ' (rad/s)']); legend('Location','best');
    title(labels{ax});
end
subplot(3,1,3); hold on; grid on;
log1 = results.cfg1;
plot(log1.t, log1.dhat(1,:), 'Color', colors{1}, 'LineWidth', 1.4, 'DisplayName','d\_hat roll (LESO)');
plot(log1.t, log1.dhat(2,:), 'Color', colors{2}, 'LineWidth', 1.4, 'DisplayName','d\_hat pitch (LESO)');
plot(log1.t, log1.extm(1,:)/p.I(1,1), 'k--', 'LineWidth', 1.0, 'DisplayName','gercek roll bozucu ivmesi');
ylabel('rad/s^2'); xlabel('t (s)'); legend('Location','best');
title('LESO bozucu tahmini (d\_hat) vs gercek');

saveas(fig, fullfile(fileparts(mfilename('fullpath')), 'hover_gust_test.png'));
fprintf('\nGrafik kaydedildi: hover_gust_test.png\n');
