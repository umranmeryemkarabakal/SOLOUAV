%% RUN_POSHOLD_CLIMB_JERK_SWEEP
% Adim 96: GENISLETILMIS (tilt-reaksiyon torklu, Adim 96'da tiltrotor_plant_deriv.m'e
% eklenen fizik) plant ile pos_hold+tirmanma senaryosunda `tilt_jerk_limit`'i
% tarar. Amac: SITL'de GUVENSIZ olan canli tarama (Adim 95: 0.8=pitch duzeldi
% ama pozisyon kacti, 1.5=daha da kotu) yerine, MATLAB'da GUVENLE bircok
% deger deneyip hem pitch/yaw stabilitesini HEM pozisyon-tutma performansini
% birlikte olcerek makul bir SITL baslangic degeri onermek.
%
% Kullanim: run_poshold_climb_jerk_sweep

clear; clc;
addpath(fileparts(mfilename('fullpath')));

p = tiltrotor_params();

Tsim    = 40.0;
leso_axes = [true; true; false];

dt_ctrl = p.Ts_ctrl; n_sub = 5; dt_phys = dt_ctrl/n_sub;
N = round(Tsim/dt_ctrl);

u_trim = hover_trim(p);

z_targets = [-3 -10 -18 -25 -30];
z_step_t  = [5 10 15 20 25];

jerks = [0.45, 0.40, 0.35, 0.30, 0.25, 0.20, 0.15];  % rad/s; Adim 96b -- 0.3 civari ince tarama

fprintf('\n=== GENISLETILMIS plant (tilt-reaksiyon torklu), tiltjerk taramasi ===\n');
fprintf('%10s %10s %10s %10s %10s %10s\n', 'jerk', 'pitch_p2p', 'q_RMS', 'r_RMS', 'maxsurukl', 'sonalt(m)');
fprintf('%s\n', repmat('-', 1, 65));

for ji = 1:numel(jerks)
    jerk = jerks(ji);

    x0 = zeros(22,1);
    x0(3)     = 0;
    x0(7:10)  = quat_from_euler(0,0,deg2rad(90));
    x0(14:19) = u_trim;

    x = x0;
    ctrl_state = init_ctrl_state();
    omega_dot_filt = zeros(3,1);
    alt_state = 0; alt_accum = 0; pos_state = zeros(2,1);
    Fz_sp = -p.m*p.g;
    att_sp_xy = [0;0];
    fx_sp = 0;
    pos_sp = x0(1:2);
    z_sp = 0;

    log.t = zeros(N,1); log.att = zeros(3,N); log.omega = zeros(3,N);
    log.z = zeros(N,1); log.pos = zeros(2,N);

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

        psi_sp = deg2rad(90);
        att_sp = [att_sp_xy(1); att_sp_xy(2); psi_sp];

        [T_cmd, delta_cmd, ctrl_state, ~] = indi_attitude_controller( ...
            att_sp, att, omega, omega_dot_filt, [fx_sp; Fz_sp], u_actual, ctrl_state, p, ...
            leso_axes, [], [], jerk);

        for s = 1:n_sub
            x = rk4_step(@(xx) tiltrotor_plant_deriv(xx, T_cmd, delta_cmd, p, [0;0;0], [0;0;0]), x, dt_phys);
        end

        log.t(k) = t; log.att(:,k) = att; log.omega(:,k) = omega; log.z(k) = x(3); log.pos(:,k) = x(1:2);
    end

    pitch_deg = rad2deg(log.att(2,:));
    tail_mask = log.t >= Tsim-10;
    p2p = max(pitch_deg(tail_mask)) - min(pitch_deg(tail_mask));
    q_rms = sqrt(mean(log.omega(2,tail_mask).^2));
    r_rms = sqrt(mean(log.omega(3,tail_mask).^2));
    max_drift = max(sqrt(sum(log.pos.^2,1)));
    son_alt = -log.z(end);

    jerk_str = 'kapali(Inf)';
    if isfinite(jerk), jerk_str = sprintf('%.2f', jerk); end
    fprintf('%10s %10.2f %10.5f %10.5f %9.3fm %9.2fm\n', jerk_str, p2p, q_rms, r_rms, max_drift, son_alt);
end

fprintf('\npitch_p2p = son 10s pitch tepe-tepe genligi (SITL, tiltjerk kapali: ~3.6 deg; tiltjerk=0.8 canli SITL''de: 1.03 deg).\n');
fprintf('maxsurukl = tum kosu boyunca orijinden max yatay uzaklik.\n');
