%% RUN_POSHOLD_CLIMB_GAIN_SWEEP
% Adim 90'in devami: pos_hold+tirmanma senaryosunda SITL'de gozlenen PITCH
% limit-cycle salinimini (7.3-10.9 deg, ~2-2.5s periyotlu) MATLAB'da yeniden
% uretmeye calisir -- position_loop.m'in hiz-dongusu kazanclarini (Kp_v, Ki_v)
% taban degerlerinin katlari olarak tarayarak. Amac: MATLAB'in kararlilik
% payini olcmek -- eger salinim yalnizca kucuk bir carpanda (~1-2x) baslarsa,
% SITL'nin gercek (modellenmemis gecikme/faz kaybi olan) dinamigi bu ince
% payi yiyip ayni nominal kazanclarda kararsizlanmis demektir. Eger salinim
% cok buyuk carpanlarda (~10x+) bile hic baslamiyorsa, sorun kazanc payi
% degil, MATLAB'da hic olmayan bir dinamik (gercek zaman gecikmesi, sensor
% gurultusu, dt jitter) demektir.
%
% position_loop.m'in kendisi DEGISTIRILMEDI -- kazanclar burada satir-ici
% kopyalanip carpan uygulaniyor, boylece dogrulanmis fonksiyon dokunulmadan
% kaliyor.
%
% Kullanim: run_poshold_climb_gain_sweep

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

mults = [0.3 0.5 0.7 1 2];

fprintf('\n=== pos_hold+tirmanma, Kp_v/Ki_v carpan taramasi (DUSUK carpanlar, Adim 91) ===\n');
fprintf('%8s %10s %10s %10s %12s %10s\n', 'carpan', 'pitch_min', 'pitch_max', 'p2p(deg)', 'q_RMS_son10s', 'maxsurukl');
fprintf('%s\n', repmat('-', 1, 68));

for mi = 1:numel(mults)
    mult = mults(mi);

    x0 = zeros(19,1);
    x0(3)     = 0;
    x0(7:10)  = quat_from_euler(0,0,deg2rad(90));
    x0(14:19) = u_trim;

    x = x0;
    ctrl_state = init_ctrl_state();
    omega_dot_filt = zeros(3,1);
    alt_state = 0; alt_accum = 0;
    Fz_sp = -p.m*p.g;
    att_sp_xy = [0;0];
    fx_sp = 0;
    z_sp = 0;

    % --- position_loop.m'in ici, kazanclar carpanli ---
    v_max     = 3.0;
    Kp_p      = 0.80;
    Kp_v      = 2.00 * mult;
    Ki_v      = 0.40 * mult;
    int_max   = 2.0;
    a_max     = 3.0;
    tilt_max  = deg2rad(15);
    int_v = zeros(2,1);
    pos_sp = x0(1:2);

    log.t = zeros(N,1); log.att = zeros(3,N); log.omega = zeros(3,N); log.z = zeros(N,1); log.pos = zeros(2,N);

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

            % --- position_loop.m inline, Kp_v/Ki_v carpanli ---
            err_p = pos_sp(:) - x(1:2);
            v_sp  = Kp_p * err_p;
            nv = norm(v_sp);
            if nv > v_max, v_sp = v_sp * (v_max/nv); end
            err_v = v_sp - xdot_now(1:2);
            int_v = int_v + err_v * p.Ts_pos;
            int_v = max(min(int_v, int_max), -int_max);
            a_ned = Kp_v*err_v + Ki_v*int_v;
            na = norm(a_ned);
            if na > a_max, a_ned = a_ned * (a_max/na); end
            psi_now = att(3);
            c = cos(psi_now); s = sin(psi_now);
            ax_b =  a_ned(1)*c + a_ned(2)*s;
            ay_b = -a_ned(1)*s + a_ned(2)*c;
            theta_sp = -atan2(ax_b, p.g);
            phi_sp   =  atan2(ay_b, p.g);
            att_sp_xy(1) = max(min(phi_sp, tilt_max), -tilt_max);
            att_sp_xy(2) = max(min(theta_sp, tilt_max), -tilt_max);

            delta_bar = mean(u_actual(4:6));
            s_sched  = max(0, min(1, delta_bar/p.tilt.max));
            w_sched  = 3*s_sched.^2 - 2*s_sched.^3;
            fx_sp    = p.ctrl.fx_trim * (1 - w_sched);

            alt_accum = alt_accum - p.Ts_pos;
        end

        psi_sp = deg2rad(90);
        att_sp = [att_sp_xy(1); att_sp_xy(2); psi_sp];

        [T_cmd, delta_cmd, ctrl_state, ~] = indi_attitude_controller( ...
            att_sp, att, omega, omega_dot_filt, [fx_sp; Fz_sp], u_actual, ctrl_state, p, leso_axes);

        for s = 1:n_sub
            x = rk4_step(@(xx) tiltrotor_plant_deriv(xx, T_cmd, delta_cmd, p, [0;0;0], [0;0;0]), x, dt_phys);
        end

        log.t(k) = t; log.att(:,k) = att; log.omega(:,k) = omega; log.z(k) = x(3); log.pos(:,k) = x(1:2);
    end

    pitch_deg = rad2deg(log.att(2,:));
    tail_mask = log.t >= Tsim-10;
    p2p = max(pitch_deg(tail_mask)) - min(pitch_deg(tail_mask));
    q_rms = sqrt(mean(log.omega(2,tail_mask).^2));
    max_drift = max(sqrt(sum(log.pos.^2,1)));

    fprintf('%7.1fx %10.2f %10.2f %10.2f %13.5f %9.3fm\n', mult, min(pitch_deg(tail_mask)), ...
            max(pitch_deg(tail_mask)), p2p, q_rms, max_drift);
end

fprintf('\np2p(deg) = son 10s penceresinde pitch tepe-tepe genligi (SITL''de gozlenen ~3.6 deg ile kiyaslayin).\n');
fprintf('q_RMS_son10s = son 10s pitch hizi RMS (SITL''de bu pencerede buyuklugu ~0.2-0.3 rad/s idi).\n');
fprintf('maxsurukl = tum kosu boyunca orijinden max yatay uzaklik (pos_hold performansi).\n');
