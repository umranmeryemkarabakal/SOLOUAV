%% RUN_BOX_BIND_CHECK
% MATLAB tarafinda WLS tilt slew KUTUSUNUN bagalayici olup olmadigini olcer
% ve PX4'un Adim 27'de dagitilan kutu degerinin MATLAB karsiligini test eder.
%
% NEDEN BU TEST VAR (2026-07-29, Adim 27):
% PX4'te Adim 22 `TILT_RATE_MAX`'i ikiye ayirdi:
%   - TILT_RATE_MAX      = golge modelin FIZIKSEL servo limiti
%   - TILT_SLEW_BOX_RATE = tahsisatin tek tick'te isteyebilecegi (Adim 27: 3.00)
% MATLAB'da bu ayrim YOK: `p.tilt.rate_max` ayni anda
%   - plant'in fiziksel clamp'i  (tiltrotor_plant_deriv.m:42)
%   - tahsisat kutusu            (indi_attitude_controller.m:94-95)
% olarak kullaniliyor. Yani PX4 degisikligi MATLAB'da tek sabiti oynatarak
% DOGRUDAN yeniden uretilemez -- oynatmak plant'i da degistirir.
%
% Bu script ayrimi p_ctrl/p_plant olarak YEREL emule eder (paylasilan dosyalara
% dokunmadan) ve su iki soruyu ol'cerek yanitlar:
%   1) MATLAB'da tilt kutusu hic BAGLIYOR mu? (Adim 21d'nin iddiasi: baglamiyor)
%   2) Kutuyu PX4'un yeni tick-basi degerine esitlemek MATLAB'da bir sey degistiriyor mu?
%
% Kutu karsilastirmasi (tick basina rad):
%   MATLAB  : rate * p.Ts_ctrl = rate * (1/400)
%   PX4      : TILT_SLEW_BOX_RATE * TS_BOX = 3.00 * (1/250) = 0.012
%   -> PX4 ile ayni tick-basi kutu icin MATLAB'da rate = 0.012*400 = 4.8 rad/s
%
% Kullanim:  run_box_bind_check

clear; clc;
addpath(fileparts(mfilename('fullpath')));

p0 = tiltrotor_params();

PX4_BOX_PER_TICK = 3.00 * (1/250);          % Adim 27'de dagitilan PX4 kutusu
rate_for_px4_box = PX4_BOX_PER_TICK / p0.Ts_ctrl;

% NOT (2026-07-29): Adim 27'de ayrim kalici olarak MATLAB'a tasindi, yani
% p.tilt.slew_box_rate artik gercek bir alan. Bu script onu override ederek
% tarar; p.tilt.rate_max ise plant clamp'i olarak ayri tutulur.
cfgs = { ...
    struct('name','A: eski birlesik davranis (kutu 3.0)', 'box_rate', 3.0,             'plant_rate', 3.0), ...
    struct('name','B: kutu 3.0 / plant 2.0',              'box_rate', 3.0,             'plant_rate', 2.0), ...
    struct('name','C: kutu 4.8 (=PX4 tick) / plant 2.0',  'box_rate', rate_for_px4_box,'plant_rate', 2.0), ...
    struct('name','D: MEVCUT referans (params.m)',        'box_rate', p0.tilt.slew_box_rate, 'plant_rate', p0.tilt.rate_max) ...
};

t_step = 4.0; Tsim = 25.0; yaw_steps = [30, -30];
leso_axes = [true; true; false];
dt_ctrl = p0.Ts_ctrl; n_sub = 5; dt_phys = dt_ctrl/n_sub;
N = round(Tsim/dt_ctrl);

fprintf('MATLAB kutu (tick basina): rate*Ts = rate*%.5f s\n', p0.Ts_ctrl);
fprintf('PX4 kutusu (Adim 27)     : 3.00*(1/250) = %.5f rad/tick\n', PX4_BOX_PER_TICK);
fprintf('-> ayni tick-basi kutu icin MATLAB rate = %.2f rad/s\n\n', rate_for_px4_box);

fprintf('%-42s %6s %9s %10s %9s %9s %9s\n', 'konfig', 'adim', 'kutu(rad)', 'baglama%', 'asim%', 'ts2deg', 'RMSr_son');
fprintf('%s\n', repmat('-', 1, 100));

for c = 1:numel(cfgs)
    cfg = cfgs{c};
    p_ctrl  = p0;  p_ctrl.tilt.slew_box_rate = cfg.box_rate;    % tahsisat kutusu
    p_plant = p0;  p_plant.tilt.rate_max     = cfg.plant_rate;  % fiziksel servo clamp
    box = cfg.box_rate * p0.Ts_ctrl;

    for s_i = 1:numel(yaw_steps)
        psi_sp_final = deg2rad(yaw_steps(s_i));

        u_trim = hover_trim(p0);
        x = zeros(19,1); x(3) = -50;
        x(7:10) = quat_from_euler(0,0,0); x(14:19) = u_trim;
        z_sp = x(3); Fx_sp = 0;

        ctrl_state = init_ctrl_state();
        omega_dot_filt = zeros(3,1); alt_state = 0; alt_accum = 0;
        Fz_sp = -p0.m*p0.g;

        t_log = zeros(N,1); psi_log = zeros(N,1); r_log = zeros(N,1);
        bind_log = zeros(N,1);

        for k = 1:N
            t = (k-1)*dt_ctrl;
            att_sp = [0;0;0];
            if t >= t_step, att_sp(3) = psi_sp_final; end

            att = quat_to_euler(x(7:10));
            omega = x(11:13); u_actual = x(14:19);

            xdot_now = tiltrotor_plant_deriv(x, u_actual(1:3), u_actual(4:6), p_plant, [0;0;0], [0;0;0]);
            omega_dot_filt = omega_dot_filt + 0.3*(xdot_now(11:13) - omega_dot_filt);

            alt_accum = alt_accum + dt_ctrl;
            if alt_accum >= p0.Ts_pos - 1e-12
                [Fz_sp, alt_state] = altitude_loop(z_sp, x(3), xdot_now(3), alt_state, p0);
                alt_accum = alt_accum - p0.Ts_pos;
            end

            [T_cmd, delta_cmd, ctrl_state, diagn] = indi_attitude_controller( ...
                att_sp, att, omega, omega_dot_filt, [Fx_sp; Fz_sp], u_actual, ctrl_state, p_ctrl, leso_axes);

            % tilt kanallarinda kutuya deyip demedigi (1e-9 tolerans)
            bind_log(k) = double(any(abs(abs(diagn.du(4:6)) - box) < 1e-9));

            for s = 1:n_sub
                x = rk4_step(@(xx) tiltrotor_plant_deriv(xx, T_cmd, delta_cmd, p_plant, [0;0;0], [0;0;0]), x, dt_phys);
            end

            t_log(k) = t; psi_log(k) = att(3); r_log(k) = omega(3);
        end

        idx = t_log >= t_step;
        tt = t_log(idx); psi = psi_log(idx); rr = r_log(idx);
        err = psi - psi_sp_final;
        % asim: hedefi gecen en buyuk sapmanin adim buyuklugune orani
        if psi_sp_final > 0, ov = max(0, max(psi) - psi_sp_final);
        else,                ov = max(0, psi_sp_final - min(psi)); end
        ov_pct = 100*ov/abs(psi_sp_final);
        % +-2 deg oturma
        tol = deg2rad(2); ok = abs(err) <= tol; ts = NaN;
        for i = 1:numel(ok)
            if all(ok(i:end)), ts = tt(i) - t_step; break; end
        end
        rms_tail = sqrt(mean(rr(tt >= tt(end)-5).^2));

        nm = ''; if s_i == 1, nm = cfg.name; end
        fprintf('%-42s %+6d %9.5f %9.1f%% %8.1f%% %8.2f %9.5f\n', ...
                nm, yaw_steps(s_i), box, 100*mean(bind_log(idx)), ov_pct, ts, rms_tail);
    end
end
fprintf('\nNot: "baglama%%" = adim sonrasi tilt kutusunun aktif oldugu tick orani.\n');
fprintf('Bu oran ~0 ise MATLAB bu kisiti hic gormuyor demektir; o durumda kutu\n');
fprintf('degerini MATLAB''da degistirmek YAPISAL OLARAK bir sey test etmez\n');
fprintf('(Adim 21d dersi: bir ablasyon ancak hedefledigi mekanizma o ortamda\n');
fprintf('aktifse bir seyi eler).\n');
