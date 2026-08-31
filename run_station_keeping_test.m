%% RUN_STATION_KEEPING_TEST
% Madde (N) testi: yatay pozisyon dongusu VAR/YOK A/B'si + GERCEK durus
% hover'inda yaw davranisi (2026-07-29, Adim 28).
%
% NEDEN BU TEST VAR:
% Rapor §4 (N): tum tiltler [0, pi/2] araliginda oldugu icin hover'da toplam
% Fx >= 0; yaw trim'i ~3 N ileri itki doguruyor ve YATAY POZISYON DONGUSU
% OLMADIGI icin arac surukleniyor (SITL'de 25 s'de 235 m olculdu). Adim 16'nin
% kritik sonucu: bu yuzden simdiye kadarki HER "hover" dogrulamasi aslinda
% ~10 m/s seyir testiydi ve yaw'in EN KOTU kosulu olan gercek durus hicbir
% zaman test edilmedi.
%
% Bu test iki seyi olcer:
%   1) pozisyon dongusu suruklenmeyi durduruyor mu (A/B),
%   2) GERCEK durusta (v_h ~ 0) yaw adim yaniti nasil -- saf MATLAB burada
%      yapisal olarak en kotu durum, cunku M_aero(3) = 0 (Adim 17), yani
%      hicbir aerodinamik yaw sonumlemesi yok.
%
% Kullanim:  run_station_keeping_test
% Cikti: konsol metrikleri + station_keeping_test.png

clear; clc;
addpath(fileparts(mfilename('fullpath')));

p = tiltrotor_params();

Tsim    = 40.0;
t_step  = 15.0;                  % s, yaw adimi (durus oturduktan SONRA)
yaw_steps_deg = [30, -30];   % madde (P): iki yon de kosulur (asimetri olcumu)
leso_axes = [true; true; false];

dt_ctrl = p.Ts_ctrl; n_sub = 5; dt_phys = dt_ctrl/n_sub;
N = round(Tsim/dt_ctrl);

u_trim = hover_trim(p);
x0 = zeros(19,1);
x0(3)     = -50;
x0(7:10)  = quat_from_euler(0,0,0);
x0(14:19) = u_trim;

cfgs = {struct('name','pos.dongusu KAPALI','pos_loop',false), ...
        struct('name','pos.dongusu ACIK (N+P cozumu)', 'pos_loop',true)};

results = struct();

for c = 1:numel(cfgs)
  for si = 1:numel(yaw_steps_deg)
    cfg = cfgs{c};

    x = x0;
    ctrl_state = init_ctrl_state();
    omega_dot_filt = zeros(3,1);
    alt_state = 0; alt_accum = 0; pos_state = zeros(2,1);
    Fz_sp = -p.m*p.g;
    att_sp_xy = [0;0];
    fx_sp = 0;   % pozisyon dongusu kapaliyken trim yok (madde (P) -> (N) bagimliligi)

    pos_sp = x0(1:2);
    z_sp   = x0(3);

    log.t     = zeros(N,1);
    log.pos   = zeros(2,N);
    log.vh    = zeros(N,1);
    log.att   = zeros(3,N);
    log.attsp = zeros(3,N);
    log.omega = zeros(3,N);
    log.delta = zeros(3,N);
    log.z     = zeros(N,1);

    for k = 1:N
        t = (k-1)*dt_ctrl;

        att   = quat_to_euler(x(7:10));
        omega = x(11:13);
        u_actual = x(14:19);

        xdot_now = tiltrotor_plant_deriv(x, u_actual(1:3), u_actual(4:6), p, [0;0;0], [0;0;0]);
        omega_dot_filt = omega_dot_filt + 0.3*(xdot_now(11:13) - omega_dot_filt);

        % --- dis donguler (Ts_pos decimasyonu) ---
        alt_accum = alt_accum + dt_ctrl;
        if alt_accum >= p.Ts_pos - 1e-12
            [Fz_sp, alt_state] = altitude_loop(z_sp, x(3), xdot_now(3), alt_state, p);
            if cfg.pos_loop
                [att_sp_xy, fx_sp, pos_state] = position_loop(pos_sp, x(1:2), xdot_now(1:2), ...
                                                              att(3), mean(u_actual(4:6)), pos_state, p);
            end
            alt_accum = alt_accum - p.Ts_pos;
        end

        psi_sp = 0;
        if t >= t_step, psi_sp = deg2rad(yaw_steps_deg(si)); end
        att_sp = [att_sp_xy(1); att_sp_xy(2); psi_sp];

        [T_cmd, delta_cmd, ctrl_state, ~] = indi_attitude_controller( ...
            att_sp, att, omega, omega_dot_filt, [fx_sp; Fz_sp], u_actual, ctrl_state, p, leso_axes);

        for s = 1:n_sub
            x = rk4_step(@(xx) tiltrotor_plant_deriv(xx, T_cmd, delta_cmd, p, [0;0;0], [0;0;0]), x, dt_phys);
        end

        log.t(k)       = t;
        log.pos(:,k)   = x(1:2);
        log.vh(k)      = norm(xdot_now(1:2));
        log.att(:,k)   = att;
        log.attsp(:,k) = att_sp;
        log.omega(:,k) = omega;
        log.delta(:,k) = delta_cmd;
        log.z(k)       = x(3);
    end

    results.(sprintf('cfg%d_s%d',c,si)) = log;
    fprintf('[%s, %+d deg] kosu tamamlandi.\n', cfg.name, yaw_steps_deg(si));
  end
end

%% --- Metrikler ---
fprintf('\n=== DURUS (STATION KEEPING) TESTI — saf MATLAB, aero yaw sonumlemesi YOK ===\n');
fprintf('%-26s %6s %9s %9s %9s %9s %9s %9s\n', 'konfig', 'adim', 'maxsuruk', 'v_h(adim)', ...
        'asim%', 'ts2deg', 'RMSr_son', 'asimetri');
fprintf('%s\n', repmat('-', 1, 100));

for c = 1:numel(cfgs)
    ovs = zeros(1, numel(yaw_steps_deg));
    rows = cell(1, numel(yaw_steps_deg));
    for si = 1:numel(yaw_steps_deg)
        log = results.(sprintf('cfg%d_s%d',c,si));
        d   = sqrt(sum((log.pos - x0(1:2)).^2, 1)).';
        idx = log.t >= t_step;
        tt  = log.t(idx); psi = log.att(3,idx).'; r = log.omega(3,idx).';
        psi_sp = deg2rad(yaw_steps_deg(si));
        if psi_sp > 0, o = max(0, max(psi) - psi_sp); else, o = max(0, psi_sp - min(psi)); end
        ovs(si) = 100*o/abs(psi_sp);
        e = psi - psi_sp; ok = abs(e) <= deg2rad(2); ts = NaN;
        for i = 1:numel(ok)
            if all(ok(i:end)), ts = tt(i) - t_step; break; end
        end
        rows{si} = {max(d), log.vh(find(log.t >= t_step, 1)), ovs(si), ts, ...
                    sqrt(mean(r(tt >= tt(end)-5).^2))};
    end
    asym = max(ovs)/max(min(ovs), 1e-9);
    for si = 1:numel(yaw_steps_deg)
        rr = rows{si};
        lbl = ''; as_s = '';
        if si == 1, lbl = cfgs{c}.name; as_s = sprintf('%.2fx', asym); end
        fprintf('%-26s %+6d %8.2fm %9.2f %8.1f%% %9.2f %9.5f %9s\n', lbl, yaw_steps_deg(si), ...
                rr{1}, rr{2}, rr{3}, rr{4}, rr{5}, as_s);
    end
end

fprintf('\nmaxsuruk = baslangic noktasindan max yatay uzaklik (madde N).\n');
fprintf('asimetri = max(asim)/min(asim), madde (P) olcusu (Adim 17''de 7.4x idi).\n');
fprintf('v_h(adim) = yaw adimi anindaki yatay hiz — GERCEK durus icin ~0 olmali.\n');

%% --- Grafik ---
fig = figure('Position',[100 100 1100 800],'Visible','off');
ttl = {'KAPALI','ACIK'};
for c = 1:numel(cfgs)
    log = results.(sprintf('cfg%d_s1',c));
    subplot(3,2,c);
    plot(log.pos(1,:), log.pos(2,:), 'LineWidth',1.2); hold on;
    plot(x0(1), x0(2), 'ro','MarkerFaceColor','r');
    xlabel('x_{NED} (m)'); ylabel('y_{NED} (m)'); grid on; axis equal;
    title(sprintf('Yer izi — pozisyon dongusu %s', ttl{c}));

    subplot(3,2,2+c);
    plot(log.t, log.vh, 'LineWidth',1.1); grid on;
    xlabel('t (s)'); ylabel('|v_h| (m/s)');
    title(sprintf('Yatay hiz — %s', ttl{c}));

    subplot(3,2,4+c);
    plot(log.t, rad2deg(log.att(3,:)), 'LineWidth',1.1); hold on;
    plot(log.t, rad2deg(log.attsp(3,:)), '--', 'LineWidth',1.0);
    grid on; xlabel('t (s)'); ylabel('yaw (deg)'); legend('yaw','yaw_{sp}','Location','best');
    title(sprintf('Yaw — %s', ttl{c}));
end
saveas(fig, fullfile(fileparts(mfilename('fullpath')), 'station_keeping_test.png'));
close(fig);
fprintf('\nGrafik kaydedildi: station_keeping_test.png\n');
