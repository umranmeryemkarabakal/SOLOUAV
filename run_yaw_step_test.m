%% RUN_YAW_STEP_TEST
% Yaw ADIM yaniti testi — sitl/WLS_LOCKUP_INVESTIGATION_REPORT.md §4 (Q) ve (P).
%
% NEDEN BU TEST VAR (2026-07-28):
% SITL'de Adim 16, yaw adim yanitinin ILERI HIZA bagli oldugunu olctu: 11.6 m/s'de
% +30 deg adim temiz (~%13 asim), 2.45 m/s'de +-25 deg'lik SONUMSUZ salinim. Ileri
% hizda yaw'i sonumleyen sey kontrolcu degil, dikey kuyruk/govde yuzeylerinin
% ruzgar gulu (weathervane) etkisi. Gercek, yerinde duran hover -- bir pozisyon
% kontrolcusunun komut edecegi asil durum -- yaw icin EN KOTU kosul ve orada
% kriter saglanmiyor.
%
% Bu senaryo saf MATLAB'da YAPISAL OLARAK yeniden uretilebilir, cunku
% tiltrotor_plant_deriv.m'in aero modeli tamamen boylamsaldir:
%   F_aero = Ry*[-D;0;-L]  ->  F_aero(2) = 0
%   M_aero = cross(r_cp, F_aero),  r_cp = [-0.05;0;0.05]  ->  M_aero(3) = 0
% yani MATLAB plant'inde HICBIR HIZDA aerodinamik yaw sonumlemesi yoktur. Bu
% testin gordugu yaw dinamigi, SITL'in "gercek hover" (v->0) kosuluyla ayni
% sinifta. Adim 11/12'deki "MATLAB yapisal olarak goremez" tuzaginin TERSI bir
% durum: burada MATLAB, SITL'den DAHA kotumser bir kosul temsil ediyor.
%
% Ayrica madde (P): adim yaniti YON-ASIMETRIK gozlendi (+30 deg -30 deg'den cok
% daha kotu). Muhtemel sebep tek yonlu tilt araligi (p.tilt.min = 0) ve trim'de
% delta0 ~= 0.16 rad iken +yaw icin delta1'in 0 tabanindan kaldirilmasi gerekmesi.
% Bu yuzden test her iki yonu de kosar.
%
% Kullanim:
%   run_yaw_step_test          % varsayilan: +-30 deg, 25 s
%
% Cikti: konsol metrikleri + yaw_step_test.png

clear; clc;
addpath(fileparts(mfilename('fullpath')));

p = tiltrotor_params();

% Opsiyonel gecersiz kilma. DIKKAT (2026-07-29, Adim 27): bu override artik
% TAHSISAT KUTUSUNU (p.tilt.slew_box_rate) degistirir, plant'in fiziksel servo
% clamp'ini (p.tilt.rate_max) degil — ikisi Adim 27'de ayrildi. Yaw dinamigini
% belirleyen kutudur; fiziksel clamp MATLAB'da hic baglamiyor (olculdu,
% bkz. run_box_bind_check.m). Kutuyu tarama ornegi:
%   setenv('YAW_TEST_TILT_BOX_RATE','3.0'); run_yaw_step_test
% Eski YAW_TEST_TILT_RATE_MAX adi geriye donuk uyumluluk icin hala kabul edilir
% ve ayni sekilde KUTUYA uygulanir (eski cagrilarin anlamini korumak icin:
% ayrimdan once o sabit zaten kutuyu belirliyordu).
suffix = '';
ovr = getenv('YAW_TEST_TILT_BOX_RATE');
if isempty(ovr), ovr = getenv('YAW_TEST_TILT_RATE_MAX'); end
if ~isempty(ovr)
    p.tilt.slew_box_rate = str2double(ovr);
    suffix = sprintf('_box%.1f', p.tilt.slew_box_rate);
end
fprintf('tilt tahsisat kutusu: p.tilt.slew_box_rate = %.2f rad/s (kutu = %.5f rad/tick)\n', ...
        p.tilt.slew_box_rate, p.tilt.slew_box_rate*p.Ts_ctrl);
fprintf('plant fiziksel servo clamp: p.tilt.rate_max = %.2f rad/s\n', p.tilt.rate_max);

%% --- Senaryo ---
t_step   = 4.0;                 % s, yaw adiminin uygulandigi an
Tsim     = 25.0;                % s (SITL'de 15 s'de oturmuyordu — daha uzun bak)
yaw_step_deg = [30, -30];       % madde (P): iki yon de kosulur
leso_axes = [true; true; false];% uretim konfigurasyonu (yaw'da LESO ASLA acilmaz,
                                % bkz. rapor cikarim 19 — 5 s'de araci ters cevirdi)

dt_ctrl = p.Ts_ctrl;
n_sub   = 5;
dt_phys = dt_ctrl/n_sub;
N = round(Tsim/dt_ctrl);

%% --- Hover trim ---
u_trim = hover_trim(p);

x0 = zeros(19,1);
x0(3)     = -50;                % 50 m irtifa (NED)
x0(7:10)  = quat_from_euler(0,0,0);
x0(14:19) = u_trim;

z_sp  = x0(3);
Fx_sp = 0;

results = struct();

for cfg = 1:numel(yaw_step_deg)
    psi_sp_final = deg2rad(yaw_step_deg(cfg));

    x = x0;
    ctrl_state = init_ctrl_state();
    omega_dot_filt = zeros(3,1);
    alt_state = 0;
    alt_accum = 0;
    Fz_sp = -p.m*p.g;

    log.t       = zeros(N,1);
    log.att     = zeros(3,N);
    log.omega   = zeros(3,N);
    log.omegasp = zeros(3,N);
    log.Tcmd    = zeros(3,N);
    log.delta   = zeros(3,N);
    log.tau_dem = zeros(N,1);   % istenen yaw tork artisi  nu_des(3)
    log.tau_ach = zeros(N,1);   % tahsisatin urettigi      (G*du)(3)
    log.vh      = zeros(N,1);   % yatay hiz buyuklugu (NED) — baglam degiskeni
    log.satf    = zeros(N,1);

    for k = 1:N
        t = (k-1)*dt_ctrl;

        att_sp = [0; 0; 0];
        if t >= t_step
            att_sp(3) = psi_sp_final;
        end

        % Bozucu yok: bu test yalnizca yaw adim yanitini olcuyor.
        wind_ned = [0;0;0];
        ext_m    = [0;0;0];

        att   = quat_to_euler(x(7:10));
        omega = x(11:13);
        u_actual = x(14:19);

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

        % Yaw ekseni atifi: talep edilen vs gerceklesen tork artisi
        G_now = effectiveness_matrix(u_actual, p);
        nu_ach = G_now * diagn.du;

        for s = 1:n_sub
            x = rk4_step(@(xx) tiltrotor_plant_deriv(xx, T_cmd, delta_cmd, p, wind_ned, ext_m), x, dt_phys);
        end

        log.t(k)         = t;
        log.att(:,k)     = att;
        log.omega(:,k)   = omega;
        log.omegasp(:,k) = diagn.omega_sp;
        log.Tcmd(:,k)    = T_cmd;
        log.delta(:,k)   = delta_cmd;
        log.tau_dem(k)   = diagn.nu_des(3);
        log.tau_ach(k)   = nu_ach(3);
        log.vh(k)        = norm(xdot_now(1:2));
        log.satf(k)      = double(any(diagn.sat_flag(:) ~= 0));
    end

    results.(sprintf('cfg%d', cfg)) = log;
    fprintf('[yaw_sp = %+d deg] kosu tamamlandi.\n', yaw_step_deg(cfg));
end

%% --- Metrikler ---
fprintf('\n=== YAW ADIM YANITI (saf MATLAB, aerodinamik yaw sonumlemesi YOK) ===\n');
fprintf('%-8s %8s %10s %8s %9s %10s %10s %8s %8s\n', 'adim', 'asim%', 'ts2deg(s)', 'e_ss', ...
        'max|r|', 'RMSr_son', 'salinim?', 'vh_adim', 'vh_son');
fprintf('%s\n', repmat('-', 1, 92));

for cfg = 1:numel(yaw_step_deg)
    log = results.(sprintf('cfg%d', cfg));
    psi_sp = deg2rad(yaw_step_deg(cfg));
    idx = log.t >= t_step;
    tt  = log.t(idx);
    psi = log.att(3, idx).';
    r   = log.omega(3, idx).';

    err = psi - psi_sp;

    % Asim: adimin YONUNDE hedefi asan en buyuk sapma
    if psi_sp > 0
        os = max(0, max(psi) - psi_sp);
    else
        os = max(0, psi_sp - min(psi));
    end
    os_pct = 100 * os / abs(psi_sp);

    % Yerlesme suresi: |e| <= 2 deg olup KOSUNUN SONUNA KADAR oyle kalan ilk an
    tol = deg2rad(2);
    out = find(abs(err) > tol, 1, 'last');
    if isempty(out)
        ts = 0;
    elseif out >= numel(tt)
        ts = NaN;                       % kosu bitene kadar hic oturmadi
    else
        ts = tt(out+1) - t_step;
    end

    % Son 5 saniyedeki yaw hizi RMS'i = kalici salinim gostergesi
    tail = log.t >= (Tsim - 5);
    rms_r_tail = sqrt(mean(log.omega(3, tail).^2));

    vh_step = log.vh(find(log.t >= t_step, 1));
    vh_end  = log.vh(end);

    % ONEMLI: "ts2deg = -" yalnizca +-2 deg bandina hic girilmedigi anlamina
    % gelir; bu KALICI SALINIM demek DEGILDIR. Ikisini ayirt eden sey RMSr_son:
    % kalici bir salinim varsa yaw HIZI da salinir. SITL'in (Q) maddesindeki
    % sonumsuz salinimda r ~ +-0.5 rad/s olculmustu — burada esik olarak
    % 0.01 rad/s kullaniliyor (iki mertebe altinda).
    osc = rms_r_tail > 0.01;
    if isnan(ts), ts_str = '         -'; else, ts_str = sprintf('%10.2f', ts); end
    fprintf('%+8d %8.1f %s %8.2f %9.3f %10.4f %10s %8.2f %8.2f\n', ...
        yaw_step_deg(cfg), os_pct, ts_str, rad2deg(err(end)), ...
        max(abs(r)), rms_r_tail, string(osc), vh_step, vh_end);
end

fprintf('\nasim/e_ss derece, r rad/s, vh = yatay hiz (m/s).\n');
fprintf('ts2deg = |yaw hatasi| <= 2 deg bandina girip kosu sonuna kadar kalinan an\n');
fprintf('         ("-" = banda hic girilmedi; salinim mi kalici ofset mi oldugunu\n');
fprintf('          RMSr_son / salinim? sutunlari soyler).\n');
fprintf('salinim? = son 5 s yaw hizi RMS > 0.01 rad/s  (SITL (Q): ~+-0.5 rad/s).\n');

%% --- Grafikler ---
fig = figure('Position',[100 100 1150 900]);
colors = {[0.85 0.2 0.2],[0.2 0.4 0.8]};
names  = arrayfun(@(d) sprintf('yaw\\_sp = %+d deg', d), yaw_step_deg, 'UniformOutput', false);

subplot(4,1,1); hold on; grid on;
for cfg = 1:numel(yaw_step_deg)
    log = results.(sprintf('cfg%d', cfg));
    plot(log.t, rad2deg(log.att(3,:)), 'Color', colors{cfg}, 'LineWidth', 1.4, ...
         'DisplayName', names{cfg});
    yline(yaw_step_deg(cfg), '--', 'Color', colors{cfg}, 'HandleVisibility','off');
end
xline(t_step,'k--','adim','HandleVisibility','off');
ylabel('\psi (deg)'); legend('Location','best'); title('Yaw acisi — adim yaniti');

subplot(4,1,2); hold on; grid on;
for cfg = 1:numel(yaw_step_deg)
    log = results.(sprintf('cfg%d', cfg));
    plot(log.t, log.omega(3,:),   'Color', colors{cfg}, 'LineWidth', 1.4, ...
         'DisplayName', [names{cfg} ' — r']);
    plot(log.t, log.omegasp(3,:), 'Color', colors{cfg}, 'LineWidth', 0.9, ...
         'LineStyle',':', 'DisplayName', [names{cfg} ' — r_{sp}']);
end
yline( p.ctrl.rate_sp_limit(3), 'k:', 'HandleVisibility','off');
yline(-p.ctrl.rate_sp_limit(3), 'k:', 'HandleVisibility','off');
ylabel('r (rad/s)'); legend('Location','best');
title(sprintf('Yaw hizi ve setpoint (RATE\\_SP\\_LIMIT_{yaw} = %.1f rad/s)', p.ctrl.rate_sp_limit(3)));

subplot(4,1,3); hold on; grid on;
for cfg = 1:numel(yaw_step_deg)
    log = results.(sprintf('cfg%d', cfg));
    plot(log.t, rad2deg(log.delta(1,:)), 'Color', colors{cfg}, 'LineWidth', 1.3, ...
         'DisplayName', [names{cfg} ' — \delta_0']);
    plot(log.t, rad2deg(log.delta(2,:)), 'Color', colors{cfg}, 'LineWidth', 1.3, ...
         'LineStyle','--', 'DisplayName', [names{cfg} ' — \delta_1']);
end
yline(0,'k:','HandleVisibility','off');   % tek yonlu tilt araliginin alt siniri (madde P)
ylabel('tilt (deg)'); legend('Location','best');
title('Kanat tilt acilari — alt sinir 0 deg (tek yonlu aralik, madde (P))');

subplot(4,1,4); hold on; grid on;
for cfg = 1:numel(yaw_step_deg)
    log = results.(sprintf('cfg%d', cfg));
    plot(log.t, log.tau_dem, 'Color', colors{cfg}, 'LineWidth', 1.2, ...
         'DisplayName', [names{cfg} ' — talep']);
    plot(log.t, log.tau_ach, 'Color', colors{cfg}, 'LineWidth', 1.2, ...
         'LineStyle','--', 'DisplayName', [names{cfg} ' — uretilen']);
end
ylabel('\Delta\tau_z (Nm)'); xlabel('t (s)'); legend('Location','best');
title('Yaw tork artisi: WLS talebi vs tahsisatin urettigi');

png_name = ['yaw_step_test' suffix '.png'];
saveas(fig, fullfile(fileparts(mfilename('fullpath')), png_name));
fprintf('\nGrafik kaydedildi: %s\n', png_name);
