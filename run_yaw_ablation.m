%% RUN_YAW_ABLATION
% Yaw sonumleme boslugu icin ABLASYON calismasi (Adim 20, 2026-07-28).
%
% SORU: MATLAB ayni +30 deg yaw adimini 3.1-3.7 s'de oturtuyor, SITL ~8 s
% (adim) / ~33 s (arm gecicisi) aliyor -- ayni ~5 s periyot, ama SITL'in
% sonumleme orani kabaca 10x daha dusuk. Bu fark NEREDEN geliyor?
%
% Elenmis olanlar (bkz. sitl/WLS_LOCKUP_INVESTIGATION_REPORT.md):
%   - aerodinamik ruzgar gulu / ileri hiz  (Adim 19, tek degiskenli A/B)
%   - golge aktuator modelinin TILT sapmasi (Adim 18, p99 <= 0.55 deg)
%   - trim on-yukleme                       (Adim 19a)
%
% Bu script, SITL'in MATLAB'da BULUNMAYAN kusurlarini tek tek enjekte edip
% hangisinin yaw sonumlemesini bozdugunu olcer. Her ablasyon bagimsiz ve
% birlestirilebilir; hicbiri kalici bir kontrol sabiti degisikligi DEGILDIR.
%
% ABLASYONLAR
%   'shadow'  Kontrolcu, plant'in GERCEK aktuator durumu yerine PX4'teki gibi
%             ACIK CEVRIM bir golge model gorur (MulticopterIndiTiltrotor.cpp
%             :414-421'in birebir kopyasi). MATLAB'da normalde
%             u_actual = x(14:19), yani gercek durum -- bu, SITL'in sahip
%             OLMADIGI bir avantaj. INDI'nin lineerlestirme noktasi ve WLS'in
%             G matrisi bundan turedigi icin en guclu aday.
%   'odot_delay'  omega_dot_meas'e ek tasima gecikmesi (ms). SITL'de olculdu:
%             xyz_derivative ~4-8 ms (MATLAB'in alpha=0.3 filtresi ~7 ms).
%   'odot_noise'  omega_dot_meas'e beyaz gurultu (rad/s^2 RMS). SITL'de yaw
%             ekseninde HF gurultu RMS ~0.019 rad/s^2 olculdu.
%   'dt_jitter'   kontrol adiminda +-jitter (PX4 gercek-zamanli zamanlama).
%   'box_dt_mismatch'  ** ADIM 21'DE BULUNAN GERCEK PX4 HATASI **
%             Kontrol dongusu 250 Hz'de kosar (PX4 modulu
%             vehicle_angular_velocity callback'iyle surulur; olculen medyan
%             periyot 4.00 ms) AMA WLS'in slew kutusu sabit TS_CTRL = 1/400
%             ile boyutlanir (MulticopterIndiTiltrotor.cpp:315-316, 324-325 --
%             ayni fonksiyon dt'yi 171. satirda dogru hesaplayip golge model
%             ve LESO icin kullaniyor, yalnizca kutuda kullanmiyor).
%             Etki: tilt artisi tick basina TILT_RATE_MAX*(1/400) = 0.005 rad
%             ile sinirli, ama tick 4 ms suruyor -> efektif slew tavani
%             1.25 rad/s, hedeflenen 2.0'in %62'si. Kanat tilt'i yaw'in TEK
%             gercek aktuatoru oldugundan (cikarim 13) bu dogrudan yaw
%             otoritesini kisar. SITL'de olculdu: tilt sat_flag %99.4-99.9,
%             |ddelta| p99 = tam 0.00500 rad, tahsisat yaw verimi %20.6.
%             MATLAB bunu YAPISAL OLARAK goremez: dongu gercekten p.Ts_ctrl
%             periyodunda kosar, yani kutu ile dongu her zaman tutarlidir.
%
% KULLANIM
%   run_yaw_ablation            % tum matris
%
% Cikti: konsol tablosu + yaw_ablation.png

clear; clc;
addpath(fileparts(mfilename('fullpath')));
p = tiltrotor_params();

t_step = 4.0; Tsim = 30.0;
psi_sp = deg2rad(30);
leso_axes = [true; true; false];
n_sub = 5;

% --- ablasyon matrisi ---
% sutunlar: ad | shadow | odot gecikme (ms) | odot gurultu | dt jitter |
%           gercek dongu frekansi (Hz; [] = p.Ts_ctrl ile ayni, yani kutu tutarli)
cfgs = {
    'baz (MATLAB gibi)',              false, 0.0,  0.000, 0.0,  []
    'golge aktuator modeli',          true,  0.0,  0.000, 0.0,  []
    'omega_dot +8 ms gecikme',        false, 8.0,  0.000, 0.0,  []
    'omega_dot gurultu 0.02',         false, 0.0,  0.020, 0.0,  []
    'dt jitter %25',                  false, 0.0,  0.000, 0.25, []
    'KUTU/DONGU UYUMSUZ (250Hz)',     false, 0.0,  0.000, 0.0,  250
    'HEPSI + kutu uyumsuzlugu',       true,  8.0,  0.020, 0.25, 250
};

fprintf('=== YAW SONUMLEME ABLASYONU (+30 deg adim, %g s) ===\n', Tsim);
fprintf('SITL referansi: adim ~8 s, arm gecicisi ~33 s. MATLAB bazi: ~3.5 s.\n\n');
fprintf('%-26s %8s %10s %9s %9s %8s\n', 'ablasyon', 'asim%', 'ts2deg(s)', 'RMSr_son', 'max|r|', 'cevrim');
fprintf('%s\n', repmat('-', 1, 76));

logs = cell(size(cfgs,1),1);

for c = 1:size(cfgs,1)
    use_shadow = cfgs{c,2};
    odot_delay = cfgs{c,3}/1000;
    odot_noise = cfgs{c,4};
    jitter     = cfgs{c,5};
    loop_hz    = cfgs{c,6};

    % PX4 hatasinin birebir emulasyonu: dongu GERCEKTE dt_loop periyodunda
    % koser, ama indi_attitude_controller icindeki slew kutusu p.Ts_ctrl
    % (=1/400) ile boyutlanmaya devam eder -- yani kutu, tick suresinden kisa
    % bir periyoda gore hesaplanir. p.Ts_ctrl'e DOKUNULMAZ; degisen yalnizca
    % gercek adim suresi.
    if isempty(loop_hz)
        dt_loop = p.Ts_ctrl;
    else
        dt_loop = 1/loop_hz;
    end

    rng(12345);   % tekrarlanabilir gurultu/jitter

    u_trim = hover_trim(p);
    x = zeros(19,1); x(3) = -50;
    x(7:10) = quat_from_euler(0,0,0);
    x(14:19) = u_trim;

    cs = init_ctrl_state();
    odf = zeros(3,1);
    alt_state = 0; alt_accum = 0; Fz_sp = -p.m*p.g;
    u_shadow = u_trim;                       % PX4'teki gibi hover_trim ile tohumlanir
    dbuf_n = max(1, round(odot_delay/dt_loop) + 1);
    dbuf = zeros(3, dbuf_n);                 % omega_dot gecikme tamponu

    N = round(Tsim/dt_loop);
    tl = zeros(N,1); yl = zeros(N,1); rl = zeros(N,1);
    t = 0;

    for k = 1:N
        dt = dt_loop;
        if jitter > 0
            dt = dt_loop * (1 + jitter*(2*rand-1));
        end

        att_sp = [0;0;0];
        if t >= t_step, att_sp(3) = psi_sp; end

        att = quat_to_euler(x(7:10)); om = x(11:13);
        u_true = x(14:19);

        xd = tiltrotor_plant_deriv(x, u_true(1:3), u_true(4:6), p, [0;0;0], [0;0;0]);
        odf = odf + 0.3*(xd(11:13) - odf);

        % --- omega_dot bozulmalari ---
        dbuf = [odf, dbuf(:,1:end-1)];
        od_meas = dbuf(:,end);
        if odot_noise > 0
            od_meas = od_meas + odot_noise*randn(3,1);
        end

        alt_accum = alt_accum + dt;
        if alt_accum >= p.Ts_pos - 1e-12
            [Fz_sp, alt_state] = altitude_loop(-50, x(3), xd(3), alt_state, p);
            alt_accum = alt_accum - p.Ts_pos;
        end

        % --- kontrolcunun GORDUGU aktuator durumu ---
        if use_shadow
            u_seen = u_shadow;
        else
            u_seen = u_true;
        end

        [Tc, dc, cs, ~] = indi_attitude_controller(att_sp, att, om, od_meas, ...
            [0; Fz_sp], u_seen, cs, p, leso_axes);
        u_cmd = [Tc; dc];

        % --- golge modeli ilerlet (MulticopterIndiTiltrotor.cpp:414-421 kopyasi) ---
        for i = 1:3
            if u_cmd(i) >= u_shadow(i), tau_i = p.rotor.tau_up; else, tau_i = p.rotor.tau_down; end
            u_shadow(i) = u_shadow(i) + dt*(u_cmd(i)-u_shadow(i))/tau_i;
        end
        for i = 1:3
            dd = (u_cmd(3+i) - u_shadow(3+i)) / p.tilt.tau;
            dd = max(min(dd, p.tilt.rate_max), -p.tilt.rate_max);
            u_shadow(3+i) = u_shadow(3+i) + dt*dd;
        end

        for s = 1:n_sub
            x = rk4_step(@(xx) tiltrotor_plant_deriv(xx, Tc, dc, p, [0;0;0], [0;0;0]), x, dt/n_sub);
        end

        tl(k) = t; yl(k) = att(3); rl(k) = om(3);
        t = t + dt;
    end

    % --- metrikler ---
    idx = tl >= t_step;
    tt = tl(idx); psi = yl(idx); rr = rl(idx);
    os = max(0, max(psi) - psi_sp);
    os_pct = 100*os/psi_sp;
    tol = deg2rad(2);
    out = find(abs(psi - psi_sp) > tol, 1, 'last');
    if isempty(out), ts = 0; elseif out >= numel(tt), ts = NaN; else, ts = tt(out+1) - t_step; end
    tail = tl >= (tl(end) - 5);
    rms_tail = sqrt(mean(rl(tail).^2));
    % Salinim cevrim sayisi. DIKKAT: ham r'nin sifir gecislerini saymak YANLIS
    % sonuc verir -- tilt/WLS kaynakli tick-mertebesinde bir cirpinma var ve
    % 250+ sahte gecis uretiyor. Once yumusat, sonra yalnizca ANLAMLI genlikli
    % (>%10 tepe) yarim cevrimleri say.
    rs = movmean(rr, max(3, round(0.10/dt_loop)));  % 100 ms yumusatma
    thr = 0.10 * max(abs(rs));
    sig = sign(rs); sig(abs(rs) < thr) = 0;
    sig = sig(sig ~= 0);
    zc = sum(diff(sig) ~= 0);

    if isnan(ts), ts_s = '  OTURMADI'; else, ts_s = sprintf('%10.2f', ts); end
    fprintf('%-26s %8.1f %s %9.4f %9.3f %8.0f\n', cfgs{c,1}, os_pct, ts_s, ...
            rms_tail, max(abs(rr)), zc/2);

    logs{c} = struct('t', tl, 'yaw', yl, 'r', rl, 'name', cfgs{c,1});
end

fprintf(['\nts2deg = |yaw hatasi| <= 2 deg bandina girip sonuna kadar kalinan an.\n' ...
         'cevrim = adim sonrasi yaw hizi sifir gecisi / 2 (salinim cevrim sayisi).\n' ...
         'YORUM: SITL''in ~8 s / ~33 s''ini uretebilen ablasyon, sonumleme\n' ...
         'boslugunun kaynagidir. Hicbiri uretemiyorsa aday liste eksik demektir.\n']);

%% --- Grafik ---
fig = figure('Position',[100 100 1150 780]);
subplot(2,1,1); hold on; grid on;
for c = 1:numel(logs)
    plot(logs{c}.t, rad2deg(logs{c}.yaw), 'LineWidth', 1.3, 'DisplayName', logs{c}.name);
end
yline(30,'k--','HandleVisibility','off'); xline(t_step,'k:','HandleVisibility','off');
ylabel('yaw (deg)'); legend('Location','southeast','FontSize',8);
title('Yaw ablasyonu — +30 deg adim (SITL: adim ~8 s, arm gecicisi ~33 s; MATLAB bazi ~3.5 s)');

subplot(2,1,2); hold on; grid on;
for c = 1:numel(logs)
    plot(logs{c}.t, logs{c}.r, 'LineWidth', 1.1, 'DisplayName', logs{c}.name);
end
xline(t_step,'k:','HandleVisibility','off');
ylabel('yaw hizi r (rad/s)'); xlabel('t (s)'); legend('Location','northeast','FontSize',8);
title('yaw hizi');

saveas(fig, fullfile(fileparts(mfilename('fullpath')), 'yaw_ablation.png'));
fprintf('\nGrafik kaydedildi: yaw_ablation.png\n');
