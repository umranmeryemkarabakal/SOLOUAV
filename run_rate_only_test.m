%% RUN_RATE_ONLY_TEST
% RATE_ONLY failsafe seviyesi hayatta kalinabilir mi? (engelleyici B1, Adim 34)
%
% ================== SONUC (Adim 35, 2026-07-30): HAYIR ==================
% Uc senaryonun UCU DE ters dondu ve yere carpti. En temizi (bozucu yok, duz
% girildi) 179.6 deg roll'e gidip 11.66 m/s ile carpti; serbest dusus referansi
% 26.2 m/s. Yani "motorlari kesmekten kesinlikle daha iyi" savi nicel olarak
% sinirda, nitel olarak YANLIS -- kurtarilmis bir ucus degil, biraz
% yavaslatilmis bir dusus.
%
% Bunun uzerine `FsLevel::RATE_ONLY` PX4 modulunden KALDIRILDI: durus kestirimi
% artik kademelendirilebilir bir girdi degil, sert on kosul. Bu betik silinmedi
% -- o kararin KANITI, ve tekrar edilebilir olmasi gerekiyor.
%
% Ayrik bulgu, korundu: dikey politika CALISTI. FS_FZ_OPENLOOP = 0.97, durus
% duz tutuldugunda 49.5 s boyunca ort. 0.71 m/s alcalma verdi. Basarisiz olan
% omega_sp = 0.
% ========================================================================
%
% NEDEN BU TEST VAR. PX4 tarafinda kademeli failsafe'in en derin seviyesi
% (FsLevel::RATE_ONLY) su savi tasiyor: attitude kestirimi kaybolsa bile ic INDI
% hiz dongusu yalnizca omega ve omega_dot istedigi icin arac hiz sonumlemesiyle
% ucabilir, ve bu "motorlari kesmek"ten KESINLIKLE daha iyidir. SITL'de bu sav
% hic olculemedi: `ekf2 stop` sonrasi commander 0.05 s icinde nav_state'i
% NAVIGATION_STATE_TERMINATION'a aliyor, `actuator_armed.lockdown` true oluyor,
% modul ona NaN'la cevap veriyor ve arac 35 m'den serbest dusuyor (gz yer
% gercegi 34.84 -> 0.10 m, ~2.7 s). Yani seviye hic bir 4 ms tick'ten uzun
% calismadi.
%
% MATLAB'da commander YOK. Bu yuzden sav burada saf haliyle olculebilir --
% ve bu bir KONTROL YASASI savi oldugu icin dogru yer de burasi.
%
% PX4 dallarinin BIREBIR karsiligi (kontrolcude hicbir degisiklik gerekmedi):
%   * omega_sp = 0  <->  att_sp = att verilir. indi_attitude_controller.m:35-49
%     e_att = att_sp - att hesaplayip omega_sp = Kp_att .* e_att yaptigi icin
%     att_sp = att tam olarak omega_sp = 0 demektir.
%   * irtifa dongusu kapali, Fz acik cevrim = -m*g*FS_FZ_OPENLOOP (0.97)
%     -- PX4'te lpos'un tamami gecersizken kosan dal.
%   * Fx_sp = 0 ve pozisyon dongusu yok (bozulmada fx_sp sifirlanir).
%
% Uc senaryo, cunku RATE_ONLY'nin savi "hangi yatista ise onu korur" -- yani
% baslangic kosulu sonucu belirler ve tek senaryodan genelleme yapmak bu
% projede defalarca yanlis cikti (Adim 20'nin dersi):
%   1) temiz hover'dan bozulma (en iyi durum)
%   2) bozucu/gust altinda bozulma (gercekci durum: kestirim kaybi sakin havada
%      olmaz)
%   3) YATIK haldeyken bozulma (10 deg roll komutu verilmis, henuz duzelmemis)
%
% Karsilastirma olcutu, alternatifin kendisi: ayni irtifadan SERBEST DUSUS.

clear; clc;
addpath(fileparts(mfilename('fullpath')));

p = tiltrotor_params();

FS_FZ_OPENLOOP = 0.97;   % TiltrotorIndiParams.hpp ile ayni
ALT0    = 35.0;          % m, SITL seviye-3 kosusuyla ayni irtifa
T_FAIL  = 4.0;           % s, bozulma ani (once trim'e yerlessin)
TSIM    = 60.0;          % s, ya da yere temas

dt_ctrl = p.Ts_ctrl;
n_sub   = 5;
dt_phys = dt_ctrl/n_sub;
N = round(TSIM/dt_ctrl);

scen = struct( ...
    'name',      {'1) temiz hover', '2) gust + roll bozucusu', '3) 10 deg yatik'}, ...
    'gust',      {false, true, false}, ...
    'bank_deg',  {0, 0, 10});

fprintf('=== RATE_ONLY hayatta kalinabilirlik testi (irtifa %.0f m, bozulma t=%.0f s) ===\n\n', ...
        ALT0, T_FAIL);

% Karsilastirma: ayni irtifadan serbest dusus
t_ff = sqrt(2*ALT0/p.g);
v_ff = sqrt(2*p.g*ALT0);
fprintf('Referans -- SERBEST DUSUS %.0f m: %.2f s sonra %.1f m/s ile carpma\n\n', ALT0, t_ff, v_ff);

results = struct();

for s_i = 1:numel(scen)
    u_trim = hover_trim(p);
    x = zeros(19,1);
    x(3)     = -ALT0;
    x(7:10)  = quat_from_euler(0,0,0);
    x(14:19) = u_trim;

    ctrl_state = init_ctrl_state();
    omega_dot_filt = zeros(3,1);
    leso_axes = [true; true; false];      % dogrulanmis hover konfigurasyonu
    alt_state = 0;
    alt_accum = 0;
    Fz_sp = -p.m*p.g;
    z_sp  = x(3);

    log = struct('t', zeros(N,1), 'att', zeros(3,N), 'omega', zeros(3,N), ...
                 'pos', zeros(3,N), 'vel_ned', zeros(3,N), ...
                 'T', zeros(3,N), 'delta', zeros(3,N), 'degraded', false(N,1));
    n_end = N;

    for k = 1:N
        t = (k-1)*dt_ctrl;
        degraded = t >= T_FAIL;

        % --- Bozucu programi (senaryo 2) ---
        if scen(s_i).gust && t >= T_FAIL
            ramp = min(1, (t-T_FAIL)/1.0);
            wind_ned = ramp * [-5; 0; -0.8];
            ext_m = [ramp*(0.4 + 0.15*sin(2*pi*0.3*(t-T_FAIL))); 0; 0];
        else
            wind_ned = [0;0;0];
            ext_m = [0;0;0];
        end

        att      = quat_to_euler(x(7:10));
        omega    = x(11:13);
        u_actual = x(14:19);

        xdot_now = tiltrotor_plant_deriv(x, u_actual(1:3), u_actual(4:6), p, wind_ned, ext_m);
        omega_dot_raw = xdot_now(11:13);
        omega_dot_filt = omega_dot_filt + 0.3*(omega_dot_raw - omega_dot_filt);

        if ~degraded
            % --- SAGLAM: normal attitude + irtifa dongusu ---
            % Senaryo 3'te bozulmadan hemen once bir yatis komutu verilir, boylece
            % arac RATE_ONLY'ye YATIK girer -- "hangi yatista ise onu korur"
            % savini olcmenin tek yolu bu.
            if t >= T_FAIL - 2.0
                att_sp = [deg2rad(scen(s_i).bank_deg); 0; 0];
            else
                att_sp = [0;0;0];
            end

            alt_accum = alt_accum + dt_ctrl;
            if alt_accum >= p.Ts_pos - 1e-12
                [Fz_sp, alt_state] = altitude_loop(z_sp, x(3), xdot_now(3), alt_state, p);
                alt_accum = alt_accum - p.Ts_pos;
            end
        else
            % --- BOZULMUS (RATE_ONLY): dis attitude dongusu YOK, Fz acik cevrim ---
            att_sp = att;                         % <=> omega_sp = 0
            Fz_sp  = -p.m*p.g*FS_FZ_OPENLOOP;
        end

        F_sp = [0; Fz_sp];    % Fx_sp = 0 (bozulmada da, oncesinde de)

        [T_cmd, delta_cmd, ctrl_state] = indi_attitude_controller( ...
            att_sp, att, omega, omega_dot_filt, F_sp, u_actual, ctrl_state, p, leso_axes);

        for sub = 1:n_sub
            x = rk4_step(@(xx) tiltrotor_plant_deriv(xx, T_cmd, delta_cmd, p, wind_ned, ext_m), x, dt_phys);
        end

        log.t(k)          = t;
        log.att(:,k)      = att;
        log.omega(:,k)    = omega;
        log.pos(:,k)      = x(1:3);
        log.vel_ned(:,k)  = xdot_now(1:3);
        log.T(:,k)        = T_cmd;
        log.delta(:,k)    = delta_cmd;
        log.degraded(k)   = degraded;

        if x(3) >= 0        % yere temas (NED, z>=0)
            n_end = k;
            break;
        end
    end

    % Kayitlari kirp
    f = fieldnames(log);
    for i = 1:numel(f)
        if size(log.(f{i}), 2) == N
            log.(f{i}) = log.(f{i})(:, 1:n_end);
        else
            log.(f{i}) = log.(f{i})(1:n_end);
        end
    end

    results.(sprintf('s%d', s_i)) = log;

    % --- Metrikler (yalnizca bozulma penceresi) ---
    d = log.degraded;
    att_d   = log.att(:, d);
    om_d    = log.omega(:, d);
    vel_d   = log.vel_ned(:, d);
    T_d     = log.T(:, d);
    del_d   = log.delta(:, d);
    t_d     = log.t(d);

    v_h  = hypot(vel_d(1,:), vel_d(2,:));
    vz   = vel_d(3,:);                      % NED, + = asagi
    z_end = log.pos(3, end);
    hit   = z_end >= 0;

    yaw_turn = rad2deg(trapz(t_d, om_d(3,:)));
    pin_lo = 100*mean(T_d(:) <= p.rotor.Tmin + 1e-6);
    pin_hi = 100*mean(T_d(:) >= p.rotor.Tmax - 1e-6);

    % Kontrolun yitirilmesine kadar gecen sure: pratikte belirleyici sayi, cunku
    % bir pilotun (ya da toparlanma mantiginin) elinde ne kadar zaman oldugunu
    % soyler. Yatis esikleri: 30 deg = artik duruslu ucus degil, 90 deg = itki
    % vektoru yatay/asagi, yani acik cevrim Fz artik yer cekimine karsi koymuyor.
    tilt_ang = max(abs(att_d(1,:)), abs(att_d(2,:)));
    t30 = first_cross(t_d, tilt_ang, deg2rad(30)) - t_d(1);
    t90 = first_cross(t_d, tilt_ang, deg2rad(90)) - t_d(1);
    z30 = -log.pos(3, find(d, 1) + max(0, find(tilt_ang > deg2rad(30), 1) - 1));

    fprintf('--- %s ---\n', scen(s_i).name);
    fprintf('  bozulma suresi        : %.2f s  (%s)\n', t_d(end)-t_d(1), ...
            ternary(hit, sprintf('YERE TEMAS t=%.2f s', log.t(end)), 'hala havada'));
    fprintf('  KONTROL KAYBI         : |yatis|>30 deg @ %.1f s (irtifa %.1f m), >90 deg @ %.1f s\n', ...
            t30, z30, t90);
    fprintf('  max |roll| / |pitch|  : %.1f / %.1f deg   (son: %.1f / %.1f)\n', ...
            rad2deg(max(abs(att_d(1,:)))), rad2deg(max(abs(att_d(2,:)))), ...
            rad2deg(att_d(1,end)), rad2deg(att_d(2,end)));
    fprintf('  yaw toplam donus      : %+.1f deg\n', yaw_turn);
    fprintf('  |omega| max           : %.3f rad/s\n', max(abs(om_d(:))));
    fprintf('  alcalma vz ort / max  : %.2f / %.2f m/s\n', mean(vz), max(vz));
    fprintf('  yatay v_h son / max   : %.2f / %.2f m/s   (yatay yol %.1f m)\n', ...
            v_h(end), max(v_h), hypot(log.pos(1,end)-log.pos(1,1), log.pos(2,end)-log.pos(2,1)));
    fprintf('  itki araligi          : %.2f - %.2f N   (pin lo %.1f%% / hi %.1f%%)\n', ...
            min(T_d(:)), max(T_d(:)), pin_lo, pin_hi);
    fprintf('  tilt araligi          : %.2f - %.2f deg\n', ...
            rad2deg(min(del_d(:))), rad2deg(max(del_d(:))));
    if hit
        fprintf('  ** CARPMA HIZI        : dusey %.2f m/s, yatay %.2f m/s  (serbest dusus %.1f m/s)\n', ...
                vz(end), v_h(end), v_ff);
    end
    fprintf('\n');
end

%% --- Grafikler ---
fig = figure('Position',[100 100 1150 900]);
cols = {[0.85 0.2 0.2], [0.2 0.4 0.8], [0.15 0.6 0.25]};
rows = {'irtifa (m, yukari +)', 'roll / pitch (deg)', 'alcalma hizi vz (m/s)', 'itki (N)'};
for r = 1:4
    subplot(4,1,r); hold on; grid on;
    for s_i = 1:numel(scen)
        L = results.(sprintf('s%d', s_i));
        switch r
            case 1, y = -L.pos(3,:);
            case 2, y = rad2deg(L.att(1,:));
            case 3, y = L.vel_ned(3,:);
            case 4, y = L.T(1,:);
        end
        plot(L.t, y, 'Color', cols{s_i}, 'LineWidth', 1.3, 'DisplayName', scen(s_i).name);
        if r == 2
            plot(L.t, rad2deg(L.att(2,:)), '--', 'Color', cols{s_i}, 'HandleVisibility','off');
        end
    end
    xline(T_FAIL, 'k--', 'RATE\_ONLY', 'HandleVisibility','off');
    ylabel(rows{r});
    if r == 1, legend('Location','best'); title('RATE\_ONLY failsafe: hayatta kalinabilirlik (duz=roll, kesikli=pitch)'); end
    if r == 4, xlabel('t (s)'); end
end
saveas(fig, fullfile(fileparts(mfilename('fullpath')), 'rate_only_test.png'));
fprintf('Grafik kaydedildi: rate_only_test.png\n');

function out = ternary(c, a, b)
if c, out = a; else, out = b; end
end

function t = first_cross(tv, y, thr)
i = find(y > thr, 1);
if isempty(i), t = NaN; else, t = tv(i); end
end
