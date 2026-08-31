function run_cruise_wingborne_test()
%RUN_CRUISE_WINGBORNE_TEST  Madde (V): seyir gercekten "sabit kanat" mi?
%
% Adim 46 (2026-08-03). Adim 43 SITL'de olctu: 14.4 m/s seyirde kanat tilti
% 32 deg, kuyruk tilti 0.6 deg, ve AGIRLIGIN %59'u hala rotorlarda. Zarf
% supruldugunde (fx 12 -> 24 N) her iki KANAT rotoru tamamen kapaniyor ama
% KUYRUK rotoru tek basina 21.8 N tasimaya devam ediyor.
%
% HIPOTEZ (bu testin sinadigi sav): kuyruk rotorunun dik kalip yuk tasimasinin
% sebebi tasima ihtiyaci degil, PITCH TRIM OTORITESIDIR. Kontrol yuzeyleri
% oynatilmadigi icin seyirde pitch'i tutabilecek tek sey kuyruk rotorunun
% itkisidir (0.74 m kol), ve iki yone de otorite gerektigi icin sifira
% inemez. 0.70 m kollu elevator tahsisata girerse (15 m/s'de 111 N*m/rad)
% kuyruk rotoru serbest kalmali, tilt daha ileri yatmali, kanat yuku artmali.
%
% YONTEM: ayni senaryo IKI KEZ kosulur -- yuzeyler KAPALI (Adim 42 davranisi,
% referans) ve ACIK. Aradaki fark yalnizca yuzeylerdir; plant, kazanclar,
% rampalar, rusgar ve baslangic kosullari birebir aynidir.
%
% FX = 10 N SECIMI, VE NEDEN p.ft.fx_cruise (12 N) DEGIL -- OLCULDU:
% fx zarfi tarandi (8/10/12/14 N, yuzeyler acik ve kapali). fx >= 12 N'de
% YUZEYLER KAPALIYKEN BILE rejim marjinal: tahsisat doygunlugu %13.5-16.1 ve
% kanat rotorlarinin tilt'i 89.5 derecede, yani 90 derecelik MEKANIK DURAKTA
% cakili. Orada fazladan fx talebi karsilanamaz; aracin hizini sinirlayan sey
% bir kontrol yasasi degil, o duraktir. Yuzeyler acilinca bu kazara koruma
% ortadan kalkiyor, arac hizlanmaya devam ediyor ve trimlenemeyecegi bir
% rejime giriyor (t = 29-37 s'de iraksama).
%
% BU BIR YUZEY KUSURU DEGIL, EKSIK BIR KATMANDIR: seyirde hiz/enerji yonetimi
% (TECS benzeri) YOK ve fx acik dongu bir hizlandiricidir. Bu, README'de zaten
% "otonomi icin kalan eksik #1" olarak yazili. Yuzeyleri o eksigin uzerinde
% sinamak, yasayi degil test kurgusunu olcerdi.
%
% 10 N ise yuzeysiz halde %9.0 doygunlukla rahat kararli bir rejimdir ve
% 70 s boyunca oyle kalir; yuzeylerin katkisi orada TEMIZ olculebilir.

p = tiltrotor_params();
FX_GATE = 10.0;   % N -- gerekce yukarida

fprintf('=== seyir / kanat-tasimali ucus testi (madde V, Adim 46) ===\n');
fprintf('agirlik = %.2f N,  fx hedefi = %.1f N,  rampa = %.0f s, Tsim = 70 s\n\n', ...
        p.m*p.g, FX_GATE, 12);

R_off = fly(p, false, FX_GATE);
R_on  = fly(p, true,  FX_GATE);

fprintf('\n%s\n', repmat('=', 1, 74));
fprintf('%-34s %14s %14s\n', 'olcut', 'yuzey KAPALI', 'yuzey ACIK');
fprintf('%s\n', repmat('-', 1, 74));
row('ortalama tilt (deg)',        R_off.tilt_bar,  R_on.tilt_bar);
row('  kanat tilt d0 (deg)',      R_off.tilt_w,    R_on.tilt_w);
row('  kuyruk tilt d2 (deg)',     R_off.tilt_t,    R_on.tilt_t);
row('ileri hiz u (m/s)',          R_off.u,         R_on.u);
row('KANAT YUK PAYI (%)',         R_off.wing_pct,  R_on.wing_pct);
row('kanat rotor itkisi T0 (N)',  R_off.T0,        R_on.T0);
row('KUYRUK rotor itkisi T2 (N)', R_off.T2,        R_on.T2);
row('toplam rotor itkisi (N)',    R_off.Tsum,      R_on.Tsum);
row('irtifa sapmasi (m)',         R_off.dz,        R_on.dz);
row('max |omega| (rad/s)',        R_off.wmax,      R_on.wmax);
row('doygun aktuator (%)',        R_off.sat_pct,   R_on.sat_pct);
fprintf('%s\n', repmat('-', 1, 74));
fprintf('%-34s %14s %14.3f\n', 'aileron sapmasi (deg)',  '-', R_on.ail);
fprintf('%-34s %14s %14.3f\n', 'elevator sapmasi (deg)', '-', R_on.ele);
fprintf('%-34s %14s %14.3f\n', 'rudder sapmasi (deg)',   '-', R_on.rud);
fprintf('%s\n\n', repmat('=', 1, 74));

% --- HUKUM ------------------------------------------------------------------
ok = true;
ok = check('0) IRAKSAMA YOK (|omega| hic 1 rad/s''yi asmadi)', ...
           isnan(R_on.tdiv), ok, ...
           sprintf('t_div = %s', tern(isnan(R_on.tdiv),'-',sprintf('%.1f s',R_on.tdiv))));
% NOT: kuyruk rotorunun rahatlamasi OLCULDU ama ZAYIF (14.83 -> 13.62 N, %8).
% Hipotezin bu bileseni yalnizca KISMEN destekleniyor: ail+ele ikilisinde daha
% belirgin (11.45 N), rudder eklenince geri yukseliyor. Esik bu yuzden mutevazi
% tutuldu -- olculenden fazlasini iddia etmemek icin.
ok = check('1) kuyruk rotoru rahatladi (T2 dustu) [ZAYIF etki]', ...
           R_on.T2 < R_off.T2 - 0.5, ok, ...
           sprintf('%.2f -> %.2f N', R_off.T2, R_on.T2));
ok = check('2) kanat yuk payi ARTTI', ...
           R_on.wing_pct > R_off.wing_pct + 2, ok, ...
           sprintf('%.1f -> %.1f %%', R_off.wing_pct, R_on.wing_pct));
ok = check('3) tilt DAHA ILERI yatti', ...
           R_on.tilt_bar > R_off.tilt_bar + 1, ok, ...
           sprintf('%.1f -> %.1f deg', R_off.tilt_bar, R_on.tilt_bar));
ok = check('4) kararlilik bozulmadi (max|omega| < 2x)', ...
           R_on.wmax < max(2*R_off.wmax, 0.05), ok, ...
           sprintf('%.4f -> %.4f rad/s', R_off.wmax, R_on.wmax));
ok = check('5) irtifa kacmadi (|dz| < 5 m)', abs(R_on.dz) < 5, ok, ...
           sprintf('%.2f m', R_on.dz));
ok = check('6) yuzeyler sapma limitine YAPISMADI', ...
           abs(R_on.ail) < rad2deg(p.surf.max(1))*0.9 && ...
           abs(R_on.ele) < rad2deg(p.surf.max(3))*0.9 && ...
           abs(R_on.rud) < rad2deg(p.surf.max(5))*0.9, ok, ...
           sprintf('limitler %.0f/%.0f/%.0f deg', rad2deg(p.surf.max(1)), ...
                   rad2deg(p.surf.max(3)), rad2deg(p.surf.max(5))));

fprintf('\n');
if ok
    fprintf('SONUC: HIPOTEZ DESTEKLENDI\n');
else
    fprintf('SONUC: ***HIPOTEZ DESTEKLENMEDI*** -- yukaridaki [HATA] satirlari\n');
end

end

% ---------------------------------------------------------------------------
function R = fly(p, use_surf, fx_gate)

n_surf = p.surf.n;
u_trim = hover_trim(p);

if use_surf
    nx = 19 + n_surf;
else
    nx = 19;
end
x = zeros(nx,1);
x(3)      = -80;
x(7:10)   = quat_from_euler(0,0,0);
x(14:19)  = u_trim;

att_sp   = [0;0;0];
z_sp     = x(3);
fx_final = fx_gate;
t_ramp   = 12;
Tsim     = 70;

dt_ctrl = p.Ts_ctrl;
n_sub   = 5;
dt_phys = dt_ctrl/n_sub;
N       = round(Tsim/dt_ctrl);
leso_axes = [true;true;false];

ctrl_state     = init_ctrl_state();
omega_dot_filt = zeros(3,1);
alt_state = 0; alt_accum = 0;
Fz_sp = -p.m*p.g;
a_virt = zeros(3,1);          % sanal yuzey aktuator durumu (OLCULEN)
surf_cmd = zeros(n_surf,1);
Mv = surf_virtual_map(p);

% Iraksama tanisi: ilk kez |omega| > 1 rad/s olan an ve o andaki rejim.
t_div = NaN; div_snap = [];

wind_ned = [0;0;0];
ext_m    = [0;0;0];

wmax = 0; sat_n = 0; sat_d = 0;

% kanat yuku olcumu icin yalniz kanat panelleri
pw = p; pw.aero.area(3:5) = 0;

for k = 1:N
    t = (k-1)*dt_ctrl;
    Fx_sp = fx_final * min(1, t/t_ramp);

    att      = quat_to_euler(x(7:10));
    omega    = x(11:13);
    u_rotor  = x(14:19);

    if use_surf
        xdot_now = tiltrotor_plant_deriv(x, u_rotor(1:3), u_rotor(4:6), p, wind_ned, ext_m, surf_cmd);
    else
        xdot_now = tiltrotor_plant_deriv(x, u_rotor(1:3), u_rotor(4:6), p, wind_ned, ext_m);
    end
    omega_dot_raw  = xdot_now(11:13);
    omega_dot_filt = omega_dot_filt + 0.3*(omega_dot_raw - omega_dot_filt);

    alt_accum = alt_accum + dt_ctrl;
    if alt_accum >= p.Ts_pos - 1e-12
        [Fz_sp, alt_state] = altitude_loop(z_sp, x(3), xdot_now(3), alt_state, p);
        alt_accum = alt_accum - p.Ts_pos;
    end
    F_sp = [Fx_sp; Fz_sp];

    if use_surf
        % qbar: gercek hava-goreli hizdan. PX4'te bu airspeed/yer hizidir.
        Va   = norm(x(4:6));
        qbar = 0.5 * p.rho * Va^2;
        % INDI'nin lineerlestirme noktasi KOMUT degil OLCUM olmali (rotorlarda
        % zaten x(14:19) kullaniliyor). Fiziksel yuzey durumundan sanal
        % koordinata geri projeksiyon: Mv sutunlari dik oldugu icin bu
        % (Mv'*Mv)\Mv' ile tam olarak (s0-s1)/2, (s2+s3)/2, s4 demektir.
        % Trim ofseti CIKARILMALI: sanal koordinat, ofsetin USTUNDEKI sapmadir.
        % Cikarilmazsa kontrolcu kendi trimini bir hata sanip kovalar ve
        % elevator sapma limitine sarilir (Adim 52'de bir kez olculdu:
        % t = 8.8 s'de iraksama, elevator -29.8 deg = limit).
        a_virt = (Mv'*Mv) \ (Mv' * (x(20:19+n_surf) - surf_trim_offset(p, qbar)));
        u_act = [u_rotor; a_virt];
        [T_cmd, delta_cmd, ctrl_state, diagn, surf_cmd] = indi_attitude_controller( ...
            att_sp, att, omega, omega_dot_filt, F_sp, u_act, ctrl_state, p, leso_axes, [], qbar);
    else
        [T_cmd, delta_cmd, ctrl_state, diagn] = indi_attitude_controller( ...
            att_sp, att, omega, omega_dot_filt, F_sp, u_rotor, ctrl_state, p, leso_axes);
    end

    sat_n = sat_n + sum(diagn.sat_flag);
    sat_d = sat_d + numel(diagn.sat_flag);

    for s = 1:n_sub
        if use_surf
            x = rk4_step(@(xx) tiltrotor_plant_deriv(xx, T_cmd, delta_cmd, p, ...
                               wind_ned, ext_m, surf_cmd), x, dt_phys);
        else
            x = rk4_step(@(xx) tiltrotor_plant_deriv(xx, T_cmd, delta_cmd, p, ...
                               wind_ned, ext_m), x, dt_phys);
        end
    end

    wmax = max(wmax, norm(omega));
    if isnan(t_div) && norm(omega) > 1.0
        t_div = t;
        div_snap = struct('u', x(4), 'Va', norm(x(4:6)), 'qbar', 0.5*p.rho*norm(x(4:6))^2, ...
                          'tilt', rad2deg(mean(x(17:19))), 'theta', rad2deg(att(2)), ...
                          'omega', omega, 'sat', sum(diagn.sat_flag));
        if use_surf
            div_snap.surf = rad2deg(x(20:19+n_surf))';
        end
    end
end

if ~isnan(t_div)
    fprintf('  !! IRAKSAMA: t = %.2f s''de |omega| 1 rad/s''yi asti\n', t_div);
    fprintf('     o anda: u = %.2f m/s, qbar = %.1f Pa, tilt = %.1f deg, theta = %.1f deg\n', ...
            div_snap.u, div_snap.qbar, div_snap.tilt, div_snap.theta);
    fprintf('     omega = [%.2f %.2f %.2f], doygun aktuator = %d\n', ...
            div_snap.omega(1), div_snap.omega(2), div_snap.omega(3), div_snap.sat);
    if isfield(div_snap, 'surf')
        fprintf('     yuzeyler (deg) = [%.2f %.2f %.2f %.2f %.2f]\n', div_snap.surf);
    end
end

% --- son durum olculeri (son 2 s ortalamasi degil, son ornek: denge) --------
R.tilt_bar = rad2deg(mean(x(17:19)));
R.tilt_w   = rad2deg(x(17));
R.tilt_t   = rad2deg(x(19));
R.u        = x(4);
R.T0       = x(14);
R.T2       = x(16);
R.Tsum     = sum(x(14:16));
R.dz       = -x(3) - 80;
R.wmax     = wmax;
R.sat_pct  = 100*sat_n/sat_d;

if use_surf
    sv = x(20:19+n_surf);
else
    sv = zeros(n_surf,1);
end
[Fw, ~] = aero_panels(x(4:6), x(11:13), sv, pw);
R.wing_pct = -Fw(3) / (p.m*p.g) * 100;

R.ail  = rad2deg(sv(1));
R.ele  = rad2deg(sv(3));
R.rud  = rad2deg(sv(5));
R.tdiv = t_div;

end

function s = tern(c,a,b)
if c, s = a; else, s = b; end
end

% ---------------------------------------------------------------------------
function row(name, a, b)
fprintf('%-34s %14.3f %14.3f\n', name, a, b);
end

function ok = check(name, cond, ok, detail)
if cond
    fprintf('  [ok]   %s   %s\n', name, detail);
else
    fprintf('  [HATA] %s   %s\n', name, detail);
    ok = false;
end
end
