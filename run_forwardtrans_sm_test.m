function run_forwardtrans_sm_test()
%RUN_FORWARDTRANS_SM_TEST  forwardtrans_loop.m'in SAF MANTIK testi (madde V).
%
% Adim 42 (2026-08-03). run_backtrans_sm_test.m ile ayni disiplin ve ayni
% gerekce: MATLAB plant'i bu MANEVRAYI uretemez (tek boylamsal yuzey, 12 m/s'de
% ~25 N tasima -- 49 N agirligi kaldiramaz), ama sorulan sey manevra degil
% MANTIK: "cikis kosulu saglanmazsa makine ne yapar", "emniyetler tetikleniyor
% mu", "iptal dogru sinyali veriyor mu". Mantik icin plant gereksiz, ve sentetik
% iz gercek aerodinamikten DAHA IYIDIR cunku arizali rejimi istedigin gibi
% kurabilirsin.
%
% Adim 21d'nin kurali korunuyor: ortam, hedeflenen mekanizma orada AKTIF oldugu
% icin gecerli. Sayilarin (fx_cruise = 12 N, cruise_v = 8 m/s) dogrulanmasi
% SITL'de yapilir.

p = tiltrotor_params();

fprintf('=== forwardtrans_loop durum makinesi testi (madde V) ===\n');
fprintf('fx_cruise = %.1f N, fx_rate = %.2f N/s, cruise_v = %.1f m/s\n', ...
        p.ft.fx_cruise, p.ft.fx_rate, p.ft.cruise_v);
fprintf('alt_band = %.1f m, timeout = %.1f s\n\n', p.ft.alt_band, p.ft.timeout_s);

ok = true;

% --- 1) NORMAL: hiz rampayla birlikte artiyor, CRUISE'a varilmali ----------
% Olculen profile gore: 0 -> 10 N / 12 s rampasi 10.86 m/s veriyor, yani hiz
% kabaca fx ile birlikte buyuyor. Burada hizi zamanin fonksiyonu olarak
% veriyoruz (0.75 m/s^2, ~11 s'de 8 m/s).
r1 = simulate(p, @(t) 0.75*t, @(t) 0.0, 60.0);
ok = check('1) CRUISE''a varildi', r1.state_final == 2, ok, ...
           sprintf('son durum = %d, t_cruise = %.1f s', r1.state_final, r1.t_cruise));
ok = check('1) iptal YOK', ~r1.aborted, ok, '');
ok = check('1) fx tavana oturdu', abs(r1.fx_final - p.ft.fx_cruise) < 1e-6, ok, ...
           sprintf('fx = %.2f N', r1.fx_final));
% Rampa suresi fx_rate'ten turemeli, elle yazilmis bir sayidan degil.
fprintf('    -> fx tavanina %.1f s''de ulasti (beklenen %.1f s = fx_cruise/fx_rate)\n', ...
        r1.t_fx_full, p.ft.fx_cruise / p.ft.fx_rate);

% --- 2) EMNIYET 1: KACIS TIRMANISI (adim 29'un arizasi) --------------------
% Adim 29: pos_hold 14.5 m/s'de devreye girince arac 35 s boyunca ~1.1 m/s
% tirmandi (z -9.9 -> -54.0). Burada ayni imza kuruluyor: hiz normal artiyor
% ama irtifa kaciyor. Makine bunu GORMELI ve iptal etmeli.
% IKI MOD DA SINANIR. Varsayilan p.ft.allow_abort = false (2026-08-04
% gereksinimi: ileri gecis iptal olmaz) -- o modda dedektor UYARI verir ama
% manevrayi kesmez. allow_abort = true eski davranisi birebir geri getirmeli;
% ikisi de test edilir ki gereksinim degisirse hangi yolun bozuldugu belli olsun.
p_ab = p; p_ab.ft.allow_abort = true;

r2 = simulate(p_ab, @(t) 0.75*t, @(t) -1.1*t, 60.0);
ok = check('2a) [allow_abort=1] irtifa kacisinda IPTAL', r2.aborted, ok, ...
           sprintf('iptal t = %.1f s, |dz| = %.1f m', r2.t_abort, abs(1.1*r2.t_abort)));
ok = check('2a) iptal tam bandi asinca', ...
           abs(abs(1.1*r2.t_abort) - p.ft.alt_band) <= 1.1*2*p.Ts_pos + 1e-6, ok, ...
           sprintf('band = %.1f m', p.ft.alt_band));

r2b = simulate(p, @(t) 0.75*t, @(t) -1.1*t, 60.0);
ok = check('2b) [allow_abort=0] IPTAL YOK', ~r2b.aborted, ok, '');
ok = check('2b) ama dedektor UYARI veriyor (warn=1)', r2b.warn_max == 1, ok, ...
           sprintf('warn_code = %d, ilk t = %.1f s', r2b.warn_max, r2b.t_warn));
ok = check('2b) uyari tam bandi asinca (iptalle AYNI an)', ...
           abs(r2b.t_warn - r2.t_abort) <= 2*p.Ts_pos + 1e-9, ok, ...
           sprintf('uyari t = %.2f s vs iptal t = %.2f s', r2b.t_warn, r2.t_abort));

% --- 3) EMNIYET 2: HIZLANAMIYOR (aero-bagimsiz) ----------------------------
% Gercek kanat beklendigi gibi tasimazsa/suruklerse arac seyir hizina hic
% cikamaz. fx tavana varir ve orada kalir -- HIZ terimi bir daha asla
% saglanmaz. Sure terimi olmasaydi makine sonsuza kadar RAMP'te kalirdi.
r3 = simulate(p_ab, @(t) min(4.0, 0.4*t), @(t) 0.0, 120.0);
ok = check('3a) [allow_abort=1] hizlanamayinca IPTAL', r3.aborted, ok, ...
           sprintf('iptal t = %.1f s (timeout %.1f)', r3.t_abort, p.ft.timeout_s));
ok = check('3a) iptal tam timeout''ta', abs(r3.t_abort - p.ft.timeout_s) <= 2*p.Ts_pos, ok, ...
           sprintf('t_abort = %.2f s', r3.t_abort));

r3b = simulate(p, @(t) min(4.0, 0.4*t), @(t) 0.0, 120.0);
ok = check('3b) [allow_abort=0] IPTAL YOK, uyari var (warn=2)', ...
           ~r3b.aborted && r3b.warn_max == 2, ok, ...
           sprintf('warn_code = %d, ilk t = %.1f s', r3b.warn_max, r3b.t_warn));
ok = check('3b) ve makine RAMP''te kaliyor (kendini CRUISE sanmiyor)', ...
           r3b.state_final == 1, ok, sprintf('son durum = %d', r3b.state_final));

% --- 4) EMNIYETSIZ MANTIK AYNI IZDE SONSUZA KADAR TAKILIR ------------------
% Regresyonun gercekten var oldugunu gostermek icin (test 3 "zaten calisiyordu"
% ile karismasin) -- run_backtrans_sm_test.m'in 4. testiyle ayni disiplin.
r4 = simulate_no_timeout(p, @(t) min(4.0, 0.4*t), 120.0);
ok = check('4) suresiz mantik ayni izde 120 s RAMP''te kaliyor', r4.state_final == 1, ok, ...
           sprintf('suresiz son durum = %d', r4.state_final));

% --- 5) ITKI DOYGUNKEN fx BUYUMEZ ------------------------------------------
% Doygunluk bir iptal sebebi degil bekleme sebebidir: tahsisat zaten kutuda,
% fx'i buyutmek cozulemeyen bir talep yaratir.
st = [1; 5.0; 0.0; 1.0];
[fx_a, ~, ~, ~, st_a] = forwardtrans_loop(true, 5.0, 0.0, true,  st, p);
[fx_b, ~, ~, ~, ~   ] = forwardtrans_loop(true, 5.0, 0.0, false, st, p);
ok = check('5) doygunken fx sabit kaliyor', abs(fx_a - 5.0) < 1e-9, ok, ...
           sprintf('fx = %.4f N', fx_a));
ok = check('5) doygun degilken fx buyuyor', fx_b > 5.0, ok, ...
           sprintf('fx = %.4f N', fx_b));
ok = check('5) doygunluk durumu DEGISTIRMIYOR', st_a(1) == 1, ok, ...
           sprintf('durum = %d', st_a(1)));

% --- 6) pitch HER ZAMAN 0 (adim 29) ----------------------------------------
% Bu bir "olmasi gereken" degil, olculmus bir gerekce: ~5-6 m/s ustunde burun
% yukari bir TIRMANMA komutudur ve irtifa dongusu buna karsi koyamaz.
pmax = 0.0;
st = [0; 0.0; 0.0; 0.0];
for k = 1:2000
    [~, pit, ~, ~, st] = forwardtrans_loop(true, 0.75*(k-1)*p.Ts_pos, 0.0, false, st, p);
    pmax = max(pmax, abs(pit));
end
ok = check('6) pitch komutu her zaman 0', pmax == 0.0, ok, sprintf('max |pitch| = %.3g', pmax));

% --- 7) enable dusurulunce her sey sifirlaniyor -----------------------------
st = [2; 12.0; -30.0; 40.0];
[fx, ~, rel, ab, st] = forwardtrans_loop(false, 12.0, -30.0, false, st, p);
ok = check('7) enable=0 fx''i sifirliyor', fx == 0.0, ok, sprintf('fx = %.2f', fx));
ok = check('7) enable=0 durumu IDLE yapiyor', st(1) == 0, ok, sprintf('durum = %d', st(1)));
ok = check('7) enable=0 hold''u birakmiyor', ~rel, ok, '');
ok = check('7) enable=0 iptal istemiyor', ~ab, ok, '');

fprintf('\n%s\n', repmat('-', 1, 62));
if ok
    fprintf('SONUC: GECTI -- ileri gecis mantigi dogru\n');
else
    fprintf('SONUC: KALDI\n');
end
fprintf(['NOT: bu test yalnizca YAPIYI dogrular. fx_cruise = %.1f N ve\n' ...
         'cruise_v = %.1f m/s SAYILARININ dogrulanmasi SITL''de yapilir\n' ...
         '(bu manevranin aerosu MATLAB plant''inda olusmuyor).\n'], ...
        p.ft.fx_cruise, p.ft.cruise_v);

end


function r = simulate(p, v_of_t, dz_of_t, t_end)
%SIMULATE  Makineyi sentetik hiz ve irtifa-sapmasi izleriyle kosar.
st = [0; 0.0; 0.0; 0.0];
r = struct('state_final', 0, 'aborted', false, 't_abort', NaN, ...
           't_cruise', NaN, 't_fx_full', NaN, 'fx_final', NaN, ...
           'warn_max', 0, 't_warn', NaN);
prev = 0;
n = round(t_end / p.Ts_pos);
z0 = -30.0;
for k = 1:n
    t = (k - 1) * p.Ts_pos;
    [fx, ~, ~, ab, st, wc] = forwardtrans_loop(true, v_of_t(t), z0 + dz_of_t(t), false, st, p);
    if isnan(r.t_fx_full) && fx >= p.ft.fx_cruise - 1e-6
        r.t_fx_full = t;
    end
    if st(1) == 2 && prev == 1
        r.t_cruise = t;
    end
    if wc ~= 0 && r.warn_max == 0
        r.warn_max = wc;
        r.t_warn   = t;
    end
    if ab && ~r.aborted
        r.aborted = true;
        r.t_abort = t;
        break;      % cagiran taraf iptalde geri gecise gecer, makine burada durur
    end
    prev = st(1);
    r.fx_final = fx;
end
r.state_final = st(1);
end


function r = simulate_no_timeout(p, v_of_t, t_end)
%SIMULATE_NO_TIMEOUT  Sure emniyeti OLMAYAN mantigin birebir kopyasi.
% Yalnizca regresyonun varligini gostermek icin; kontrol yolunda kullanilmaz.
state = 0; fx = 0.0;
r = struct('state_final', 0);
n = round(t_end / p.Ts_pos);
for k = 1:n
    t = (k - 1) * p.Ts_pos;
    v = v_of_t(t);
    switch state
        case 0
            state = 1; fx = 0.0;
        case 1
            fx = min(p.ft.fx_cruise, fx + p.ft.fx_rate * p.Ts_pos);
            if fx >= p.ft.fx_cruise - 1e-6 && v >= p.ft.cruise_v
                state = 2;
            end
    end
end
r.state_final = state;
end


function ok = check(name, cond, ok_in, detail)
if cond
    fprintf('  [GECTI] %s', name);
else
    fprintf('  [KALDI] %s', name);
end
if ~isempty(detail)
    fprintf('  (%s)', detail);
end
fprintf('\n');
ok = ok_in && cond;
end
