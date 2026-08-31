function run_backtrans_sm_test()
%RUN_BACKTRANS_SM_TEST  backtrans_loop.m durum makinesinin SAF MANTIK testi.
%
% Adim 38 (2026-07-31), madde (R) icin yazildi.
% Adim 39 (2026-08-03), madde (S) icin genisletildi: test 7-9.
%
% NEDEN PLANT YOK
% ---------------
% backtrans_loop.m'in kendi basligi dogru sekilde uyariyor: MATLAB plant'i bu
% MANEVRAYI yeniden uretemez (tek boylamsal yuzey, 12 m/s'de ~25 N tasima), o
% yuzden "geri gecis calisiyor mu" sorusu yalnizca SITL'de cevaplanir. Ama
% madde (R)'nin sorusu bu DEGIL. Soru su: "cikis kosulu saglanmazsa durum
% makinesi ne yapar?" -- bu tamamen bir MANTIK sorusu, ve mantik icin plant
% gereksiz. v_h'yi sentetik olarak vermek, gercek aerodinamigin ureteceginden
% DAHA IYIDIR, cunku arizali rejimi (terminal hiz esigin ustunde) istedigin
% gibi kurabilirsin -- SITL'de o rejim rastlantisaldi: 8 ucusun 3'unde cikti.
%
% Adim 21d'nin kurali burada da gecerli: bir ortam ancak hedeflenen mekanizma
% orada AKTIFSE bir sey kanitlar. Hedeflenen mekanizma "cikis kosulu ile denge
% arasindaki iliski" oldugu icin AKTIF ortam durum makinesinin kendisidir.
% Aero esikleri (release_v'nin 10.0 SAYISI dogru mu) burada DEGIL, SITL'de
% dogrulanir -- bu test yalnizca yasanin yapisini denetler.

p = tiltrotor_params();

fprintf('=== backtrans_loop durum makinesi testi (madde R + S) ===\n');
fprintf('release_v = %.1f m/s, floor_dwell = %.1f s, ceil_floor = %.1f deg\n', ...
        p.bt.release_v, p.bt.floor_dwell, rad2deg(p.bt.ceil_floor));
fprintf('handoff_v = %.1f m/s (ISARETLI govde ileri hizina uygulanir)\n\n', ...
        p.bt.handoff_v);

ok = true;

% --- 1) NORMAL: hiz esigin altina iner, cikis HIZ terimiyle olmali ----------
% Giris 15 m/s, tavan 41 deg'den 2 deg/s ile iniyor (16 s), hiz 0.35 m/s^2 ile
% azaliyor -- yani tavan tabana vardiginda arac zaten ~9.4 m/s'de, esigin
% (10.0) altinda. Ucurulan iyi ucuslarin profili bu.
r1 = simulate(p, @(t) max(1.0, 15.0 - 0.35*t), deg2rad(41), 300.0);
ok = check('1) normal dizi tamamlaniyor', r1.state_final == 3, ok, ...
           sprintf('son durum = %d', r1.state_final));
ok = check('1) cikis HIZ terimiyle (emniyet degil)', strcmp(r1.exit_cause, 'speed'), ok, ...
           sprintf('cikis sebebi = %s', r1.exit_cause));
ok = check('1) BRAKE tam tavan tabana varinca basliyor', ...
           abs(r1.t_brake - r1.t_floor) <= 2*p.Ts_pos, ok, ...
           sprintf('t_floor = %.1f s, t_brake = %.1f s', r1.t_floor, r1.t_brake));

% --- 2) MADDE (R)'NIN OLCULEN ARIZASI: terminal hiz 8.97 m/s ---------------
% 2026-07-30, log 11_20_38: RETRACT 200.3 s boyunca cikmadi cunku arac
% 8.97 m/s'nin altina hic inmedi ve eski esik 8.0 idi. Iz olculen profile
% gore kuruldu: 14.5 m/s giris, 8.97'ye oturma, ve tavanin tabana vardigi
% anda (16.0 s) ~9.5 m/s -- adim 37'nin 11_16_48 ucusunda olculen deger.
% Yeni esikle (10.0) HIZ terimi bunu tek basina cozmeli.
r2 = simulate(p, @(t) 8.97 + 5.53*exp(-t/9.0), deg2rad(41), 300.0);
ok = check('2) olculen takilma RETRACT''tan cikiyor', r2.state_final >= 2, ok, ...
           sprintf('son durum = %d, t_brake = %.1f s', r2.state_final, r2.t_brake));
ok = check('2) ve cikis HIZ terimiyle oluyor', strcmp(r2.exit_cause, 'speed'), ok, ...
           sprintf('cikis sebebi = %s', r2.exit_cause));
% MARJ RAPORU -- bu projenin kendi kurali: "tamamlandi" kaydi ne kadar PAYLA
% tamamlandigini soylemez (adim 37, genel ders 1). Yeni esigin payi kucuk.
fprintf('    -> BRAKE giris hizi %.2f m/s, esige (%.1f) pay %.2f m/s\n', ...
        r2.v_at_brake, p.bt.release_v, p.bt.release_v - r2.v_at_brake);
fprintf('       (iste bu yuzden esik TEK BASINA yeterli sayilmiyor -- test 3)\n');

% --- 3) EMNIYET: terminal hiz YENI esigin de ustunde (donanim senaryosu) ---
% 10.0 sayisi Gazebo aerodinamiginden geldi. Gercek kanat daha az suruklerse
% terminal hiz esigin de ustune cikar ve HIZ terimi bir daha asla saglanmaz.
% Emniyetin butun varlik sebebi bu: cikis yine de olmali.
r3 = simulate(p, @(t) 11.5 + 5.0*exp(-t/25.0), deg2rad(41), 300.0);
ok = check('3) terminal hiz esigin USTUNDE iken de cikiliyor', r3.state_final >= 2, ok, ...
           sprintf('son durum = %d', r3.state_final));
ok = check('3) cikis EMNIYET terimiyle', strcmp(r3.exit_cause, 'dwell'), ok, ...
           sprintf('cikis sebebi = %s', r3.exit_cause));
dwell_err = abs((r3.t_brake - r3.t_floor) - p.bt.floor_dwell);
ok = check('3) emniyet tam floor_dwell sonra tetikleniyor', dwell_err <= 2*p.Ts_pos, ok, ...
           sprintf('tabanda gecen sure = %.2f s (hedef %.2f)', ...
                   r3.t_brake - r3.t_floor, p.bt.floor_dwell));
% Asil kazanc: yaw'i ac birakan konfigurasyonda gecirilen sure SINIRLI.
% Olculen suruklenme ~0.59 deg/s (200 s'de +117.7 deg).
fprintf('    -> tabanda maruz kalinan yaw suruklenmesi ~ %.1f deg (olcut 45)\n', ...
        0.59 * (r3.t_brake - r3.t_floor));

% --- 4) ESKI KOD BU IZDE SONSUZA KADAR TAKILIRDI ---------------------------
% Ayni izi emniyetsiz mantikla kosarak regresyonun gercekten var oldugunu
% gosteriyoruz (yoksa test 3 "zaten calisiyordu" ile karisabilir).
r3_old = simulate_no_backstop(p, @(t) 11.5 + 5.0*exp(-t/25.0), deg2rad(41), 300.0);
ok = check('4) emniyetsiz mantik ayni izde 300 s takiliyor', r3_old.state_final == 1, ok, ...
           sprintf('emniyetsiz son durum = %d', r3_old.state_final));

% --- 5) enable dusurulunce tavan BIRAKILIYOR, sayac sifirlaniyor -----------
st = [1; deg2rad(9.0); 15.0];
[tc, ~, ~, st] = backtrans_loop(false, 5.0, 5.0, deg2rad(9.0), st, p);
ok = check('5) enable=0 tavani birakiyor', abs(tc - p.tilt.max) < 1e-9, ok, ...
           sprintf('tilt_ceil = %.1f deg', rad2deg(tc)));
ok = check('5) enable=0 sayaci sifirliyor', st(3) == 0.0, ok, ...
           sprintf('floor_dwell = %.2f s', st(3)));

% --- 6) Sayac tabanda DEGILKEN islemiyor (giris hizina bagli olmamali) -----
% Tavan 60 deg'den iniyor: tabana varmasi 25.5 s suruyor, yani floor_dwell'den
% (20 s) UZUN. Sayac inis sirasinda isleseydi manevra daha tavan inmeden
% BRAKE'e gecerdi -- Fz kaynakli kacisi geri davet eden tam olarak bu.
r6 = simulate(p, @(t) 20.0 - 0.0*t, deg2rad(60), 60.0);
ok = check('6) tavan inerken emniyet tetiklenmiyor', r6.state_final == 1 || r6.t_brake >= r6.t_floor, ok, ...
           sprintf('t_floor = %.1f s, son durum = %d', r6.t_floor, r6.state_final));

% --- 7) MADDE (S)'NIN OLCULEN ARIZASI: yanal surukleme buyuklugu tutuyor ---
% 2026-07-31, log 14_13_11 (kancali ucus): BRAKE penceresinde min|v_h| = 3.08
% m/s iken govde ileri hizi u = -0.51, yanal hiz v = +3.04 m/s. Yani ileri
% eksen bitmisti; esigi tutan sey fren yasasinin KONTROL ETMEDIGI eksendi.
% Iz bunu birebir kuruyor: v_fwd 13.5'ten -0.51'e iniyor (yani ileri eksen
% duruyor ve isaret degistiriyor), yanal bilesen 3.04'te SABIT -- BRAKE boyunca
% onu azaltacak hicbir sey yok. v_h = hypot(ikisi), yani v_h hicbir zaman
% 3.04'un altina INEMIYOR ve esik 3.0. Asimptot tam olarak olculen degerlere
% oturur: v_fwd -> -0.51, v_h -> 3.08.
vf7 = @(t) 14.0*exp(-t/18.0) - 0.51;
vh7 = @(t) hypot(vf7(t), 3.04);
r7  = simulate(p, vh7, deg2rad(41), 300.0, vf7);
ok = check('7) yanal surukleme varken de HANDOFF''a geciliyor', r7.state_final == 3, ok, ...
           sprintf('son durum = %d, t_handoff = %.1f s', r7.state_final, r7.t_handoff));
ok = check('7) handoff ILERI hiz esigi gecince oluyor', ...
           ~isnan(r7.t_handoff) && abs(vf7(r7.t_handoff) - p.bt.handoff_v) <= 0.05, ok, ...
           sprintf('t_handoff''ta v_fwd = %.2f m/s (v_h = %.2f)', ...
                   vf7(r7.t_handoff), vh7(r7.t_handoff)));
fprintf('    -> handoff aninda |v_h| = %.2f m/s, yani BUYUKLUK esigin (%.1f) USTUNDE\n', ...
        vh7(r7.t_handoff), p.bt.handoff_v);
fprintf('       (eski mantik tam olarak bu yuzden hic gecemiyordu -- test 8)\n');

% --- 8) ESKI (buyukluk) MANTIK AYNI IZDE HIC GECMIYOR ----------------------
% Regresyonun gercekten var oldugunu gostermek icin, test 4 ile ayni disiplin.
r8 = simulate_v_h_handoff(p, vh7, deg2rad(41), 300.0);
ok = check('8) buyukluk esigi ayni izde 300 s BRAKE''te kaliyor', r8.state_final == 2, ok, ...
           sprintf('buyukluk mantigiyla son durum = %d', r8.state_final));

% --- 9) FREN MARJI geri giderken SIFIRLANIYOR (kacisi durduran terim) ------
% Durum degistirmek yetmez: itmeyi kesen sey marjin sifirlanmasidir. BRAKE
% durumunda birakip pitch ciktisini dogrudan olcuyoruz.
pitch_trim = asin(min(1.0, p.ctrl.fx_trim / (p.m * p.g)));
[~, pit_fwd, ~, ~] = backtrans_loop(true, 3.08, +3.00, deg2rad(9.0), [2; p.bt.brake_ceil; 0.0], p);
[~, pit_back, ~, ~] = backtrans_loop(true, 3.08, -0.51, deg2rad(9.0), [2; p.bt.brake_ceil; 0.0], p);
ok = check('9) ileri giderken tam marj var', ...
           abs(pit_fwd - (pitch_trim + p.bt.pitch_max)) < 1e-9, ok, ...
           sprintf('pitch = %.2f deg (trim %.2f + marj %.2f)', ...
                   rad2deg(pit_fwd), rad2deg(pitch_trim), rad2deg(p.bt.pitch_max)));
ok = check('9) geri giderken YALNIZCA durus trimi kaliyor', ...
           abs(pit_back - pitch_trim) < 1e-9, ok, ...
           sprintf('pitch = %.2f deg (trim %.2f)', rad2deg(pit_back), rad2deg(pitch_trim)));
fprintf('    -> olculen kacista eski yasa burada %.2f deg (= %.2f N) veriyordu;\n', ...
        rad2deg(pitch_trim + p.bt.pitch_max), ...
        p.m * p.g * sin(pitch_trim + p.bt.pitch_max));
fprintf('       yenisi %.2f deg (= %.2f N), yani net itme ~ %.2f N (fx_trim %.1f N''e karsi)\n', ...
        rad2deg(pit_back), p.m * p.g * sin(pit_back), ...
        p.m * p.g * sin(pit_back) - p.ctrl.fx_trim, p.ctrl.fx_trim);

fprintf('\n%s\n', repmat('-', 1, 62));
if ok
    fprintf('SONUC: GECTI -- durum makinesi mantigi dogru\n');
else
    fprintf('SONUC: KALDI\n');
end
fprintf(['NOT: bu test yalnizca YAPIYI dogrular. release_v = 10.0 ve\n' ...
         'floor_dwell = 20 s SAYILARININ dogrulanmasi SITL''de yapilir\n' ...
         '(bu manevra MATLAB plant''inda olusmuyor).\n']);

end


function r = simulate(p, v_of_t, delta0, t_end, vf_of_t)
%SIMULATE  Durum makinesini sentetik bir v_h (ve v_fwd) izi ile kosar.
% vf_of_t verilmezse arac tam ileri gidiyor kabul edilir (v_fwd = v_h) --
% adim 38 testlerinin izleri budur ve o testler bu yuzden degismeden gecer.
if nargin < 5 || isempty(vf_of_t)
    vf_of_t = v_of_t;
end
st = [0; p.tilt.max; 0.0];
r = struct('t_floor', NaN, 't_brake', NaN, 't_handoff', NaN, ...
           'state_final', 0, 'exit_cause', 'yok', 'v_at_brake', NaN);
prev = 0;
n = round(t_end / p.Ts_pos);
for k = 1:n
    t = (k - 1) * p.Ts_pos;
    v = v_of_t(t);
    [~, ~, ~, st] = backtrans_loop(true, v, vf_of_t(t), delta0, st, p);

    % t_floor durumdan BAGIMSIZ okunmali: hiz kosulu zaten saglanmissa tavanin
    % tabana vardigi tick ile BRAKE'e gecilen tick AYNIDIR, yani "durum hala
    % RETRACT iken" diye bakmak bu ani hic goremez (testin ilk surumunun
    % hatasi). Olcum penceresi, olctugu seyden bagimsiz bir sinyalden
    % kurulmali -- adim 34(e)/37(a)'nin ayni tuzagi.
    if isnan(r.t_floor) && st(2) <= p.bt.ceil_floor + 1e-6
        r.t_floor = t;
    end
    if st(1) == 2 && prev == 1
        r.t_brake = t;
        r.v_at_brake = v;
        % Cikisi hangi terim saglamis? Kararin verildigi andaki v_h ile
        % sayacin degeri ayirt eder.
        if v < p.bt.release_v
            r.exit_cause = 'speed';
        else
            r.exit_cause = 'dwell';
        end
    end
    if st(1) == 3 && prev == 2
        r.t_handoff = t;
    end
    prev = st(1);
end
r.state_final = st(1);
end


function r = simulate_no_backstop(p, v_of_t, delta0, t_end)
%SIMULATE_NO_BACKSTOP  Adim 38 ONCESI cikis kosulunun birebir kopyasi.
% Yalnizca regresyonun varligini gostermek icin; kontrol yolunda kullanilmaz.
state = 0; tilt_ceil = p.tilt.max;
r = struct('state_final', 0);
n = round(t_end / p.Ts_pos);
for k = 1:n
    t = (k - 1) * p.Ts_pos;
    v = v_of_t(t);
    switch state
        case 0
            state = 1; tilt_ceil = min(p.tilt.max, delta0);
        case 1
            tilt_ceil = max(p.bt.ceil_floor, tilt_ceil - p.bt.retract_rate * p.Ts_pos);
            if tilt_ceil <= p.bt.ceil_floor + 1e-6 && v < 8.0   % ESKI esik
                state = 2;
            end
        case 2
            if v < p.bt.handoff_v
                state = 3;
            end
    end
end
r.state_final = state;
end


function r = simulate_v_h_handoff(p, v_of_t, delta0, t_end)
%SIMULATE_V_H_HANDOFF  Adim 39 ONCESI handoff kosulunun birebir kopyasi.
% Tek fark: BRAKE -> HANDOFF `v_h < handoff_v` (BUYUKLUK) okuyor. Yalnizca
% madde (S) regresyonunun gercekten var oldugunu gostermek icin; kontrol
% yolunda kullanilmaz.
state = 0; tilt_ceil = p.tilt.max; floor_dwell = 0.0;
r = struct('state_final', 0);
n = round(t_end / p.Ts_pos);
for k = 1:n
    t = (k - 1) * p.Ts_pos;
    v = v_of_t(t);
    switch state
        case 0
            state = 1; tilt_ceil = min(p.tilt.max, delta0);
        case 1
            tilt_ceil = max(p.bt.ceil_floor, tilt_ceil - p.bt.retract_rate * p.Ts_pos);
            at_floor  = tilt_ceil <= p.bt.ceil_floor + 1e-6;
            if at_floor
                floor_dwell = floor_dwell + p.Ts_pos;
            else
                floor_dwell = 0.0;
            end
            if at_floor && (v < p.bt.release_v || floor_dwell >= p.bt.floor_dwell)
                state = 2;
            end
        case 2
            if v < p.bt.handoff_v   % ESKI sinyal: buyukluk
                state = 3;
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
