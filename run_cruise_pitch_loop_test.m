function run_cruise_pitch_loop_test()
%RUN_CRUISE_PITCH_LOOP_TEST  cruise_pitch_loop.m'in PLANT'SIZ mantik testi.
%
% Adim 49 (2026-08-04). Adim 38'in dersi: bir yasanin mantigi plant'ten BAGIMSIZ
% da sinanabilmeli, yoksa bir mantik hatasi ancak aerodinamik bir kilikta ortaya
% cikar. Beklenen degerler ELDE turetildi ve acikca yaziliyor.
%
% Denetlenenler:
%   A) KAPALI    -- pitch_enable = false iken cikis TAM sifir
%   B) KAPI      -- v_on altinda otorite TAM sifir (adim 29'un rejimi korunur)
%   C) ISARET    -- Fz_sp hedeften negatif -> burun YUKARI; pozitif -> ASAGI
%   D) DENGE     -- basit bir kanat modelinde Fz_sp HEDEFE yakinsiyor
%                   (hedef SIFIR DEGIL: sifir itki bir ucurumdur, bkz.
%                    cruise_pitch_loop.m ve tiltrotor_params.m pitch_fz_sp)
%                   D3 (Adim 53): ILERI BESLEME tek basina, integratorden
%                   BAGIMSIZ olarak dogru noktayi veriyor
%   E) HIZ       -- integralin zaman sabiti, MODEL HATASI altinda, turetilen
%                   1/(qbar*S*cla*Ki) ile ayni mertebede (irtifa dongusunden
%                   ~10x YAVAS olma sarti). Adim 53'ten once bu "hedefe varma
%                   suresi" idi; ileri beslemeyle o sure ~0 oldugu icin
%                   olculen sey degisti, turetme degismedi.
%   F) SINIR     -- |pitch_sp| TAM pitch_max'ta doyuyor, ve anti-windup
%                   TOPLAM (th_ff + I) uzerinde sismiyor

p  = tiltrotor_params();
ok = true;
fprintf('=== seyir pitch trim mantik testi (Adim 49) ===\n');
fprintf('kapi %.1f -> %.1f m/s, Ki = %.1e rad/(N*s), sinir = %.1f deg, Fz hedefi = %.1f N\n\n', ...
        p.tecs.pitch_v_on, p.tecs.pitch_v_full, p.tecs.pitch_ki, rad2deg(p.tecs.pitch_max), ...
        p.tecs.pitch_fz_sp);

% --- A) KAPALI --------------------------------------------------------------
poff = p; poff.tecs.pitch_enable = false;
[th, st] = cruise_pitch_loop(20.0, -30.0, 0.0, poff);
ok = check('A1) pitch_enable=false iken cikis TAM sifir', th == 0 && st == 0, ok, ...
           sprintf('%.3g', th));

% --- B) KAPI ----------------------------------------------------------------
% Adim 29'un rejimi (~5-6 m/s) korunmali: orada otorite SIFIR olmali, yoksa
% "burun yukari = tirmanma" tuzagina geri donulur.
st = 0;
for k = 1:2000
    [th, st] = cruise_pitch_loop(6.0, -40.0, st, p);
end
ok = check('B1) 6 m/s''de (adim 29 rejimi) otorite TAM sifir', th == 0, ok, ...
           sprintf('pitch = %.3g deg', rad2deg(th)));
ok = check('B2) ve integrator birikmiyor (bumpless giris)', st == 0, ok, ...
           sprintf('I = %.3g', st));
st = 0;
for k = 1:2000
    [th_mid, st] = cruise_pitch_loop(14.5, -40.0, st, p);
end
ok = check('B3) kapi ortasinda (14.5 m/s) KISMI otorite', ...
           th_mid > 0 && abs(th_mid) < p.tecs.pitch_max, ok, ...
           sprintf('pitch = %.3f deg', rad2deg(th_mid)));

% --- C) ISARET --------------------------------------------------------------
% Hata HEDEFE goredir: Fz_sp hedeften daha NEGATIF = kanat yetmiyor -> burun
% YUKARI. Daha POZITIF = kanat fazla tasiyor -> burun ASAGI.
Ft = p.tecs.pitch_fz_sp;
st = 0;
for k = 1:400, [th_up, st] = cruise_pitch_loop(20.0, Ft - 8.0, st, p); end
st = 0;
for k = 1:400, [th_dn, st] = cruise_pitch_loop(20.0, Ft + 8.0, st, p); end
% Adim 53: isaret olcutu de ILERI BESLEME NOKTASINA goredir. Yasa artik
% th_ff + I komut ediyor ve 20 m/s'de th_ff zaten -1.57 deg; "burun yukari"
% demek mutlak olarak pozitif olmak degil, th_ff'in USTUNE cikmak demektir.
pff = p; pff.tecs.pitch_ki = 0;
th_ff20 = cruise_pitch_loop(20.0, Ft - 8.0, 0, pff);
ok = check('C1) Fz_sp hedeften negatif (kanat yetmiyor) -> burun YUKARI', ...
           th_up > th_ff20, ok, ...
           sprintf('%.3f deg (th_ff = %.3f)', rad2deg(th_up), rad2deg(th_ff20)));
ok = check('C2) Fz_sp hedeften pozitif (kanat fazla) -> burun ASAGI', ...
           th_dn < th_ff20, ok, ...
           sprintf('%.3f deg (th_ff = %.3f)', rad2deg(th_dn), rad2deg(th_ff20)));
% Adim 53: simetri artik SIFIR etrafinda degil, ILERI BESLEME NOKTASI
% etrafindadir -- yasa th_ff + I komut ediyor ve simetrik olmasi gereken sey
% integratorun katkisidir. th_ff'i olcmek icin Ki = 0 ile tek bir cagri yeter.
ok = check('C3) ileri besleme noktasi etrafinda simetrik', ...
           abs((th_up - th_ff20) + (th_dn - th_ff20)) < 1e-12, ok, ...
           sprintf('%.3g (th_ff = %.3f deg)', (th_up-th_ff20)+(th_dn-th_ff20), rad2deg(th_ff20)));
[~, st_hold] = cruise_pitch_loop(20.0, Ft, 0.05, p);
ok = check('C4) Fz_sp = hedef iken integrator DURUYOR', ...
           abs(st_hold - 0.05) < 1e-12, ok, sprintf('I: 0.05 -> %.6f', st_hold));

% --- D/E) DENGE ve HIZ ------------------------------------------------------
% Oyuncak model: aktuator dikey yuku Fz_sp = -(W - L(theta)),
%   L(theta) = qbar*S*cla*(a0 + theta)
% Bu, irtifa dongusunun KALICI cozumudur (irtifa tutuluyorsa aktuatorler
% eksigi kapatir). Yasa Fz_sp'yi HEDEFE surmeli, yani L -> W + pitch_fz_sp.
V = 20.0; W = p.m*p.g;
qbar = 0.5*p.aero.rho*V^2;
S = 2*p.aero.area(1);            % iki kanat yarisi
dLdth = qbar * S * p.aero.cla;
Fz_of = @(th) -(W - qbar*S*p.aero.cla*(p.aero.a0(1) + th));

st = 0; th = 0; Fz0 = Fz_of(0);
Ts = p.Ts_pos;
for k = 1:round(300/Ts)
    Fz = Fz_of(th);
    [th, st] = cruise_pitch_loop(V, Fz, st, p);
end
Fz_end = Fz_of(th);
ok = check('D1) Fz_sp HEDEFE yakinsiyor (|hata| < 0.5 N)', ...
           abs(Fz_end - p.tecs.pitch_fz_sp) < 0.5, ok, ...
           sprintf('%.2f -> %.4f N (hedef %.1f)', Fz0, Fz_end, p.tecs.pitch_fz_sp));
th_hand = (W + p.tecs.pitch_fz_sp)/(qbar*S*p.aero.cla) - p.aero.a0(1);
ok = check('D2) denge pitch''i elde turetmeyle ortusuyor', ...
           abs(th - th_hand) < deg2rad(0.1), ok, ...
           sprintf('%.3f deg (elde %.3f)', rad2deg(th), rad2deg(th_hand)));

% --- D3) ILERI BESLEME TEK BASINA (Adim 53) ---------------------------------
% Duvari kaldiran terim budur, ve dogrulugu integratorden BAGIMSIZ sinanmali:
% Ki = 0 iken TEK bir cagri, hicbir gecmis olmadan, dogrudan th_hand vermeli.
% Bu, D1/D2'nin yakalayamayacagi bir hatayi yakalar -- yanlis isaretli ya da
% a0'i eksik bir th_ff'i integral yine de (yavasca) kapatir ve D1/D2 gecerdi.
th_ff_only = cruise_pitch_loop(V, -999, 0, pff);   % Fz girdisi onemsiz: Ki = 0
ok = check('D3) ILERI BESLEME tek basina (Ki=0, gecmis yok) dogru noktada', ...
           abs(th_ff_only - th_hand) < 1e-12, ok, ...
           sprintf('%.4f deg (elde %.4f)', rad2deg(th_ff_only), rad2deg(th_hand)));

% --- E) INTEGRATORUN ZAMAN SABITI, MODEL HATASI ALTINDA ---------------------
% Adim 53 oncesi bu, "yasa hedefe ne kadar surede varir" idi. Ileri beslemeyle
% birlikte o sure ~SIFIRDIR (D3), ve olculmesi gereken sey degisti: integral
% artik yalnizca MODEL HATASINI kapatiyor. Bu yuzden oyuncak modele yasanin
% BILMEDIGI bir hata verilir (gercek oturma acisi a0 + 0.02 rad) ve kalan
% sapmanin kapanma hizi olculur. Turetme ayni: tau = 1/(dL/dtheta * Ki).
% Olcum ILERI BESLEME UYGULANDIKTAN SONRA baslar. Aksi halde ilk tick'teki
% th_ff sicramasi hatanin buyuk kismini bir anda kapatir ve 1/e esigi daha
% integral hic calismadan asilir (olculen 2.4 s, turetilen 17.5 s -- yani
% olculen sey integralin hizi degil, ileri beslemenin adimi olurdu).
da0    = 0.02;                                   % rad, yasanin gormedigi hata
Fz_err = @(th) -(W - qbar*S*p.aero.cla*(p.aero.a0(1) + da0 + th));
st = 0;
[th, st] = cruise_pitch_loop(V, Fz_err(0), st, p);   % 1 tick: th_ff yerlesti
Fz0e = Fz_err(th);                                   % KALAN hata bu
tau_hit = NaN;
for k = 1:round(300/Ts)
    Fz = Fz_err(th);
    if isnan(tau_hit) && abs(Fz - p.tecs.pitch_fz_sp) <= abs(Fz0e - p.tecs.pitch_fz_sp)/exp(1)
        tau_hit = (k-1)*Ts;
    end
    [th, st] = cruise_pitch_loop(V, Fz, st, p);
end
tau_hand = 1/(dLdth * p.tecs.pitch_ki);
ok = check('E0) model hatasi INTEGRAL ile kapaniyor (I -> -da0)', ...
           abs(st + da0) < deg2rad(0.1), ok, ...
           sprintf('I = %.4f rad (beklenen %.4f)', st, -da0));
ok = check('E1) zaman sabiti turetilenle ayni mertebede (0.5x-2x)', ...
           tau_hit > 0.5*tau_hand && tau_hit < 2.0*tau_hand, ok, ...
           sprintf('olculen %.1f s, elde %.1f s', tau_hit, tau_hand));
ok = check('E2) ve irtifa dongusunden (tau ~ 1.7 s) EN AZ 5x yavas', ...
           tau_hit > 5*1.7, ok, sprintf('%.1f s vs 8.5 s', tau_hit));

% --- F) SINIR ---------------------------------------------------------------
st = 0;
for k = 1:round(600/Ts), [th_sat, st] = cruise_pitch_loop(20.0, -200.0, st, p); end
% Adim 53: F1 artik "sinira ULASIYOR mu" sorusunu da soruyor. Kirpma ile
% kosullu integrasyon ayri sinirlara bakarsa cikis sessizce |th_ff| kadar
% eksik doyar (20 m/s'de 6.0 yerine 4.4 deg) ve eski F1 bunu gormezdi.
% Tolerans BIR integral adimi mertebesinde (Ki*|err|*Ts = 1.9e-4 rad): kosullu
% integrasyon sinira son adimin bir tik altinda kilitlenir. Ayirt etmesi
% gereken sey o degil, |th_ff| kadarlik (0.027 rad) sessiz bir eksik doymadir.
ok = check('F1) |pitch| TAM sinirda doyuyor', ...
           abs(abs(th_sat) - p.tecs.pitch_max) < deg2rad(0.05), ok, ...
           sprintf('%.3f deg (sinir %.1f)', rad2deg(th_sat), rad2deg(p.tecs.pitch_max)));
% F2'nin degismezi de TOPLAM uzerinedir: sismeyen sey komut edilen degerdir.
ok = check('F2) anti-windup: TOPLAM (th_ff + I) sinirin otesine sismedi', ...
           abs(th_ff20 + st) <= p.tecs.pitch_max + 1e-12, ok, ...
           sprintf('I = %.4f, th_ff + I = %.4f (sinir %.4f)', st, th_ff20 + st, p.tecs.pitch_max));
% Sinir birakilinca GECIKMELI geri tepme olmamali.
n_back = 0;
for k = 1:round(60/Ts)
    [th_b, st] = cruise_pitch_loop(20.0, +50.0, st, p);
    if th_b < 0, n_back = n_back + 1; end
end
ok = check('F3) sinir birakilinca hemen tepki veriyor (windup yok)', ...
           th_b < th_sat, ok, sprintf('%.3f -> %.3f deg', rad2deg(th_sat), rad2deg(th_b)));

fprintf('\n%s\n', repmat('-', 1, 62));
if ok
    fprintf('SONUC: TUM DENETIMLER GECTI\n');
else
    fprintf('SONUC: ***BASARISIZ***\n');
end

end

function ok = check(name, cond, ok, detail)
if cond
    fprintf('  [ok]   %s   %s\n', name, detail);
else
    fprintf('  [HATA] %s   %s\n', name, detail);
    ok = false;
end
end
