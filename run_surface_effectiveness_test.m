function run_surface_effectiveness_test()
%RUN_SURFACE_EFFECTIVENESS_TEST  Sanal yuzey aktuatorlerinin dogrulanmasi.
%
% Adim 46 (2026-08-03), madde (V). Adim 44'un testi bes BAGIMSIZ yuzey sutununu
% denetliyordu; Adim 45 o tasarimi iki kez ucurdu ve ikisi de coktu, bu yuzden
% tasarim UC SANAL aktuatore degisti (bkz. effectiveness_matrix.m basligi).
%
% Bu projede bir turetme "dogrudur" diye kabul edilmez: adim 34'te ayni sinif
% bir turetme (ROTOR_PY isareti) atlanan bir FLU->FRD cevrimi yuzunden aylarca
% "celiski" olarak durmustu. Asagidaki beklenen degerler ELDE, koddan bagimsiz
% turetildi ve acikca yaziliyor.
%
% Denetlenenler:
%   A) NOTRLUK    -- qbar = 0'da yuzey sutunlari TAM sifir, matris eski 5x6 ile ayni
%   B) SUTUNLAR   -- uc sanal sutun elde turetilen degerlerle ortusuyor
%   C) CANCEL     -- aileron'un Fz ve tau_y'si SAYISAL toplamdan sifir cikiyor
%   D) AGIRLIK PENCERESI -- wu_ele turetilen pencerenin ICINDE; ve ayni hesap
%      simetrik elevon icin BOS pencere veriyor (= neden aktuator degil)
%   E) GERIYE UYUMLULUK -- yuzeyler kapaliyken kontrolcu BIREBIR eski davranis

p  = tiltrotor_params();
ok = true;

fprintf('=== sanal yuzey aktuator testi (madde V, Adim 46) ===\n');
Mv = surf_virtual_map(p);
fprintf('fiziksel yuzey = %d, sanal aktuator = %d\n\n', p.surf.n, size(Mv,2));

% --- A) NOTRLUK -------------------------------------------------------------
u6 = [12; 12; 12; 0.2; 0.1; 0.0];
u9 = [u6; zeros(3,1)];

[G6, nu6] = effectiveness_matrix(u6, p);
[G9, nu9] = effectiveness_matrix(u9, p, 0.0);

ok = check('A1) eski 6-elemanli cagri hala 5x6 veriyor', isequal(size(G6), [5 6]), ok, ...
           sprintf('boyut = %dx%d', size(G6,1), size(G6,2)));
ok = check('A2) 9-elemanli cagri 5x9 veriyor', isequal(size(G9), [5 9]), ok, ...
           sprintf('boyut = %dx%d', size(G9,1), size(G9,2)));
ok = check('A3) rotor sutunlari BIREBIR ayni', max(max(abs(G9(:,1:6) - G6))) == 0, ok, ...
           sprintf('max fark = %.3g', max(max(abs(G9(:,1:6) - G6)))));
ok = check('A4) qbar=0''da yuzey sutunlari TAM sifir', all(all(G9(:,7:9) == 0)), ok, ...
           sprintf('max |sutun| = %.3g', max(max(abs(G9(:,7:9))))));
ok = check('A5) nu0 degismiyor', max(abs(nu9 - nu6)) == 0, ok, ...
           sprintf('max fark = %.3g', max(abs(nu9 - nu6))));

% --- B) SUTUNLAR, 15 m/s'de -------------------------------------------------
% ELDE TURETME (kod okunmadan). Yuzey basina dF/ddelta = qbar*k*e,
% k = alan*rad_to_cl, e = FRD "upward"; tau = cp x dF.
%   elevon (k = 0.5*(-4) = -2, e = (0,0,-1)):  dF = (0,0,+2q)
%     sol  cp = (-0.05,-0.30,-0.05): tau_x = cp_y*Fz = -0.6q ; tau_y = -cp_x*Fz = +0.1q
%     sag  cp = (-0.05,+0.30,-0.05): tau_x = +0.6q          ; tau_y = +0.1q
%     AILERON = sol(+a) + sag(-a):
%       tau_x = -0.6q - 0.6q = -1.2q ; tau_y = +0.1q - 0.1q = 0 ; Fz = 2q - 2q = 0
%   elevator (k = 0.048*(-12) = -0.576, e = (0,0,-1)): dF = (0,0,+0.576q) her panel
%     cp_x = -0.70 -> tau_y = -cp_x*Fz = +0.403q her panel; SIMETRIK: +0.806q
%     tau_x: cp_y = -+0.15 -> cancel ; Fz = 2*0.576q = +1.152q
%   rudder (k = 0.032*(-6) = -0.192, e = (0,-1,0)): dF = (0,+0.192q,0)
%     cp = (-0.74,0,-0.12): tau_x = -cp_z*Fy = +0.0230q ; tau_z = cp_x*Fy = -0.1421q
V = 15.0;
q = 0.5 * p.rho * V^2;
[G, ~] = effectiveness_matrix(u9, p, q);

exp_ail = [-1.2*q;    0;         0;        0; 0        ];
exp_ele = [ 0;       +0.806*q;   0;        0; +1.152*q ];
exp_rud = [+0.0230*q; 0;        -0.1421*q; 0; 0        ];
EXP = [exp_ail, exp_ele, exp_rud];
nm  = {'aileron','elevator','rudder'};

fprintf('\n  qbar(%.0f m/s) = %.1f Pa\n', V, q);
fprintf('  sutun        tau_x      tau_y      tau_z         Fx         Fz\n');
for k = 1:3
    c = G(:, 6+k);
    fprintf('  %-10s %9.3f  %9.3f  %9.3f  %9.3f  %9.3f\n', nm{k}, c(1), c(2), c(3), c(4), c(5));
end
for k = 1:3
    c   = G(:, 6+k);
    sc  = max(abs(EXP(:,k)));
    err = max(abs(c - EXP(:,k)))/sc;
    ok  = check(sprintf('B%d) %s sutunu elde turetmeyle ortusuyor (<%%1)', k, nm{k}), ...
                err < 0.01, ok, sprintf('bagil hata = %.5f', err));
end

% --- C) CANCEL SAYISAL MI? --------------------------------------------------
% Adim 45'in 2. denemesi bir satiri ELLE sifirlayarak "kuvvetsiz moment"
% taklidi yapti ve arac ters dondu. Buradaki sifirlar elle yazilmadi; iki
% gercek panelin toplamindan cikiyor. Kaniti: fiziksel matris Gp'nin ilgili
% sutunlari SIFIR DEGIL, ama Gp*Mv sifir.
Gp = zeros(5, p.surf.n);
for j = 1:p.surf.n
    dF = (q * p.surf.k(j)) * p.surf.dir(:,j);
    Gp(1:3,j) = cross(p.surf.cp(:,j), dF);
    Gp(4,j)   = dF(1);
    Gp(5,j)   = dF(3);
end
fprintf('\n  fiziksel elevon Fz etkinligi: sol %+8.2f, sag %+8.2f N/rad\n', Gp(5,1), Gp(5,2));
ok = check('C1) fiziksel elevon sutunlarinin Fz''si sifir DEGIL', ...
           abs(Gp(5,1)) > 100 && abs(Gp(5,2)) > 100, ok, '');
ok = check('C2) ama aileron kombinasyonunda TAM cancel', ...
           abs(G(5,7)) < 1e-9 && abs(G(2,7)) < 1e-9, ok, ...
           sprintf('Fz = %.3g, tau_y = %.3g', G(5,7), G(2,7)));
ok = check('C3) Gv gercekten Gp*Mv (elle yazilmis sutun yok)', ...
           max(max(abs(G(:,7:9) - Gp*Mv))) < 1e-12, ok, ...
           sprintf('max fark = %.3g', max(max(abs(G(:,7:9) - Gp*Mv)))));

% --- D) AGIRLIK PENCERESI ---------------------------------------------------
% Kural (gain_schedule.m): WLS birim-etki cezasini karsilastirir, Wu/|G|.
% Seyirde referans aktuator rotor tilt'idir: T ~ 9 N, wu_tilt_cruise = 1.5
%   Fz    etkinligi ~ T*sin(45 deg) = 6.3 N/rad     -> birim ceza 1.5/6.3 = 0.238
%   pitch etkinligi ~ 0.106*T       = 0.95 N*m/rad  -> birim ceza 1.5/0.95 = 1.58
% Bir yuzey sutununun GUVENLI olmasi icin:
%   ALT SINIR: Fz'yi tilt'ten ucuza CALMAMALI      -> wu > |G_Fz| * 0.238
%   UST SINIR: asil ekseni icin SECILEBILMELI      -> wu < |G_tau| * 1.58
% Pencere BOS ise o sutun hicbir agirlikta guvenli degildir. Adim 45'in 1.
% denemesinin neden coktugunun sayisal ifadesi budur.
c_fz  = 1.5/6.3;
c_tau = 1.5/0.95;
fprintf('\nD) agirlik penceresi (15 m/s):\n');

lo_ele = abs(G(5,8)) * c_fz;
hi_ele = abs(G(2,8)) * c_tau;
fprintf('   elevator       : %6.1f < wu < %6.1f   (secilen wu_ele = %.1f)\n', ...
        lo_ele, hi_ele, p.wls.wu_ele);
ok = check('D1) elevator penceresi BOS DEGIL', hi_ele > lo_ele, ok, '');
ok = check('D2) wu_ele pencerenin ICINDE', ...
           p.wls.wu_ele > lo_ele && p.wls.wu_ele < hi_ele, ok, ...
           sprintf('marj: alt %.2fx, ust %.2fx', p.wls.wu_ele/lo_ele, hi_ele/p.wls.wu_ele));

% Ayni hesap SIMETRIK ELEVON icin -- aktuator OLMAYAN yon.
% s0 = s1 = f  ->  Fz = 2*(2q) = 4q ,  tau_y = 2*(0.1q) = 0.2q
g_flap_fz  = Gp(5,1) + Gp(5,2);
g_flap_tau = Gp(2,1) + Gp(2,2);
lo_flap = abs(g_flap_fz)  * c_fz;
hi_flap = abs(g_flap_tau) * c_tau;
fprintf('   simetrik elevon: %6.1f < wu < %6.1f   <-- TERS, yani pencere BOS\n', ...
        lo_flap, hi_flap);
ok = check('D3) simetrik elevonun penceresi BOS (= neden aktuator degil)', ...
           hi_flap < lo_flap, ok, sprintf('alt %.1f, ust %.1f', lo_flap, hi_flap));

% Aileron'un ALT SINIRI YOKTUR (Fz'si tam sifir), yani ne kadar ucuz olursa
% olsun tilt mekanizmasini calamaz. Bu, sutunun yapisal bagisikligidir.
ok = check('D4) aileron''un Fz''si tam sifir -> alt sinir yok', abs(G(5,7)) < 1e-9, ok, ...
           sprintf('|G_Fz| = %.3g', abs(G(5,7))));

% --- E) GERIYE UYUMLULUK ----------------------------------------------------
% qbar verilmezse kontrolcu 6-aktuatorlu eski koda birebir inmeli.
st = init_ctrl_state();
att_sp = [0.02; -0.01; 0.0];
att    = [0.0; 0.0; 0.0];
om     = [0.01; -0.02; 0.005];
omd    = [0.1; -0.05; 0.02];
F_sp   = [0; -p.m*p.g];
[T1, d1] = indi_attitude_controller(att_sp, att, om, omd, F_sp, u6, st, p, [true;true;false]);
[T2, d2, ~, ~, s2] = indi_attitude_controller(att_sp, att, om, omd, F_sp, u9, st, p, ...
                                              [true;true;false], [], 0.0);
ok = check('E1) qbar=0''da 9-aktuator cikisi 6-aktuatorle BIREBIR ayni', ...
           max(abs(T1-T2)) < 1e-12 && max(abs(d1-d2)) < 1e-12, ok, ...
           sprintf('maxdT = %.3g, maxdd = %.3g', max(abs(T1-T2)), max(abs(d1-d2))));
ok = check('E2) hover''da yuzey komutu TAM sifir', all(s2 == 0), ok, ...
           sprintf('max|surf| = %.3g', max(abs(s2))));

% --- ozet -------------------------------------------------------------------
fprintf('\n%s\n', repmat('-', 1, 62));
if ok
    fprintf('SONUC: TUM DENETIMLER GECTI\n');
else
    fprintf('SONUC: ***BASARISIZ*** -- yukaridaki [HATA] satirlarina bakin\n');
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
