function run_aero_panels_test()
%RUN_AERO_PANELS_TEST  Adim 46: yeni bes-panelli plant aerodinamiginin denetimi.
%
% Bu testin varlik sebebi Adim 45'in dersidir: "bir sabiti dogru transkribe
% etmek, ne anlama geldigini anlamak degildir." Bu yuzden asagidaki beklenen
% degerlerin HEPSI elde, koddan bagimsiz turetildi ve acikca yaziliyor.
%
% Denetlenen alt basliklar:
%   A) ISARET   -- burun yukari TASIMA uretmeli (eski plant'in TERS oldugu yer)
%   B) BUYUKLUK -- kanat tasimasi analitik q*S*cla*(a0+theta) ile ortusmeli
%   C) NOTRLUK  -- sifir hizda hicbir panel kuvvet uretmemeli
%   D) AILERON  -- antisimetrik elevon SAF roll olmali (Fz ve pitch TAM cancel)
%   E) ELEVATOR -- simetrik elevator 0.70 m kolla pitch uretmeli
%   F) RUDDER   -- yaw uretmeli, isareti dogru olmali
%   G) TUTARLILIK -- plant'in gercek yasasi ile kontrolcunun (effectiveness_
%      matrix) kucuk-alpha lineerlestirmesi ayni sayiyi vermeli
%   H) SITL CAPRAZ KONTROLU -- Adim 43'un olctugu "14.4 m/s'de kanat agirligin
%      %41-50'sini tasiyor" degeriyle tutarli mi

p  = tiltrotor_params();
ok = true;
W  = p.m * p.g;

fprintf('=== aero panel testi (Adim 46) ===\n');
fprintf('agirlik = %.2f N, panel sayisi = %d\n\n', W, p.aero.n);

z5 = zeros(5,1);
w0 = zeros(3,1);

% --- A) ISARET --------------------------------------------------------------
% Seviye ucus, burun yukari theta: govde hizi v_b = R_eb' * [V;0;0].
% Beklenen: theta buyudukce Fz DAHA NEGATIF (FRD'de negatif = YUKARI).
fprintf('A) isaret: 15 m/s, seviye ucus, burun yukari taramasi\n');
V = 15; Fz_prev = inf; a_sign_ok = true;
for th = [-5 0 5 10]*pi/180
    v_b = [V*cos(th); 0; V*sin(th)];
    [F, ~] = aero_panels(v_b, w0, z5, p);
    fprintf('   theta=%+5.1f deg -> Fz = %+8.2f N (negatif=yukari), Fx = %+7.2f N\n', ...
            th*180/pi, F(3), F(1));
    if F(3) >= Fz_prev, a_sign_ok = false; end
    Fz_prev = F(3);
end
ok = check('A1) tasima theta ile MONOTON artiyor (Fz azaliyor)', a_sign_ok, ok, '');
v_b = [V; 0; 0];
[F0, ~] = aero_panels(v_b, w0, z5, p);
ok = check('A2) theta=0''da tasima YUKARI (a0 oturma acisi)', F0(3) < 0, ok, ...
           sprintf('Fz = %+.2f N', F0(3)));

% --- B) BUYUKLUK ------------------------------------------------------------
% ELDE TURETME (yalnizca iki kanat yarisi, kuyruk haric):
%   q = 0.5*1.2041*V^2, S = 2*0.5 = 1.0 m^2
%   L = q*S*cla*(a0 + theta),  theta = 0 icin:
%   V=15 -> q = 135.46 -> L = 135.46*1.0*4.752799*0.05984281 = 38.53 N
fprintf('\nB) buyukluk: theta=0, V=15 m/s, yalniz kanat panelleri\n');
q15 = 0.5*p.aero.rho*15^2;
L_hand = q15 * 1.0 * p.aero.cla * p.aero.a0(1);
% kanat panellerini tek basina olcmek icin kuyruk alanlarini gecici sifirla
pw = p; pw.aero.area(3:5) = 0;
[Fw, ~] = aero_panels([15;0;0], w0, z5, pw);
fprintf('   elde: %.2f N   plant: %.2f N   (fark %.3f N)\n', L_hand, -Fw(3), abs(L_hand+Fw(3)));
ok = check('B1) kanat tasimasi elde turetmeyle ortusuyor (<%1 hata)', ...
           abs(-Fw(3) - L_hand)/L_hand < 0.01, ok, ...
           sprintf('bagil hata = %.4f', abs(-Fw(3)-L_hand)/L_hand));

% --- C) NOTRLUK -------------------------------------------------------------
[Fz0, Mz0] = aero_panels([0;0;0], w0, z5, p);
ok = check('C1) sifir hizda kuvvet TAM sifir', all(Fz0 == 0) && all(Mz0 == 0), ok, ...
           sprintf('max|F| = %.3g', max(abs(Fz0))));
[Fz1, ~] = aero_panels([0;0;0], w0, [0.5;-0.5;0.3;0.3;0.2], p);
ok = check('C2) sifir hizda yuzey sapmasi da etkisiz', all(Fz1 == 0), ok, ...
           sprintf('max|F| = %.3g', max(abs(Fz1))));

% --- D) AILERON (antisimetrik elevon) --------------------------------------
% ELDE TURETME: sol panel cp_y = -0.30, sag +0.30 (FRD). Her ikisinin de
% dF/ddelta'si ayni yonde (0,0,+2q) [k = 0.5*(-4) = -2, e = (0,0,-1)].
%   tau_x = cp_y * Fz  -> sol -0.30*Fz_L, sag +0.30*Fz_R
%   antisimetrik (dL=+a, dR=-a): Fz_L = +2qa, Fz_R = -2qa
%     => tau_x = -0.30*2qa + 0.30*(-2qa) = -1.2*q*a      [SAF roll]
%     => Fz toplam = 2qa - 2qa = 0                        [TAM cancel]
%     => tau_y = -cp_x*Fz -> +0.05*(2qa) + 0.05*(-2qa) = 0 [TAM cancel]
fprintf('\nD) aileron: antisimetrik elevon, V=15 m/s\n');
a = 0.10;
[Fa, Ma] = aero_panels([15;0;0], w0, [a;-a;0;0;0], p);
[Fb, Mb] = aero_panels([15;0;0], w0, z5, p);
dF = Fa - Fb; dM = Ma - Mb;
tau_x_hand = -1.2 * q15 * a;
fprintf('   dtau_x elde = %+8.3f  plant = %+8.3f N*m\n', tau_x_hand, dM(1));
fprintf('   dFz = %+.4f N, dtau_y = %+.4f N*m  (ikisi de ~0 olmali)\n', dF(3), dM(2));
ok = check('D1) dtau_x elde turetmeyle ortusuyor (<%2)', ...
           abs(dM(1) - tau_x_hand)/abs(tau_x_hand) < 0.02, ok, ...
           sprintf('bagil hata = %.4f', abs(dM(1)-tau_x_hand)/abs(tau_x_hand)));
ok = check('D2) dFz cancel oluyor (|dFz| < 0.01 N)', abs(dF(3)) < 0.01, ok, ...
           sprintf('dFz = %.4g N', dF(3)));
ok = check('D3) dtau_y cancel oluyor (|dtau_y| < 0.01 N*m)', abs(dM(2)) < 0.01, ok, ...
           sprintf('dtau_y = %.4g', dM(2)));

% --- E) ELEVATOR (simetrik) -------------------------------------------------
% ELDE TURETME: k = 0.048*(-12) = -0.576, e = (0,0,-1), cp_x = -0.70
%   dFz/ddelta (panel basina) = +0.576*q ; iki panel = +1.152*q
%   tau_y = -cp_x * Fz = +0.70 * 1.152*q*e = +0.806*q*e
fprintf('\nE) elevator: simetrik, V=15 m/s\n');
e = 0.10;
[Fe, Me] = aero_panels([15;0;0], w0, [0;0;e;e;0], p);
dFe = Fe - Fb; dMe = Me - Mb;
tau_y_hand = 0.806 * q15 * e;
Fz_hand    = 1.152 * q15 * e;
fprintf('   dtau_y elde = %+8.3f  plant = %+8.3f N*m\n', tau_y_hand, dMe(2));
fprintf('   dFz    elde = %+8.3f  plant = %+8.3f N\n', Fz_hand, dFe(3));
ok = check('E1) elevator pitch momenti (<%3 hata)', ...
           abs(dMe(2) - tau_y_hand)/abs(tau_y_hand) < 0.03, ok, ...
           sprintf('bagil hata = %.4f', abs(dMe(2)-tau_y_hand)/abs(tau_y_hand)));
ok = check('E2) elevator Fz''si GERCEK ve modellenen isarette', dFe(3) > 0, ok, ...
           sprintf('dFz = %+.2f N', dFe(3)));

% --- F) RUDDER --------------------------------------------------------------
% ELDE TURETME: k = 0.032*(-6) = -0.192, e = (0,-1,0), cp = (-0.74, 0, -0.12)
%   dFy/ddelta = +0.192*q
%   tau_z = cp_x*Fy - cp_y*Fx = -0.74*0.192*q = -0.142*q
fprintf('\nF) rudder, V=15 m/s\n');
r = 0.10;
[Fr, Mr] = aero_panels([15;0;0], w0, [0;0;0;0;r], p);
dFr = Fr - Fb; dMr = Mr - Mb;
tau_z_hand = -0.142 * q15 * r;
fprintf('   dtau_z elde = %+8.3f  plant = %+8.3f N*m,  dFy = %+7.3f N\n', ...
        tau_z_hand, dMr(3), dFr(2));
ok = check('F1) rudder yaw momenti (<%3 hata)', ...
           abs(dMr(3) - tau_z_hand)/abs(tau_z_hand) < 0.03, ok, ...
           sprintf('bagil hata = %.4f', abs(dMr(3)-tau_z_hand)/abs(tau_z_hand)));

% --- G) KONTROLCU MODELIYLE TUTARLILIK -------------------------------------
% effectiveness_matrix'in yuzey sutunlari kucuk-alpha lineerlestirmesidir;
% plant'in gercek yasasiyla ayni sayiyi vermeli (aksi halde kontrolcu, plant'ta
% olmayan bir otoriteye guvenir -- bu projede Adim 11/21/27'nin hata sinifi).
fprintf('\nG) plant <-> effectiveness_matrix tutarliligi (V=15 m/s)\n');
% Adim 46'dan beri effectiveness_matrix UC SANAL sutun veriyor (5x9), fiziksel
% bes sutunu degil. Karsilastirma bu yuzden FIZIKSEL Jacobian uzerinden yapilir:
% kontrolcunun kullandigi ayni p.surf sabitleriyle Gp kurulur ve plant'in sonlu
% farkiyla karsilastirilir. (Sanal sutunlarin Gp*Mv oldugu ayrica
% run_surface_effectiveness_test C3'te denetleniyor.)
Gp = zeros(5, p.surf.n);
for j = 1:p.surf.n
    dF = (q15 * p.surf.k(j)) * p.surf.dir(:,j);
    Gp(1:3,j) = cross(p.surf.cp(:,j), dF);
    Gp(4,j)   = dF(1);
    Gp(5,j)   = dF(3);
end
dd = 1e-4;
max_rel = 0;
for j = 1:p.surf.n
    sj = z5; sj(j) = dd;
    [Fp, Mp] = aero_panels([15;0;0], w0, sj, p);
    num = [(Mp-Mb)/dd; (Fp(1)-Fb(1))/dd; (Fp(3)-Fb(3))/dd];
    ana = Gp(:, j);
    sc  = max(1, max(abs(ana)));
    rel = max(abs(num - ana))/sc;
    max_rel = max(max_rel, rel);
    fprintf('   yuzey %d: max bagil fark = %.4f\n', j, rel);
end
ok = check('G1) tum yuzeylerde plant ile model %5''ten yakin', max_rel < 0.05, ok, ...
           sprintf('en kotu = %.4f', max_rel));

% --- H) SITL CAPRAZ KONTROLU -----------------------------------------------
% Adim 43 SITL'de olctu: 14.4 m/s seyirde kanat agirligin %41-50'sini tasiyor.
% Yeni plant o rejimde ayni mertebeyi vermeli. Seyirde arac hafif burun-asagi
% oturuyor; theta'yi tarayip hangi theta'nin olculen orani verdigi yazilir.
fprintf('\nH) SITL capraz kontrolu: 14.4 m/s''de kanat yuk payi\n');
Vc = 14.4;
for th = [-3 -1.5 0 1.5 3]*pi/180
    v_b = [Vc*cos(th); 0; Vc*sin(th)];
    [Fc, ~] = aero_panels(v_b, w0, z5, pw);   % pw = yalniz kanat
    fprintf('   theta=%+5.1f deg -> kanat yuk payi = %5.1f %%\n', th*180/pi, -Fc(3)/W*100);
end
[Fc0, ~] = aero_panels([Vc;0;0], w0, z5, pw);
frac0 = -Fc0(3)/W*100;
ok = check('H1) theta=0 payi SITL bandiyla ayni mertebede (%30-90)', ...
           frac0 > 30 && frac0 < 90, ok, sprintf('%.1f %%', frac0));

% --- ozet -------------------------------------------------------------------
fprintf('\n');
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
