function [G, nu0] = effectiveness_matrix(u, p, qbar)
%EFFECTIVENESS_MATRIX  INDI/WLS icin anlik kontrol etkinligi Jacobian'i.
%
%   u    = [T0;T1;T2; d0;d1;d2; a_ail;a_ele;a_rud]
%          rotor itkileri (N), tilt acilari (rad) ve SANAL kontrol yuzeyi
%          aktuatorleri (rad, istege bagli -- bkz. asagisi)
%   qbar = dinamik basinc 0.5*rho*V^2 (Pa). VERILMEZSE 0 kabul edilir, o zaman
%          yuzey sutunlari TAM OLARAK SIFIRDIR ve matris eski 5x6 davranisiyla
%          birebir aynidir.
%   G    = d(nu)/d(u)  (5x6 veya 5x9),  nu = [taux;tauy;tauz;Fx;Fz]
%   nu0  = u operasyon noktasindaki gercek nu degeri (geometrik, tam nonlineer)
%
% INDI mantigi: her kontrol adiminda G, MEVCUT (olculen/komut edilen) aktuator
% durumunda yeniden hesaplanir — tam analitik model yerine anlik lineerlestirme
% kullanilir, bu da model belirsizliklerine karsi dayanikliligin kaynagidir.
%
% KONTROL YUZEYLERI: UC SANAL AKTUATOR (Adim 46, 2026-08-03) -- madde (V)
% ======================================================================
% NEDEN: Adim 43 seyir zarfini supurdu ve aracin tam kanat-tasimali ucusa
% GECEMEDIGINI olctu; sebep aerodinamik degil MIMARI idi (bes yuzey hic
% oynatilmiyordu). Adim 45 yuzeyleri BAGIMSIZ aktuator olarak tahsisata soktu
% ve IKI KEZ coktu (tilt 44 -> 13.3 deg; sonra arac ters dondu).
%
% *** UCUNCU TASARIMIN DAYANDIGI OLCUM (Adim 31 Faz 0): TILT'I SUREN SEY Fz
% TALEBIDIR. *** Seyirde kanat yukun bir kismini tasir, irtifa dongusu surekli
% "tasimayi AZALT" der (olculen nu_des(Fz) = +2.9 N), ve WLS bunu yapmanin en
% ucuz yolu olarak rotorlari ONE YATIRIR (dFz/ddelta = T*sin d). Yani tilt,
% Fz talebinin bir YAN URUNUDUR.
%
% BUNDAN CIKAN TASARIM KURALI: tahsisata UCUZ BIR Fz AKTUATORU VERILEMEZ --
% verilirse tilt mekanizmasi olur. Adim 45'in 1. denemesi tam olarak buydu:
% simetrik elevon 0.05 m kollu bir TASIMA cihazidir (Fz etkinligi 490 N/rad,
% pitch etkinligi 24.5 N*m/rad -- 20 kat fark), Fz talebini tilt'ten cok daha
% ucuza karsiladi, tilt 13.3 dereceye cokup rotorlar TMAX'a dayandi.
%
% Bu yuzden yuzeyler BAGIMSIZ aktuator DEGIL, UC SANAL aktuatordur:
%
%   a_ail (aileron)  = antisimetrik elevon:  s0 = +a_ail, s1 = -a_ail
%       -> Fz ve tau_y IKISI DE TAM CANCEL (sifirlanmaz -- iki esit ve zit
%          FIZIKSEL kuvvetin toplami gercekten sifirdir). Adim 45'in 2. denemesi
%          bir satiri SIFIRLAYARAK bunu taklit etmeye calisti ve arac ters
%          dondu; fark yapisaldir, ve bu sutun o tuzaga karsi bagisiktir.
%       -> SAF roll: tau_x = -1.2*qbar (rad basina)
%
%   a_ele (elevator) = simetrik servo_2/3:   s2 = s3 = a_ele
%       -> tau_y = +0.806*qbar,  Fz = +1.152*qbar
%          Fz satiri KORUNUR: oran 0.70 m, yani DURUST bir kuyruk kolu.
%          Elevon'un 0.05 m'siyle karistirilmamali -- 14 kat fark, ve ceza
%          agirligi (p.wls.wu_ele) tam da bu orana gore turetildi.
%
%   a_rud (rudder)   = servo_4
%       -> tau_z = -0.142*qbar,  tau_x = +0.023*qbar,  Fz = 0
%
% SIMETRIK ELEVON (flap) BILEREK AKTUATOR DEGILDIR ve 0'da tutulur. SDF'in
% kendi yorumu da bunu soyluyor: "The x8 elevons act as ailerons here (roll
% only) - pitch is the tailplane's job."
%
% MODEL: gz LiftDrag her yuzey icin  F = qbar*area*(rad_to_cl*delta)*e_up
% uygular (cp noktasinda). Sapmaya gore LINEER, yani Jacobian delta'dan
% BAGIMSIZ; yalnizca qbar olcekler. Sabitler p.aero panellerinden gelir
% (tek kaynak) ve plant'in gercek yasasiyla run_aero_panels_test G) maddesinde
% sayisal olarak karsilastirilir.
%
% *** VE BU YAPININ EN GUZEL YANI: MOD DEGISIMI GEREKTIRMEMESI. ***
% Hover'da qbar = 0, yani yuzey sutunlari TAM SIFIR: WLS onlari kullanamaz
% (sifir etkinlikli bir sutunun maliyeti yalnizca Wu*du^2'dir, minimumu du=0)
% ve tahsisat bit-birebir eski davranistir. Hiz arttikca sutunlar buyur ve
% yuzeyler kendiliginden devralir. Bu, kontrolcunun butun felsefesiyle ayni:
% gain-schedule gibi SUREKLI bir karisim, ayri bir durum makinesi degil.

if nargin < 3 || isempty(qbar)
    qbar = 0.0;
end

T  = u(1:3);
de = u(4:6);
r   = p.rotor.pos;      % 3x3, sutunlar rotor pozisyonlari
km  = p.rotor.km;

% Yuzeyler yalnizca u onlari icerecek kadar uzunsa tahsisata girer; boylece
% eski 6 elemanli cagrilar (hover_trim, eski testler) degismeden calisir.
Mv = surf_virtual_map(p);      % 5x3, sanal aktuator -> fiziksel yuzey sapmasi
n_virt = 0;
if numel(u) >= 6 + size(Mv,2)
    n_virt = size(Mv,2);
end

G   = zeros(5, 6 + n_virt);
nu0 = zeros(5,1);

for i = 1:3
    s = sin(de(i)); c = cos(de(i));
    dir_i  = [s; 0; -c];          % itki yonu (FRD)
    ddir_i = [c; 0;  s];          % d(dir)/d(delta)

    f_i   = T(i) * dir_i;
    m_i   = km(i) * T(i) * dir_i;
    ri    = r(:,i);

    tau_i = cross(ri, f_i) + m_i;

    dtau_dT     = cross(ri, dir_i) + km(i)*dir_i;                 % d(tau_i)/dT_i
    dtau_ddelta = (cross(ri, ddir_i) + km(i)*ddir_i) * T(i);      % d(tau_i)/ddelta_i

    % --- nu0 birikimi ---
    nu0(1:3) = nu0(1:3) + tau_i;
    nu0(4)   = nu0(4) + f_i(1);   % Fx
    nu0(5)   = nu0(5) + f_i(3);   % Fz

    % --- Jacobian sutunlari ---
    col_T     = i;      % T_i sutunu
    col_delta = 3 + i;   % delta_i sutunu

    G(1:3, col_T)     = dtau_dT;
    G(1:3, col_delta) = dtau_ddelta;

    G(4, col_T)     = dir_i(1);
    G(4, col_delta) = T(i)*ddir_i(1);

    G(5, col_T)     = dir_i(3);
    G(5, col_delta) = T(i)*ddir_i(3);
end

% --- kontrol yuzeyleri: FIZIKSEL sutunlar, sonra SANAL'a indirgeme (Adim 46) -
% F_j = qbar * k_j * delta_j * e_j   (k_j = alan * rad_to_cl, isaret dahil)
% tau_j = r_j x F_j
% delta'da lineer oldugu icin Jacobian delta'dan bagimsiz; yalnizca qbar
% olcekler. qbar = 0 -> sutunlar tam sifir (yukaridaki nota bakin).
%
% Once BES fiziksel sutun kurulur (Gp, 5x5), sonra Gv = Gp*Mv ile UC sanal
% sutuna indirgenir. Cancel'lar boylece SAYISAL olarak, iki gercek kuvvetin
% toplaminda olusur -- elle "bu satir sifir" yazilmaz. Adim 45'in 2. denemesi
% tam da elle sifirlama yuzunden aracin ters donmesiyle bitmisti.
if n_virt > 0
    Gp = zeros(5, p.surf.n);
    for j = 1:p.surf.n
        dF_dd   = (qbar * p.surf.k(j)) * p.surf.dir(:,j);   % d(F_j)/d(delta_j)
        dtau_dd = cross(p.surf.cp(:,j), dF_dd);

        Gp(1:3, j) = dtau_dd;
        Gp(4,   j) = dF_dd(1);      % Fx
        Gp(5,   j) = dF_dd(3);      % Fz
    end

    Gv = Gp * Mv;                          % 5x3
    G(:, 7:6+n_virt) = Gv;

    % nu0 katkisi: yasa lineer oldugu icin dogrudan sapma x Jacobian.
    av = u(7:6+n_virt);
    nu0 = nu0 + Gv * av(:);
end

end
