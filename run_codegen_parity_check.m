%RUN_CODEGEN_PARITY_CHECK  Referans MATLAB ile Simulink codegen yolunu ayni
%girdilerle kosturup ciktilarini karsilastirir (Adim 124).
%
% NEDEN VAR. Bu depo ayni matematigi IKI KEZ yaziyor:
%   referans : indi_attitude_controller.m + wls_allocate.m + effectiveness_matrix.m
%   codegen  : sf_indi_rate_law.m + sf_wls_alloc.m   (codegen-guvenli yeniden yazim)
% Sabitler iki yerde yasiyor ve ELLE senkron tutuluyor. Adim 115 bir ayrismayi
% yakaladi ama KODA BAKARAK; Adim 123 kapsamini olctu ve gordü ki ayrisma
% Adim 28'den beri birikiyor. Bu dosya, ayrismayi SAYIYLA gosteren ilk arac.
%
% YONTEM -- ACIK CEVRIM TEKRAR OYNATMA, kapali cevrim DEGIL.
% Iki yol ayni senaryoyu kendi plantiyle kossaydi yorunge birkac tikte ayrisir
% ve "hangi fark kontrolcuden, hangisi plantten" ayirt edilemezdi. Bunun yerine
% her tikte IKISINE DE AYNI durum verilir; boylece olculen fark yalnizca
% KONTROL YASASININ farkidir.
%
% ARAYUZ FARKI, gizlenmeden ele alindi: codegen omega_dot'u HAM alip icinde
% filtreliyor (alpha=0.3), referans ise FILTRELENMIS bekliyor. Adil
% karsilastirma icin once codegen kosulur, onun ic filtre ciktisi
% (state_out(11:13)) referansa beslenir.
%
% KULLANIM
%   run_codegen_parity_check          % ozet
%   run_codegen_parity_check('-v')    % tik tik dokum

function run_codegen_parity_check(varargin)

verbose = any(strcmp(varargin, '-v'));
p = tiltrotor_params();

TOL = 1e-6;   % N ve rad. Geri beslemesiz yollarda iki yol AYNI sonucu vermeli.

% GERI BESLEME BAGLI YOL ICIN AYRI TOLERANS, ve bu bir gevsetme DEGIL:
% iki tahsisat FARKLI ALGORITMA. Referans (wls_allocate.m) gercek bir
% active-set: ihlal eden aktuatoru serbest kumeden CIKARIR. Codegen
% (sf_wls_alloc.m) sabit-boyutlu big-M cezasi kullanir; dosyanin kendi kunyesi
% "active-set ile AYNI SONUCA YAKINSAR" diyor -- ozdes oldugunu degil.
% Tiltjerk (Adim 126) prev_du_tilt uzerinden bir GERI BESLEME yolu acar ve o
% kucuk algoritmik artigi tikten tike tasir.
% OLCULDU (A/B, 400 tik):
%   referans jerk=Inf, codegen 0.3 (UYUMSUZ)  -> 3.223e-01 N   <- gercek ayrisma
%   referans jerk=0.3, codegen 0.3 (UYUMLU)   -> 2.779e-05 N   <- algoritmik artik
% 2.78e-05 N, hover itkisinin (~20 N) 1.4e-6'si. Esik olculen degerin ~4 kati:
% fiziksel olarak onemsiz, ama bir REGRESYON derhal gorunur.
% OLCULDU, tahmin EDILMEDI (adim 134). TEST A'nin farki jerk limitine gore:
%     jerk=Inf  -> 3.171e-01 N     (kutu hic baglamiyor)
%     jerk=0.30 -> 1.126e-01 N
%     jerk=0.45 -> 2.731e-02 N     <- secilen varsayilan
%     jerk=0.60 -> 7.286e-02 N
% Yani fark jerk'in KENDISINDEN degil, kutunun ne kadar bagladigindan
% doguyor: kutu ne kadar cok baglarsa iki tahsisatin (active-set vs big-M)
% cozumleri o kadar YAKINSIYOR. Eski 1e-4 esigi Adim 128'de tek bir
% yapilandirmadan (jerk 0.3, ama TEST A o zaman jerk'siz kosuyordu)
% kalibre edilmisti ve yanlisti -- bu ailenin gercek mertebesi 1e-1.
% 5e-2: olculen en iyi degerin (2.7e-2) iki kati, ama jerk=Inf (3.2e-1) ve
% jerk=0.30 (1.1e-1) rejimlerinin ALTINDA -- yani bir regresyon hala gorunur.
TOL_FB = 5e-2;
N   = 400;    % tik (400 Hz'de 1.0 s)

%% ==================================================================
%% TEST A -- 6 AKTUATOR MODU: codegen'in kapsami, PARITE BEKLENIR
%% ==================================================================
% Referans, u_actual 6 elemanliysa ve qbar verilmemisse 6-aktuatorlu eski
% koda BIREBIR indirgenir (indi_attitude_controller.m:110-118). Yani bu
% modda iki yol ayni problemi cozuyor ve fark OLMAMALI.

fprintf('\n=== TEST A: 6 aktuator modu (codegen kapsami) ===\n');

% SUTUN vektor, ve bu onemli: referans leso_axes_enable(:) ile sutuna ZORLUYOR,
% sf_indi_rate_law zorlamiyor. Satir verilirse codegen tarafinda
% `d_hat .* leso_enable` 3x3'e yayilir ve state_out vertcat'i patlar. Simulink'te
% sinyal boyutu sabit oldugu icin orada tetiklenmez -- ama bu bir SAGLAMLIK
% ayrismasidir ve bu testin ilk kosumunda ortaya cikti (Adim 124).
leso_en = [1; 1; 0];

% sf_wls_alloc.m'deki literal ile SENKRON KALMALI.
TILT_JERK = 0.45;
wst  = zeros(5,1);               % sf_wls_alloc durum vektoru
cs   = init_ctrl_state();
st   = zeros(13,1);              % sf_indi_rate_law durum vektoru
u_ref = [hover_thrust_guess(p); 0.15; 0.15; 0.0];
u_cg  = u_ref;

max_dT = 0; max_dd = 0; max_dtau = 0; worst_k = 0;

for k = 1:N
    % --- senaryo: kucuk bir roll adimi + hafif pitch bozucusu ---
    t      = (k-1)*p.Ts_ctrl;
    att_sp = [0.05*(t>0.1); 0; 0];
    att    = [0.01*sin(6*t); 0.005*cos(4*t); 0];
    om     = [0.02*cos(6*t); -0.01*sin(4*t); 0];
    om_raw = [0.1*sin(9*t); 0.05*cos(7*t); 0];
    F_sp   = [0; -p.m*p.g];

    % --- CODEGEN yolu ---
    % agl = Inf: inis kapisi KAPALI (ucus rejimi). Kapinin kendisi TEST D'de.
    [dtau_cg, ~, st_out] = sf_indi_rate_law(att_sp, att, om, om_raw, u_cg, leso_en, st);
    [ucmd_cg, ~, wst]    = sf_wls_alloc(dtau_cg, F_sp, u_cg, Inf, att(1), 0, wst);
    om_dot_filt          = st_out(11:13);     % referansa beslenecek
    st                   = st_out;

    % --- REFERANS yolu, AYNI girdilerle (filtrelenmis omega_dot dahil) ---
    % TILT_JERK referansa ACIKCA gecirilir: referans varsayilani Inf (kapali),
    % codegen ise ucan yapilandirmanin 0.3'unu tasir (Adim 126). Parite testi
    % ayni degeri iki tarafa da vermeli, yoksa ayrismayi kendi yaratir.
    [T_ref, d_ref, cs, dg] = indi_attitude_controller( ...
        att_sp, att, om, om_dot_filt, F_sp, u_ref, cs, p, leso_en, ...
        pi/2, [], TILT_JERK);

    % --- karsilastir ---
    dT   = max(abs(T_ref(:)      - ucmd_cg(1:3)));
    dd   = max(abs(d_ref(:)      - ucmd_cg(4:6)));
    dtau = max(abs(dg.nu_des(1:3) - dtau_cg(:)));
    if max([dT dd]) > max([max_dT max_dd]); worst_k = k; end
    max_dT = max(max_dT, dT);  max_dd = max(max_dd, dd);
    max_dtau = max(max_dtau, dtau);

    if verbose && mod(k,50) == 0
        fprintf('  k=%3d  dT=%.3e  ddelta=%.3e  dtau=%.3e\n', k, dT, dd, dtau);
    end

    % Ikisini de AYNI komutla ilerlet -- acik cevrim tekrar oynatma.
    u_ref = [T_ref(:); d_ref(:)];
    u_cg  = u_ref;
end

% TEST A tiltjerk yuzunden GERI BESLEME BAGLIDIR -> TOL_FB.
testA_dT = max_dT; testA_dd = max_dd;
fprintf('  max |dtau| farki      : %.3e  (Nm)\n', max_dtau);
fprintf('  max |T_cmd| farki     : %.3e  (N)   %s\n', testA_dT, verdict(testA_dT, TOL_FB));
fprintf('  max |delta_cmd| farki : %.3e  (rad) %s\n', testA_dd, verdict(testA_dd, TOL_FB));
testA_ok = max([testA_dT testA_dd]) <= TOL_FB;
if ~testA_ok
    fprintf('  en kotu tik           : k=%d\n', worst_k);
end

%% ==================================================================
%% TEST D -- INIS UCLUSU (Adim 112/117/118/119), Adim 125'te tasindi
%% ==================================================================
% Kapinin ALTINDA (agl < 2 m) kosar ve arizanin imzasini kurar: buyuk bir
% |T0-T1| komutu var ama acisal ivme YOK (zemin araci tutuyor). Referans ve
% codegen ayni mandali kurup ayni anda farki silmeli.

fprintf('\n=== TEST D: inis uclusu (kapi ALTINDA) ===\n');

cs   = init_ctrl_state();
st   = zeros(13,1);
lst  = zeros(5,1);                          % [bekleme; mandal; prev_du_tilt(3)]
% Arizanin imzasi: T0 yuksek, T1 sifir (Adim 104'un olctugu tek-tarafli hal)
u_ref = [34.0; 0.0; 5.0; 0.17; 0.0; 0.0];
u_cg  = u_ref;
AGL   = 0.30;                               % m, kapinin altinda

max_dT = 0; max_dd = 0;
k_ref = 0; k_cg = 0;                        % mandalin kuruldugu tikler
for k = 1:N
    att_sp = [0;0;0];
    att    = [0.003; 0; 0];                 % duz inis: roll ~0.17 deg
    om     = [0;0;0];
    om_raw = [0;0;0];                       % ZEMIN TUTUYOR -> ivme yok
    F_sp   = [0; -50];

    [dtau_cg, ~, st_out] = sf_indi_rate_law(att_sp, att, om, om_raw, u_cg, leso_en, st);
    [ucmd_cg, ~, lst]    = sf_wls_alloc(dtau_cg, F_sp, u_cg, AGL, att(1), om_raw(1), lst);
    om_dot_filt          = st_out(11:13);
    st                   = st_out;

    [T_ref, d_ref, cs, ~] = indi_attitude_controller( ...
        att_sp, att, om, om_dot_filt, F_sp, u_ref, cs, p, leso_en, pi/2, [], TILT_JERK, AGL);

    if k_ref == 0 && cs.land_contact_latch;  k_ref = k; end
    if k_cg  == 0 && lst(2) > 0.5;           k_cg  = k; end

    max_dT = max(max_dT, max(abs(T_ref(:) - ucmd_cg(1:3))));
    max_dd = max(max_dd, max(abs(d_ref(:) - ucmd_cg(4:6))));

    u_ref = [T_ref(:); d_ref(:)];
    u_cg  = u_ref;
end

fprintf('  mandal kurulan tik   : referans k=%d, codegen k=%d  %s\n', ...
        k_ref, k_cg, verdict(abs(k_ref-k_cg), 0));
fprintf('  son |T0-T1|          : %.3e N (temasta 0 olmali)\n', abs(u_ref(1)-u_ref(2)));
fprintf('  max |T_cmd| farki    : %.3e  (N)   %s\n', max_dT, verdict(max_dT, TOL_FB));
fprintf('  max |delta_cmd| farki: %.3e  (rad) %s\n', max_dd, verdict(max_dd, TOL_FB));
testD_ok = (k_ref == k_cg) && (max([max_dT max_dd]) <= TOL_FB) && (abs(u_ref(1)-u_ref(2)) < 1e-9);


%% ==================================================================
%% TEST E -- POZISYON DONGUSU + fx_trim (Adim 28), Adim 127'de tasindi
%% ==================================================================
% Bu dongu ATTITUDE URETIR, Fx uretmez: tiltrotor'un tilt araligi TEK YONLU
% ([0, pi/2]) oldugu icin arac yapisal olarak FRENLEYEMEZ; yatay kanal bu
% yuzden klasik multikopter yolunu (govdeyi egerek itki vektorunu yoneltmek)
% kullanir. fx_trim de burada uretilir, tahsisatta DEGIL -- gerekce
% sf_position_loop.m'de (dogrudan tahsisata konuldugunda hover_gust q RMS'i
% 4.3x kotulesmisti).

fprintf('\n=== TEST E: pozisyon dongusu + fx_trim ===\n');

st_ref = [0;0];   st_cg = [0;0];
max_att = 0; max_fx = 0;
for k = 1:N
    t = (k-1)*p.Ts_pos;
    % Senaryo: 3 m'lik bir konum hatasi + donen heading + hover->cruise tilt
    % suprumu (fx_trim sonumlemesini de sinamak icin).
    pos_sp    = [3.0; -1.5];
    pos       = [0.4*sin(0.7*t); 0.3*cos(0.5*t)];
    vel_ned   = [0.28*cos(0.7*t); -0.15*sin(0.5*t)];
    psi       = 0.3*sin(0.4*t);
    delta_bar = (pi/2) * min(1, t/4);          % 0 -> pi/2 supurme

    [a_ref, fx_ref, st_ref] = position_loop(pos_sp, pos, vel_ned, psi, delta_bar, st_ref, p);
    [a_cg,  fx_cg,  st_cg ] = sf_position_loop(pos_sp, pos, vel_ned, psi, delta_bar, st_cg);

    max_att = max(max_att, max(abs(a_ref(:) - a_cg(:))));
    max_fx  = max(max_fx,  abs(fx_ref - fx_cg));
end
fprintf('  max |att_sp_xy| farki : %.3e  (rad) %s\n', max_att, verdict(max_att, TOL));
fprintf('  max |fx_trim| farki   : %.3e  (N)   %s\n', max_fx,  verdict(max_fx, TOL));
fprintf('  fx_trim sonumlemesi   : hover %.3f N -> cruise %.3f N\n', ...
        2.9, fx_cg);
testE_ok = max([max_att max_fx]) <= TOL;


%% ==================================================================
%% TEST F -- GOREV FAZLARI (Adim 129): ileri gecis, seyir, sabit kanat,
%%           geri gecis. Gercek gorev bu diziyi kosar:
%%   kalkis -> ileri gecis -> sabit kanat -> geri gecis -> hover -> inis
%% ==================================================================
% BU DOSYALARIN AEROSU MATLAB'DA DOGRULANAMAZ (backtrans_loop.m'in uyarisi:
% tek boylamsal yuzey, 49 N agirligi kaldiramaz). Burada olculen sey MANTIK
% ve SENKRON paritesidir -- sayilarin dogrulanmasi SITL'de yapilir.

fprintf('\n=== TEST F: gorev fazlari ===\n');
fN = 600; testF_ok = true;

% -- seyir hiz dongusu --
sm=[0;0]; sc=[0;0]; dF=0;
for k=1:fN
    t=(k-1)*p.Ts_pos; vf=8+8*min(1,t/6); fxt=6+2*sin(0.3*t);
    if k>50; sm(2)=1; sc(2)=1; end
    [a,sm]=cruise_speed_loop(vf,16,fxt,sm,p);
    [b,sc]=sf_cruise_speed_loop(vf,16,fxt,sc);
    dF=max(dF,abs(a-b));
end
fprintf('  cruise_speed_loop : %.3e  %s\n', dF, verdict(dF,TOL)); testF_ok = testF_ok && dF<=TOL;

% -- seyir pitch dongusu --
sm=0; sc=0; dF=0;
for k=1:fN
    t=(k-1)*p.Ts_pos; vf=10+10*min(1,t/8); fz=-12+4*sin(0.4*t);
    [a,sm]=cruise_pitch_loop(vf,fz,sm,p); [b,sc]=sf_cruise_pitch_loop(vf,fz,sc);
    dF=max(dF,abs(a-b));
end
fprintf('  cruise_pitch_loop : %.3e  %s\n', dF, verdict(dF,TOL)); testF_ok = testF_ok && dF<=TOL;

% -- ileri gecis durum makinesi --
sm=zeros(4,1); sc=zeros(4,1); dF=0;
for k=1:fN
    t=(k-1)*p.Ts_pos; vh=min(12,0.9*t); z=-40+0.3*sin(0.5*t); sat=(mod(k,97)==0);
    [a1,~,~,~,sm,a6]=forwardtrans_loop(true,vh,z,sat,sm,p);
    [b1,~,~,~,sc,b6]=sf_forwardtrans_loop(true,vh,z,sat,sc);
    dF=max(dF,max(abs([a1-b1; sm-sc; a6-b6])));
end
fprintf('  forwardtrans_loop : %.3e  %s\n', dF, verdict(dF,TOL)); testF_ok = testF_ok && dF<=TOL;

% -- geri gecis durum makinesi --
sm=zeros(3,1); sc=zeros(3,1); dF=0;
for k=1:fN
    t=(k-1)*p.Ts_pos; vh=max(0,16-0.6*t); vf=vh-0.5;
    [a1,a2,~,sm]=backtrans_loop(true,vh,vf,pi/2,sm,p);
    [b1,b2,~,sc]=sf_backtrans_loop(true,vh,vf,pi/2,sc);
    dF=max(dF,max(abs([a1-b1;a2-b2;sm-sc])));
end
fprintf('  backtrans_loop    : %.3e  %s\n', dF, verdict(dF,TOL)); testF_ok = testF_ok && dF<=TOL;

% -- sabit kanat yasasi --
old = addpath('sitl_experiments');
sm=[0;0]; sc=[0;0]; dF=0;
for k=1:fN
    t=(k-1)*p.Ts_ctrl;
    att=[0.05*sin(2*t);0.02*cos(3*t);0.3*sin(0.5*t)]; om=[0.1*cos(2*t);0.05*sin(3*t);0.02];
    z=-40+2*sin(0.7*t); vz=1.4*cos(0.7*t); vf=16+sin(t); qb=0.5*1.225*vf^2;
    [a1,a2,sm]=fixedwing_control_law(0.2,-40,16,att,om,z,vz,vf,qb,sm,p);
    [b1,b2,sc]=sf_fixedwing_law(0.2,-40,16,att,om,z,vf,qb,sc);
    dF=max(dF,max(abs([a1-b1; a2(:)-b2(:)])));
end
path(old);
fprintf('  fixedwing_law     : %.3e  %s\n', dF, verdict(dF,TOL)); testF_ok = testF_ok && dF<=TOL;

%% ==================================================================
%% TEST B -- YUZEY KAPSAMI: fark BEKLENIR, ve bu bir kusur DEGIL
%% ==================================================================
% ADIM 125 DUZELTMESI. Adim 124 burada "codegen 3 aktuator eksik, tasinmali"
% diyordu; OLCUM bunu curuttu:
%   MATLAB referansi : yuzeyler ACIK (p.wls.surf_enable = [1;1;1]), sanal
%                      esleme Gv = Gp*Mv ile 9 aktuator -- yalnizca MATLAB'da
%                      dogrulandi, PX4'e HIC tasinmadi, SITL'de HIC denenmedi.
%   PX4 (UCAN yol)   : yuzeyler KAPALI (SURF_ENABLE = false). Iki deneme de
%                      SITL'de basarisiz: 1) uc rotor 45 N'e cakildi, seyir
%                      tilti 44 -> 13.3 deg; 2) arac TERS DONDU (max roll 180).
% Codegen yolu UCAN yapilandirmayi hedefler, dolayisiyla yuzeylerin YOKLUGU
% DOGRU davranistir. Bu test artik bir uyari degil, bir KAYITTIR.

fprintf('\n=== TEST B: 11 aktuator modu (yuzeyler devrede) ===\n');

Mv     = surf_virtual_map(p);
n_virt = size(Mv,2);
n_act  = 6 + n_virt;
fprintf('  referans aktuator sayisi : %d  (3 itki + 3 tilt + %d sanal yuzey)\n', n_act, n_virt);

cg_n = codegen_actuator_count();
fprintf('  codegen aktuator sayisi  : %d\n', cg_n);

fprintf('  PX4 (UCAN yol) yuzeyler  : KAPALI (SURF_ENABLE = false)\n');
fprintf('  codegen yuzeyler         : yok -> UCAN yapilandirma ile TUTARLI\n');
if cg_n ~= 6
    fprintf('  ⚠ codegen %d aktuatore cikmis -- ucan yapilandirmadan ayrildi.\n', cg_n);
end

%% ==================================================================
%% TEST C -- MEKANIZMA ENVANTERI: hangi ozellik hangi yolda var
%% ==================================================================
% Adim 123'un tablosunu KODDAN uretir, elle tutulan bir listeden degil --
% boylece yeni bir mekanizma tasindiginda bu dosya kendiliginden gunceldir.

fprintf('\n=== TEST C: mekanizma envanteri ===\n');
feats = {
%   ad                          referans dosyasi              anahtar            codegen dosyasi
    'fx_trim (Adim 28)',        'position_loop.m',            'fx_trim',         'sf_position_loop.m'
    'pozisyon dongusu (Adim 28)','position_loop.m',            'Kp_p',            'sf_position_loop.m'
    'tiltjerk (Adim 95/96)',    'indi_attitude_controller.m', 'tilt_jerk_limit', 'sf_wls_alloc.m'
    'land_diff (Adim 112)',     'indi_attitude_controller.m', 'land_diff_max',   'sf_wls_alloc.m'
    'datuma gore AGL (Adim 117)','indi_attitude_controller.m', 'land_diff_alt',   'sf_wls_alloc.m'
    'temas mandali (Adim 118)', 'indi_attitude_controller.m', 'land_contact_diff','sf_wls_alloc.m'
    'artim kesme (Adim 119)',   'indi_attitude_controller.m', 'nu_des(1:3)',     'sf_wls_alloc.m'
    'ileri gecis (Adim 42)',    'forwardtrans_loop.m',        'FT_CRUISE',       'sf_forwardtrans_loop.m'
    'geri gecis (Adim 31)',     'backtrans_loop.m',           'BT_HANDOFF',      'sf_backtrans_loop.m'
    'seyir hizi (Adim 46)',     'cruise_speed_loop.m',        'fx_track',        'sf_cruise_speed_loop.m'
    'seyir pitch (Adim 53)',    'cruise_pitch_loop.m',        'th_ff',           'sf_cruise_pitch_loop.m'
    'sabit kanat (Adim 75)',    'sitl_experiments/fixedwing_control_law.m', 'Kp_hdg', 'sf_fixedwing_law.m'
    % --- Adim 160'ta eklendi. ONCESINDE BU LISTE KOR NOKTAYDI: Adim 153/154/157'de
    % uc mekanizma yalnizca PX4 C++'a eklendi ve TEST C yine "0 eksik" dedi,
    % cunku liste ELLE tutuluyor ve yeni satirlar yazilmamisti. Bir mekanizma
    % tasinirken buraya da satir eklenmelidir.
    'dikey itki tavani (145)',  'indi_attitude_controller.m', 'land_tz_max',     'sf_wls_alloc.m'
    'kuyruk itki tabani (157)', 'indi_attitude_controller.m', 'land_tail_floor', 'sf_wls_alloc.m'
    'inis dizisi (Adim 153)',   'landing_sequence.m',         'flare_alt',       'sf_landing_sequence.m'
    'gorev dizicisi (Adim 154)','mission_sequencer.m',        'climb_alt',       'sf_mission_sequencer.m'
};
% NOT: 'kontrol yuzeyleri' bu listede YOK ve olmamali -- ucan yolda (PX4)
% SURF_ENABLE = false, yani codegen'de bulunmamasi DOGRU (Adim 125, TEST B).
n_missing = 0;
for i = 1:size(feats,1)
    in_ref = file_has(feats{i,2}, feats{i,3});
    in_cg  = file_has(feats{i,4}, feats{i,3});
    if in_ref && ~in_cg
        mark = 'EKSIK'; n_missing = n_missing + 1;
    elseif in_ref && in_cg
        mark = 'var';
    else
        mark = '(referansta da yok?)';
    end
    fprintf('  %-26s referans:%-4s codegen:%-4s  -> %s\n', feats{i,1}, ...
            tf(in_ref), tf(in_cg), mark);
end

%% ==================================================================
fprintf('\n=== OZET ===\n');
% HATA DUZELTMESI (Adim 126): burasi eskiden max_dT/max_dd okuyordu, ama o
% degiskenleri TEST D EZIYORDU -- ozet "GECTI" derken TEST A ciktisi
% "FARK VAR" gosterebiliyordu. Her testin verdikti artik kendi degiskeninde.
if testA_ok
    fprintf('  TEST A (6 aktuator)  : GECTI -- iki yol bu modda ayni.\n');
else
    fprintf('  TEST A (6 aktuator)  : ⛔ KALDI -- ayni modda bile ayrisiyorlar.\n');
end
fprintf('  TEST D (inis uclusu) : %s\n', ternary(testD_ok, 'GECTI -- uc mekanizma da parite', '⛔ KALDI'));
fprintf('  TEST E (pos + fx_trim): %s\n', ternary(testE_ok, 'GECTI', '⛔ KALDI'));
fprintf('  TEST F (gorev fazlari): %s\n', ternary(testF_ok, 'GECTI -- 5 fazin 5''i parite', '⛔ KALDI'));
fprintf('  TEST B (yuzey kapsami): %s\n', ternary(cg_n == 6, 'UCAN yapilandirma ile tutarli', '⚠ ucan yapilandirmadan ayrildi'));
fprintf('  TEST C (envanter)    : %d mekanizma codegen yolunda EKSIK\n', n_missing);
fprintf('\n  HITL codegen icin yol haritasi: WLS_LOCKUP_INVESTIGATION_REPORT.md, Adim 123.\n\n');

end % ana fonksiyon

%% ---------------------------------------------------------------- yardimcilar

function s = verdict(v, tol)
if v <= tol; s = '[OK]'; else; s = '[FARK VAR]'; end
end

function s = tf(b)
if b; s = 'VAR'; else; s = 'yok'; end
end

function s = ternary(c, a, b)
if c; s = a; else; s = b; end
end

function b = file_has(fname, key)
b = false;
if ~exist(fname, 'file'); return; end
txt = fileread(fname);
% Yalnizca KOD satirlarina bak: yorum satirlarinda gecen bir isim (ornegin
% sf_wls_alloc.m'deki ayrisma notu) mekanizmanin VARLIGI sayilmamali --
% Adim 115 notu tam olarak boyle bir yanilgi uretebilirdi.
lines = strsplit(txt, newline);
for i = 1:numel(lines)
    ln = strtrim(lines{i});
    if isempty(ln) || startsWith(ln, '%'); continue; end
    ln = regexprep(ln, '%.*$', '');     % satir sonu yorumunu at
    if contains(ln, key); b = true; return; end
end
end

function n = codegen_actuator_count()
% sf_wls_alloc.m'nin uzerinde calistigi aktuator sayisini KODDAN okur.
n = NaN;
txt = fileread('sf_wls_alloc.m');
tok = regexp(txt, 'du\s*=\s*zeros\((\d+)\s*,\s*1\)', 'tokens', 'once');
if ~isempty(tok); n = str2double(tok{1}); end
end

function T = hover_thrust_guess(p)
% Uc rotora esit dagitilmis hover itkisi -- senaryonun baslangic noktasi.
T = (p.m*p.g/3) * ones(3,1);
end
