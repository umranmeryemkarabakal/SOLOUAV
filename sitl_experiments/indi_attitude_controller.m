function [T_cmd, delta_cmd, ctrl_state, diagn, surf_cmd] = indi_attitude_controller( ...
        att_sp, att, omega, omega_dot_meas, F_sp, u_actual, ctrl_state, p, leso_axes_enable, ...
        tilt_ceil, qbar, tilt_jerk_limit)
%INDI_ATTITUDE_CONTROLLER  Outer attitude (P) + inner rate (INDI) + LESO +
%WLS control allocation — 3 tilt-rotorlu VTOL icin tek adimlik kontrolcu.
%
% Girisler:
%   att_sp   [phi;theta;psi]_sp        (rad)   attitude referansi
%   att      [phi;theta;psi]           (rad)   olculen attitude
%   omega    [p;q;r]                   (rad/s) olculen govde acisal hizi
%   omega_dot_meas [pdot;qdot;rdot]    (rad/s^2) filtrelenmis olculen acisal ivme
%                                       (INDI'nin "artimli" geri beslemesi)
%   F_sp     [Fx_sp; Fz_sp]            (N)     disaridaki (irtifa/hiz) dongusunden
%                                       istenen govde-eksen kuvvet setpoint'i
%   u_actual [T0;T1;T2;d0;d1;d2]       aktuatorlerin GERCEK (olculen) durumu —
%                                       INDI/WLS'in lineerlestirme noktasi.
%                                       9 ELEMANLI verilirse son uc eleman
%                                       SANAL yuzey aktuatorleridir
%                                       [a_ail;a_ele;a_rud] (bkz. asagisi).
%   ctrl_state  bkz. init_ctrl_state.m
%   leso_axes_enable  3x1 logical, orn. [true;true;false] — sohbette onerildigi
%                     gibi baslangicta sadece roll/pitch'te LESO aktif
%   tilt_ceil (rad)   ISTEGE BAGLI. Kanat rotorlerinin tilt kutusu ust siniri
%                     (geri gecis, madde B5 — bkz. backtrans_loop.m). Verilmezse
%                     p.tilt.max kullanilir ve davranis DEGISMEZ; mevcut tum
%                     cagiricilar bu yuzden oldugu gibi calisir.
%   qbar (Pa)         ISTEGE BAGLI dinamik basinc 0.5*rho*V_air^2. Yalnizca
%                     u_actual 9 elemanli VE qbar verilmisse kontrol yuzeyleri
%                     tahsisata girer; aksi halde davranis 6-aktuatorlu eski
%                     haliyle BIREBIR aynidir (madde V, Adim 46).
%
% Ciktilar:
%   T_cmd, delta_cmd   yeni aktuator komutlari (plant aktuator gecikmesine girer)
%   ctrl_state          guncellenmis kontrolcu durumu
%   diagn               tani/loglama icin ara degiskenler (struct)
%   surf_cmd  (5x1)     FIZIKSEL kontrol yuzeyi komutlari (rad), sanal
%                       aktuatorlerden surf_virtual_map ile turetilir. Yuzeyler
%                       kapaliysa sifirdir.

%% --- 0) Gain-scheduling (ortalama tilt acisina gore) ---
delta_bar = mean(u_actual(4:6));
sched = gain_schedule(delta_bar, p);

%% --- 1) Disaridaki attitude P dongusu ---
e_att = att_sp - att;
e_att(3) = atan2(sin(e_att(3)), cos(e_att(3)));   % yaw hata sarmalama

% Rate limiti (guvenlik doygunu). EKSEN BAZLI (2026-07-27, Adim 13 -- eskiden
% her eksen icin tek skaler 3.0 idi). Yaw ekseninin gercek otoritesi cok daha
% dusuk: yaw torku yalnizca diferansiyel kanat tilt'inden gelir ve tilt slew
% limitiyle (p.tilt.rate_max) sinirlidir. Yaw hala 3.0 rad/s'lik bir setpoint
% gorebiliyorken, arac donerken yaw hatasi +-180'de sarmalandigi icin bu
% setpoint surekli isaret degistiriyor ve dis dongu, ic dongunun donusu
% sonumlemesini engelleyip DONUSU BESLIYOR (SITL'de olculdu: wsp2 = -1.01,
% +1.01, +2.07, -3.00, +3.00 ... bkz. sitl/WLS_LOCKUP_INVESTIGATION_REPORT.md
% Adim 12g). Limit gercek donus hizlarinin (1-3.5 rad/s) ALTINDA olursa hata
% isaretini olculen hiz belirler ve ic dongu her zaman sonumleme yonunde
% calisir.
omega_sp = sched.Kp_att .* e_att;
omega_sp = max(min(omega_sp, p.ctrl.rate_sp_limit), -p.ctrl.rate_sp_limit);

%% --- 2) Ic INDI rate dongusu: istenen acisal ivme ---
e_omega = omega_sp - omega;
omega_dot_des = sched.Kp_rate .* e_omega;

%% --- 3) LESO guncelleme (decimasyonlu, sadece etkin eksenler) ---
ctrl_state.leso_accum = ctrl_state.leso_accum + p.Ts_ctrl;
if ctrl_state.leso_accum >= p.Ts_leso - 1e-12
    wo = 15;   % rad/s, gozlemci bant genisligi (rate-loop kapanma hizinin altinda)
    [beta1, beta2] = leso_bandwidth_gains(wo);
    for ax = 1:3
        if leso_axes_enable(ax)
            [ctrl_state.z1(ax), ctrl_state.z2(ax)] = leso_axis_update( ...
                ctrl_state.z1(ax), ctrl_state.z2(ax), omega(ax), ...
                ctrl_state.prev_u_leso(ax), beta1, beta2, p.Ts_leso);
            ctrl_state.d_hat(ax) = ctrl_state.z2(ax);
        end
    end
    ctrl_state.leso_accum = ctrl_state.leso_accum - p.Ts_leso;
end

d_hat_active = ctrl_state.d_hat .* leso_axes_enable(:);
omega_dot_des_adj = omega_dot_des - d_hat_active;

% ESO'nun "girisi" (ADRC'de b0*u), plant'a GERCEKTEN uygulanmasi ISTENEN
% (bozucu-telafili) ivme olmali — telafi ONCESI deger kullanilirsa, ESO
% kendi telafisini her donguede "gormemis" gibi davranir ve z2 sinirsizca
% surukleyerek (drift) sahte bir bozucu birikimine yol acar.
ctrl_state.prev_u_leso = omega_dot_des_adj;

%% --- 4) INDI artimli kontrol kanunu ---
domega_dot = omega_dot_des_adj - omega_dot_meas;
dtau = p.I * domega_dot;                          % (3x1) moment artisi

%% --- 5) WLS kontrol tahsisi ---
% KONTROL YUZEYLERI (madde V, Adim 46): yalnizca u_actual sanal aktuatorleri
% TASIYORSA ve qbar VERILMISSE devreye girer. Ikisi birden yoksa n_virt = 0
% ve asagidaki her sey 6-aktuatorlu eski koda birebir indirgenir.
Mv = surf_virtual_map(p);
n_virt = 0;
if numel(u_actual) >= 6 + size(Mv,2) && nargin >= 11 && ~isempty(qbar)
    n_virt = size(Mv,2);
else
    qbar = 0.0;
    u_actual = u_actual(1:6);
end
n_act = 6 + n_virt;

[G, nu0] = effectiveness_matrix(u_actual, p, qbar);

% NOT (madde (P), 2026-07-29 Adim 28): madde (P)'nin cozumu olan "Fx trim"
% BILEREK burada DEGIL, position_loop.m'de uygulanir ve F_sp(1) ile buraya
% gelir. Gerekce yapisal: trim'in bedeli kalici bir +Fx'tir ve onu ancak yatay
% pozisyon dongusu tasiyabilir (madde (N)). Ilk denemede trim dogrudan bu
% satira konuldu; hover_gust regresyonu q RMS'i 0.0004 -> 0.0013'e (4.3x)
% kotulestirdi, cunku o test pozisyon dongusu KULLANMIYOR ve olusan ileri
% kuvveti tasiyacak hicbir sey yoktu. Bagimliligi kodun yapisina gomerek
% ((P) yalnizca (N) aktifken uygulanir) hem regresyon kalkti hem de niyet
% okunur oldu. Bkz. tiltrotor_params.m p.ctrl.fx_trim.
dF = F_sp(:) - nu0(4:5);
nu_des = [dtau; dF];

Tmin = p.rotor.Tmin; Tmax = p.rotor.Tmax;
dmin = p.tilt.min;   dmax = p.tilt.max;

% KANAT TILT TAVANI (2026-07-29, Adim 31 -- madde B5, geri gecis).
% Yalnizca kanat rotorlerine (4. ve 5. kanal) uygulanir; KUYRUK BILEREK
% kisitlanmaz -- olculdu ki kuyruk tilt'i seyirde bile 0.5-0.7 deg'de duruyor
% ve Fx'e katki vermiyor, buna karsilik ucunun en guclu pitch aktuatoru
% (dM_y/ddelta = T*(-0.07*cos d + 0.65*sin d), kanat icin 43 deg'de -0.106*T).
% Varsayilan p.tilt.max, yani cagirici vermezse DAVRANIS AYNEN ESKISI GIBI.
% Gerekce ve olcumler: backtrans_loop.m.
if nargin < 10 || isempty(tilt_ceil)
    tilt_ceil = p.tilt.max;
end
dmax_wing = min(dmax, tilt_ceil);

% tiltjerk (Adim 95/96) -- ISTEGE BAGLI. Varsayilan Inf = KAPALI, davranis
% AYNEN ESKISI GIBI (mevcut hicbir cagirici bunu vermiyor). Bkz. asagida
% du_min/du_max'a uygulandigi yer.
if nargin < 12 || isempty(tilt_jerk_limit)
    tilt_jerk_limit = Inf;
end
if ~isfield(ctrl_state, 'prev_du_tilt')
    ctrl_state.prev_du_tilt = zeros(3,1);   % geriye uyumluluk: eski ctrl_state'ler
end

% SANAL YUZEY KUTULARI (Adim 46). Mutlak sinir, sanal aktuatorun surdugu
% FIZIKSEL servolarin eklem limitlerinden turer: a_k'nin izin verilen genligi,
% |Mv(j,k)| katsayisiyla suruldugu her j servosunun limitiyle sinirlidir.
% Aileron icin bu min(0.78, 0.78) = 0.78; elevator ve rudder icin 0.52.
% Elle yazilmaz -- yanlis bir katsayi degisikligi kutuyu sessizce genisletmesin.
a_max = zeros(n_virt,1);
for k = 1:n_virt
    lim = inf;
    for j = 1:p.surf.n
        if Mv(j,k) ~= 0
            lim = min(lim, p.surf.max(j)/abs(Mv(j,k)));
        end
    end
    a_max(k) = lim;
end

% KANAT ROTORU ITKI TABANI (Adim 50, 2026-08-04). Seyirde kanat rotorlerinin
% itkisi sifira inemez, cunku tilt sutunu ITKIYLE CARPILIDIR
% (dtau/ddelta ~ T) -- T = 0 olan bir tilt rotorunun otoritesi TAM SIFIRDIR ve
% tahsisat o aktuatoru kaybeder. Olculdu: Adim 49'da arac tam T0 = 0.00 N'de
% iraksadi. Gerekce ve deger secimi tiltrotor_params.m'de.
% Taban HOVER'DA TAM SIFIRDIR (sched.smooth = 0), yani hover davranisi
% birebir degismez; ortalama tilt ile smoothstep olarak acilir.
Tmin_wing = Tmin + sched.smooth * p.rotor.Tmin_cruise;

abs_lo = [Tmin_wing;Tmin_wing;Tmin; dmin;dmin;dmin; -a_max] - u_actual;
abs_hi = [Tmax;Tmax;Tmax; dmax_wing;dmax_wing;dmax; a_max] - u_actual;

% Ek guvenlik: tek adimda asiri buyuk komut sicramasini onlemek icin, servo/motor
% slew limitine gore de bir kutu kisiti uygula (aktuator gecikmesi zaten var,
% ama allocator'in gecerli araligi da fizik disina cikmasin).
% TILT KUTUSU: p.tilt.rate_max DEGIL p.tilt.slew_box_rate kullanilir (2026-07-29,
% Adim 27 — PX4 Adim 22 ayriminin MATLAB karsiligi). Birincisi plant'in fiziksel
% servo clamp'i, ikincisi tahsisatin tek tick'te isteyebilecegi artis; gerekcesi
% ve olculmus baglama orani icin bkz. tiltrotor_params.m.
rate_lo = [-Tmax/p.rotor.tau_up*p.Ts_ctrl*5*ones(3,1); -p.tilt.slew_box_rate*p.Ts_ctrl*ones(3,1); ...
           -p.surf.rate_max*p.Ts_ctrl*ones(n_virt,1)];
rate_hi = [ Tmax/p.rotor.tau_down*p.Ts_ctrl*5*ones(3,1); p.tilt.slew_box_rate*p.Ts_ctrl*ones(3,1); ...
            p.surf.rate_max*p.Ts_ctrl*ones(n_virt,1)];

du_min = max(abs_lo, rate_lo);
du_max = min(abs_hi, rate_hi);

% Tavan slew limitinin ICINDEN gecirilir, onun YERINE degil (2026-07-29, Adim 31).
% Tavan mevcut tilt'in altina inince abs_hi negatiflesir ve rate_lo'nun altina
% dusebilir, yani kutu BOSALIR. Asagidaki genel guvence bunu du_min'i abs_hi'ye
% cekerek cozerdi -- kalan tum farki TEK tick'te komut ederek, slew limitinin
% kat kat otesinde. O guvence, abs_hi = dmax - u >= 0 iken kutunun her zaman
% 0'i icerdigi varsayimiyla yazilmisti. Ters yonde cozuluyor: tilt tavanin
% altina donene kadar TAM OLARAK tahsisat slew hiziyla geri cekilir.
du_max(4:5) = max(du_max(4:5), du_min(4:5));

% tiltjerk (Adim 95/96): slew KUTUSUNUN kendisini KUCULTMEZ (o "(Q)" tahsisat-
% acligini yeniden acar, bkz. C++ tarafindaki ayni notun gerekcesi) -- bir
% turev ustunde, bu tick'in du'sunun BIR ONCEKI tick'in du'sundan ne kadar
% uzaklasabilecegini sinirlar. Uc tilt de (4,5,6. satirlar).
if isfinite(tilt_jerk_limit)
    jerk_max = tilt_jerk_limit * p.Ts_ctrl;
    du_min(4:6) = max(du_min(4:6), ctrl_state.prev_du_tilt - jerk_max);
    du_max(4:6) = min(du_max(4:6), ctrl_state.prev_du_tilt + jerk_max);
end

du_min = min(du_min, du_max);   % sayisal tutarlilik guvencesi

% Ws: roll/pitch yuksek oncelikli (iyi kosullanmis, guvenlik-kritik), yaw
% dusuk oncelikli tutulur — bu airframe'de yaw otoritesi (diferansiyel tilt)
% roll ile ayni aktuatoru paylastigindan (bkz. gain_schedule.m notu), esit
% agirlik WLS'i roll<->yaw arasinda sonumsuz bir cekismeye sokuyor; dusuk
% yaw onceligi bunu ortadan kaldirip yaw'in daha yavas ama sonik sekilde
% duzelmesini sagliyor.
%
% Fx onceligi (4. satir) DE dusuk tutulur: yaw'i duzelten diferansiyel tilt
% ayni zamanda onemli bir Fx (ileri govde-kuvveti) uretiyor; Fx=0 hedefi
% roll/pitch ile ayni/yakin mertebede agirliklandirilirsa WLS, "sapmayi
% durdur" ile "Fx'i sifirda tut" arasinda sonlanmayan bir cekismeye giriyor
% ve delta1 aktuator-rate sinirinda +/- salinip net ilerleme kaydetmiyor —
% deneysel olarak Ws_Fx ancak ~0.1'in altina indiginde bu cekisme tamamen
% ortadan kalkiyor ve psi tam olarak referansa (0) yakinsiyor (bkz.
% run_hover_gust_test.m'deki 40s dogrulama). Gercek bir ucuste Fx=0 zaten
% sadece basit bir vekil (pozisyon/hiz dis donguleri olmadigi icin) —
% burada dusuk oncelik vermek fiziksel olarak dogru: yaw duzeltmesi
% sirasinda kucuk bir ileri surukleme, kalici bir sapma hatasindan
% tercih edilir. Fz ise altitude_loop.m'nin integral etkisi sayesinde
% yuksek oncelikte kalabiliyor (irtifa kaybi olmuyor, bkz. ayni test).
% NOTE (2026-07-26): Ws_yaw=6 was tried and REVERTED -- see sitl/RUNBOOK.md
% "Adim 7" (helped nothing in SITL, Fx demand blew out to -28N, T0 still
% collapsed by the end of the test).
Ws = diag([200 200 3 0.05 20]);
% Yuzey cezalari qbar ile OLCEKLENMEZ; olceklenen sey |G|'dir. Sabit bir wu ile
% birim-etki cezasi Wu/|G| ~ 1/qbar oldugu icin devir noktasi kendiliginde bir
% HIZ olur (turetme ve pencereler: tiltrotor_params.m p.wls.wu_*).
wu_surf = [p.wls.wu_ail; p.wls.wu_ele; p.wls.wu_rud];
wu_surf = wu_surf(logical(p.wls.surf_enable));    % maske ile ayni siralama
Wu = diag([sched.wu_thrust; sched.wu_tilt; wu_surf(1:n_virt)]);
% du_pref: tilt kanallarinda 0 DEGIL, delta'yi p.tilt.bias'a dogru ceken kucuk
% bir artis (madde (P), 2026-07-29 Adim 28). Gerekce tiltrotor_params.m'de:
% du_pref = 0 iken denge delta1 = 0 tabaninda oturuyor ve yaw otoritesi
% yon-asimetrik oluyor; artimli tahsisatta bu ancak amac fonksiyonundan
% duzeltilebilir (Adim 19).
% du_pref = 0 (minimum efor). DENENDI VE GERI ALINDI (2026-07-29, Adim 28):
% madde (P) icin tilt kanallarina "bias'a cek" tercihi eklendi
% (du_pref(4:6) = (p.tilt.bias - delta)*tau/bias_tau). OLCULDU: delta1 dengesi
% bit duzeyinde degismedi (bias 0/5/10 deg ve bias_tau 3.0/1.0/0.5/0.2 -- kutunun
% 5 katina cikan pull dahil, hepsinde delta1 = 0.00 ve taban orani %100).
% Sebep alloc_probe ile bulundu: delta1'i tabanda tutan sey bir tercih zayifligi
% degil, KISITSIZ COZUMUN NEGATIF OLMASI (du_free(5) = -0.0089) -- yani arac
% delta1'i 0'in ALTINA indirmek istiyor. Gercek surucu nu_des(Fx) = -2.91 N idi;
% cozum p.ctrl.fx_trim'e tasindi (asagida). Etkisiz bir mekanizmayi kodda
% birakmamak icin bias kaldirildi.
du_pref = zeros(n_act,1);

[du, sat_flag, n_iter] = wls_allocate(G, nu_des, du_min, du_max, Ws, Wu, du_pref);

ctrl_state.prev_du_tilt = du(4:6);   % tiltjerk icin, bir sonraki tick'e tasinir

u_cmd = u_actual + du;
u_cmd(1:3) = max(min(u_cmd(1:3), Tmax), [Tmin_wing; Tmin_wing; Tmin]);
u_cmd(4:6) = max(min(u_cmd(4:6), dmax), dmin);

T_cmd     = u_cmd(1:3);
delta_cmd = u_cmd(4:6);

% Sanal aktuatorlerden FIZIKSEL servo komutlarina. Fiziksel kirpma da burada:
% sanal kutu zaten limitleri sagliyor, ama eslemenin tek dogruluk noktasi
% olmasi icin son kirpma yine de uygulanir.
if n_virt > 0
    a_cmd    = max(min(u_cmd(7:6+n_virt), a_max), -a_max);
    % SABIT TRIM OFSETI (Adim 52): sanal koordinat, ofsetin USTUNDEKI sapmadir.
    % Ofset tahsisata bilerek gorunmez -- iptal ettigi kuvvet (kuyruk asagi
    % yuku) de modelde yok, ikisi plant'te birbirini goturur. Gerekce
    % surf_trim_offset.m ve tiltrotor_params.m'de.
    surf_cmd = Mv * a_cmd + surf_trim_offset(p, qbar);
    surf_cmd = max(min(surf_cmd, p.surf.max(:)), -p.surf.max(:));
    diagn.a_cmd = a_cmd;
else
    surf_cmd = zeros(p.surf.n, 1);
    diagn.a_cmd = zeros(0,1);
end
diagn.surf_cmd = surf_cmd;
diagn.qbar     = qbar;

diagn.sched     = sched;
diagn.nu_des    = nu_des;
diagn.nu0       = nu0;
diagn.du        = du;
diagn.sat_flag  = sat_flag;
diagn.n_iter    = n_iter;
diagn.d_hat     = ctrl_state.d_hat;
diagn.omega_sp  = omega_sp;

end
