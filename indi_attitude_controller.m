function [T_cmd, delta_cmd, ctrl_state, diagn, surf_cmd] = indi_attitude_controller( ...
        att_sp, att, omega, omega_dot_meas, F_sp, u_actual, ctrl_state, p, leso_axes_enable, ...
        tilt_ceil, qbar, tilt_jerk_limit, agl)
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
%   agl (m)           ISTEGE BAGLI. YERDEN yukseklik. Yalnizca yere yakin kanat
%                     itki farki sinirini kapilar (asagida). Verilmezse veya
%                     esigin ustundeyse mekanizma HICBIR SEY YAPMAZ.
%                     SOZLESME (Adim 116): bu, zeminden yukseklik olmak
%                     ZORUNDA. Kestirimcinin yerel orijininden yukseklik
%                     (-z) DEGILDIR -- orijin kalkista bir kez kurulur ve o
%                     anin hatasini kalici bir ofset olarak tasir; 23 SITL
%                     kosumunda olculen ofset -0.67 .. +1.77 m, yani 2.0 m'lik
%                     esikle ayni mertebede. PX4 portu tam olarak bu hatayi
%                     yapiyordu ve kapi rastgele armaniyordu; C++ tarafi artik
%                     kalkis datumuna gore AGL hesapliyor
%                     (MulticopterIndiTiltrotor.cpp, captureGroundDatum()).
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

% --- TEMASTA MOMENT ARTIMI KESILIR (Adim 119) ---
% Adim 118 olctu ki tek tek CIKIS kanallarini kirpmak sarmayi tahsisatin icinde
% KOVALIYOR: kanat itki farki sifirlaninca ayni sarma tilt kanalina gocuyor
% (olculdu: T0=T1 tam esitken d0/d1 ~4.6 deg/s rampa, tilt farki 7.6 -> 38.1 deg,
% T2 sifir rayinda, yaw kacti). Kusur cikista degil, ARTIMIN KENDISINDE:
% yerde omega_dot_meas ~ 0 kaldigi icin dtau hic sonmuyor ve WLS onu hangi
% aktuator ucuzsa oraya biriktiriyor.
%
% NEDEN YALNIZCA MOMENT, KUVVET DEGIL: temasta arac AGIRLIK MERKEZI etrafinda
% degil TEMAS NOKTASI etrafinda doner, yani etkinlik matrisinin moment satirlari
% o rejimde GECERSIZDIR (bu gerekce zaten Adim 112'de yaziliydi). Kuvvet
% satirlari gecerli kalir; ayrica Fz'yi kesmek irtifa dongusunun yerde itkiyi
% AZALTMASINI da engellerdi -- Adim 110'un "arac temasta itkisini hic
% azaltmiyor" kok gozlemini kaliciLastirirdi.
%
% MANDAL: Adim 118'in olculmus temas mandali ("buyuk diferansiyel komut, acisal
% ivme yok"; 26 gorevde havada en fazla 0.01 s). Adim 112 "yerde artimi dondur"
% ailesini TEHLIKELI diye isaretlemisti, gerekcesi "asili ile oturmus ayirt
% edilemiyor" idi; o itiraz bu mandalla olcumle karsilandi.
if isfield(ctrl_state,'land_contact_latch') && ctrl_state.land_contact_latch
    nu_des(1:3) = 0;
end

Tmin = p.rotor.Tmin; Tmax = p.rotor.Tmax;
dmin = p.tilt.min;   dmax = p.tilt.max;
% KUYRUK ROTORU ICIN AYRI TAVAN (Adim 133). Fiziksel kisit: 0.10 m yaricapli
% disk 90 derecede kuyruk cubugunun ICINDEN geciyor (check_model_clearance.py).
% Alternatif -- motoru yukseltmek -- denendi ve GERI ALINDI, cunku r_z geri
% gecişte tau_y = r_z*Fx - r_x*Fz uzerinden isiriyor ve uc rotor da doydu.
% 20 derece olculen en buyuk kuyruk tilt'inin (2.5 deg) 8 kati.
% Geriye uyum: alan yoksa eski davranis (dmax) korunur.
if isfield(p.tilt, 'max_tail'); dmax_tail = p.tilt.max_tail; else; dmax_tail = dmax; end

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
abs_hi = [Tmax;Tmax;Tmax; dmax_wing;dmax_wing;dmax_tail; a_max] - u_actual;

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
u_cmd(4:5) = max(min(u_cmd(4:5), dmax), dmin);
u_cmd(6)   = max(min(u_cmd(6), dmax_tail), dmin);

% --- YERE YAKIN KANAT ITKI FARKI SINIRI (2026-08-29) ---
% Tam gerekce tiltrotor_params.m p.ctrl.land_diff_max notunda. Ozet: yerde
% omega_dot_meas = 0 kaldigi icin INDI artimi sonmez ve u_cmd tek yonde
% birikir; T1 sifir rayina cakilinca tek calisan egik rotor yaw kacisini
% baslatir. Burasi ORTALAMAYI KORUR (dikey kanal etkilenmez), yalnizca farki
% sinirlar. `agl` verilmezse veya esigin ustundeyse HICBIR SEY YAPMAZ -- yani
% ucus yollari bit duzeyinde degismez, mevcut cagiranlar da bozulmaz.
if nargin >= 13 && ~isempty(agl) && isfinite(agl) ...
        && isfield(p.ctrl, 'land_diff_enable') && p.ctrl.land_diff_enable ...
        && agl < p.ctrl.land_diff_alt
    T_mean = 0.5*(u_cmd(1) + u_cmd(2));
    T_diff = 0.5*(u_cmd(1) - u_cmd(2));
    d_lim  = 0.5*p.ctrl.land_diff_max;
    % TEMAS: sinir 0'a iner, yani kanat farki TAMAMEN silinir.
    % NEDEN: 10 N'lik sinir tek basina YETMEDI (SITL, ULog 11_04_57) -- fark
    % sinira DOYUP orada kaldi (ort +9.23 N, 3490 ornekte 5 isaret degisimi =
    % olu birikim) ve kalici yaw momenti araci dondurdu; egik rotorlarin yatay
    % bileseni de onu daire cizdirdi. Sonuc: 2.9 m surukleme, temasta 3.77 m/s
    % yanal hiz ve -134 deg/s yaw ile YERE VURARAK inis.
    %
    % NEDEN TEMASTA MODEL GECERSIZ: arac bir ayagi uzerine oturunca AGIRLIK
    % MERKEZI etrafinda degil TEMAS NOKTASI etrafinda doner. effectiveness_matrix
    % momentleri AM'ye gore kurar, yani o rejimde geri besleme duzeltmiyor,
    % BESLIYOR. Dogru davranis kazanci akillandirmak degil, momenti kesmektir.
    %
    % AYIRT EDICI ROLL ACISI: serbest hover'da tutum dongusu roll'u sifira yakin
    % tutar (olculen saglikli inis: |roll| < 1 deg); 8 deg ancak zemin araci
    % tutuyorsa gorulur (olculdu: -19.78 deg). Yer etkisinde ASILI arac da
    % duzduр, yani bu esik "asili" ile "oturmus"u da ayirir -- irtifa ve acisal
    % hizin ayiramadigi seyi.
    if isfield(p.ctrl,'land_contact_roll') && ...
            abs(att(1)) > p.ctrl.land_contact_roll
        d_lim = 0.0;
    end
    % IKINCI TEMAS OLCUTU (Adim 118): "buyuk diferansiyel komut, acisal ivme
    % YOK". Roll esigi duz inisi kaciriyordu (roll 0.2 derecede donuyor).
    % Gerekce ve olculen ayrim tiltrotor_params.m land_contact_diff notunda.
    % Mandal SART: fark sifirlaninca olcut kendi kendini bozar, mandalsiz
    % mekanizma acilip kapanarak salinir. Kapi kapaninca (arac 2 m'nin ustune
    % cikinca) mandal temizlenir -- yani ucusa donus otomatiktir.
    if isfield(ctrl_state,'land_contact_latch') && ctrl_state.land_contact_latch
        d_lim = 0.0;
    end
    T_diff = max(min(T_diff, d_lim), -d_lim);
    u_cmd(1) = T_mean + T_diff;
    u_cmd(2) = T_mean - T_diff;
    % Kutuya geri kirp: ortalama kutu icindeyse toplam da icindedir, ama
    % ortalama raya yakinken fark onu disari itebilir.
    u_cmd(1:2) = max(min(u_cmd(1:2), Tmax), Tmin_wing);

    % Bekleme sayaci, GONDERILEN farkla guncellenir (bir tick gecikmeli
    % degerlendirme: bu tick'in olcumu bir sonraki tick'in d_lim'ini kurar).
    if isfield(p.ctrl,'land_contact_dwell') && isfield(ctrl_state,'land_contact_dwell')
        hit = abs(u_cmd(1) - u_cmd(2)) > p.ctrl.land_contact_diff ...
              && abs(omega_dot_meas(1)) < p.ctrl.land_contact_acc;
        if hit
            ctrl_state.land_contact_dwell = ctrl_state.land_contact_dwell + p.Ts_ctrl;
        else
            ctrl_state.land_contact_dwell = 0;
        end
        if ctrl_state.land_contact_dwell >= p.ctrl.land_contact_dwell
            ctrl_state.land_contact_latch = true;
        end
    end

    % DIKEY ITKI TAVANI (Adim 145). Tahsisat momentleri kuvvetten 200 kat agir
    % tartiyor (ws_roll/ws_pitch = 200), yani YERDE olusan sahte bir moment
    % talebi icin kuvvet komutunu cignemekten cekinmez. Olculen ariza (SITL,
    % ULog 11_28_57): arac 0.29 m'de yere degdi, temas govdeyi hafifce
    % pitch'ledi, nu_des(2) +0.13 -> +1.22 buyudu, tahsisat kuyrugu bosaltip
    % (m2 -> 0) onu itti ve dikey itki toplami |Fz_sp|'nin 3.18 KATINA cikti --
    % arac 1.18 m'ye GERI KALKTI.
    % Yukaridaki kisma bunu GORMEZ: o yalniz kanat FARKINI (roll) kisar, ariza
    % ise on-vs-kuyruk (pitch) kanalinda.
    % NEDEN PITCH FARKI DOGRUDAN KISILMIYOR: olculdu, ISE YARAMAZ. Kapi altinda
    % pitch sapmasi SAGLIKLI inislerde de buyuk (bir kosuda ort 12.6 N) cunku
    % rotorlar kanadin AERODINAMIK pitch momentini de dengeliyor; boyle bir
    % kisit havada yanlis atesLer ve pitch otoritesini keser.
    % Bu olcut kontak tespitine HIC bagli degil. Esigin (2.0) ayirt ediciligi:
    % saglikli inislerde esigi asan ornek %0.0-1.1, arizali kosumda %16.7.
    % Uc rotor BIRLIKTE olceklenir -> moment ORANLARI korunur, kesilen net
    % kaldirmadir.
    if isfield(p.ctrl,'land_tz_max')
        ctz = u_cmd(1)*cos(u_cmd(4)) + u_cmd(2)*cos(u_cmd(5)) + u_cmd(3)*cos(u_cmd(6));
        tz_cap = p.ctrl.land_tz_max * abs(F_sp(2));
        if isfinite(ctz) && isfinite(tz_cap) && ctz > tz_cap && ctz > 1e-3
            u_cmd(1:3) = max(min(u_cmd(1:3) * (tz_cap/ctz), Tmax), Tmin);
        end
    end
elseif isfield(ctrl_state,'land_contact_latch')
    % Kapi kapali (ucus, veya agl yok): mandal ve sayac temizlenir.
    ctrl_state.land_contact_latch = false;
    ctrl_state.land_contact_dwell = 0;
end

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
