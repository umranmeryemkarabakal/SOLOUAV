function p = tiltrotor_params()
%TILTROTOR_PARAMS  3 tilt-rotorlu VTOL (2 kanat + 1 kuyruk) arac parametreleri.
%
% Kaynak: ~/Downloads/gz_tiltrotor_tailplane
%   Tools/simulation/gz/models/tiltrotor_tailplane/model.sdf
%   ROMFS/px4fmu_common/init.d-posix/airframes/4022_gz_tiltrotor_tailplane
%
% Govde ekseni: FRD (X ileri, Y sag, Z asagi). Tum aci/pozisyonlar bu cercevede.

%% --- Kutle ve atalet (SDF base_link) ---
p.m  = 5.0;                       % kg, toplam kutle
p.g  = 9.81;                      % m/s^2
p.I  = diag([0.2, 0.25, 0.25]);   % kg.m^2  [Ixx Iyy Izz], SDF base_link inertia
p.Iinv = inv(p.I);

%% --- Rotor geometrisi (FRD, m) — CA_ROTORi_P{X,Y,Z} airframe parametrelerinden ---
% Rotor 0: sag kanat (tilt eden)
% Rotor 1: sol kanat (tilt eden)
% Rotor 2: kuyruk (tilt eden, cruise'da pusher)
% NOT (2026-07-26): model.sdf'de "Right wing rotor"=motor_0@Y=-0.25,
% "Left wing rotor"=motor_1@Y=+0.35 yaziyor -- burasindaki [+0.35,-0.35,0]
% ile ters gorunuyor. DENENDI: Y isaretleri buraya gore duzeltildiginde
% (rpos=[-0.25,0.25,0], km ayni [+0.06,-0.06,+0.06] birakilarak) saf
% MATLAB referans testinde (run_hover_gust_test) RMS p/q ~0.005'ten
% 0.34/1.29 rad/s'e (kabaca 100-1000x) firladi -- yani bu "duzeltme"
% plant+kontrolcu KENDI ICINDE tutarliyken bile sistemi ciddi bicimde
% kararsizlastirdi. GERI ALINDI. Sonuc: SDF'deki Y isareti gozlemi gercek
% ama basit bir isaret-degistirme dogru duzeltme DEGIL -- ya km'nin
% gercek spin-yonu eslemesi (turningDirection ccw/cw -> +/-1) varsayilani
% dogrulanmadan alindi, ya da baska bir eksen/cerceve varsayimi hatali.
% Bkz. sitl/WLS_LOCKUP_INVESTIGATION_REPORT.md ve RUNBOOK.md "Aday cozum 4"
% -- bu deger DEGISTIRILMEDEN once konu ayrica, dikkatlice arastirilmali.
% UPDATED (2026-08-16, arkadasin dogrulanmis duzeltmesi): kanat rotorleri
% kanadin ALTINDAN USTUNE tasindi (model.sdf, 2026-08-02, x8_wing.dae'den
% dikey ray-cast ile olculmus gercek pervane-motor bosluguyla). SDF Z=+0.11
% (FLU) -> FRD Z=-0.11 (isaret ters cevrilir). sf_wls_alloc.m ile senkron.
% *** KUYRUK ROTORUNU YUKSELTME DENENDI VE GERI ALINDI (2026-08-30, Adim 133) ***
% Adim 132 kuyruk rotorunu z -0.07 -> -0.16'ya tasidi (SDF +0.07 -> +0.16).
% Gerekce geometrikti ve dogruydu: 0.10 m yaricapli disk 90 derecede kuyruk
% cubugunun ICINDEN geciyor. Gerekcelendirme ise EKSIKTI -- "X degismedi, yani
% hover pitch kolu korunur" denmisti, ki dogru; ama GERI GECIS HOVER DEGILDIR.
% Tilt'li rejimde tau_y = r_z*Fx - r_x*Fz, yani r_z tam orada isiriyor.
% OLCULEN BEDEL (SITL, tam gorev): geri gecişte (t=93.6 s, irtifa 37.8 m) uc
% rotor da 45 N'e doydu; BIG_M 0 -> 3843. Kullanici GUI'de salinim gordu.
% YERINE: cubuk inceltildi + kuyruk tilt araligi 90 -> 20 dereceye sinirlandi
% (bkz. p.tilt.max_tail). Boylece r_z HIC degismiyor.
p.rotor.pos = [ 0.27    0.27  -0.55;   % X
                0.35   -0.35   0.00;   % Y
               -0.11   -0.11  -0.07];  % Z

% Motor tork/itki orani (km, Nm/N), isaret donus yonunu tasir.
% NOT (2026-07-26): CA_ROTORi_KM airframe parametresi (4022_gz_tiltrotor_tailplane)
% 0.05 olarak set edilmis, ama gercek SDF motor model plugin'inin
% <momentConstant> degeri 0.06 (model.sdf, gz-sim-multicopter-motor-model-system,
% turningDirection: rotor0=ccw, rotor1=cw, rotor2=ccw — isaret deseni asagidakiyle
% ayni). gz-sim kaynagi: dragTorque_z = -turningDirection*thrust*momentConstant,
% yani momentConstant tam olarak bu km ile ayni birimde (Nm/N) ve dogrudan
% karsilastirilabilir — CA_ROTORi_KM'nin SDF'den yanlis transkribe edildigi
% (0.05 vs 0.06, ~%20 fark) SITL'de PX4 tarafinda dogrulandi (bkz.
% sitl/RUNBOOK.md "Aday cozum 3"). Burada (saf MATLAB) plant ve kontrolcu
% zaten AYNI p.rotor.km'yi paylastigindan bu deger onceden kendi icinde
% tutarliydi (uyumsuzluk yalnizca PX4 C++ portunun ROTOR_KM'i ile gercek
% Gazebo fizigi arasindaydi) — burasi gercek SDF airframe'ine sadakat icin
% guncellendi.
%
% ISARET DUZELTMESI (2026-07-27, Adim 12) — [+0.06,-0.06,+0.06] idi:
% Yukaridaki not gz kaynagini dogru aktariyor
% (dragTorque_z = -turningDirection*thrust*momentConstant) ama yalnizca
% isaret DESENINI (+,-,+) esledi; desenin FRD cerçevesindeki TOPLAM
% isaretini hic karsilastirmadi. Zincir: gz'de dragTorque rotor linkinin
% yerel cercevesinde (rpy=0, yani base_link ile hizali; gz FLU: x-on,
% y-SOL, z-YUKARI) uygulanir. Rotor 0 (ccw, turningDirection=+1, T>0):
% dragTorque_z(FLU) = -km*T < 0. FLU->FRD'de z isaret degistirir:
% tau_z(FRD) = +km*T > 0 (burun saga). Kontrolcu/plant modeli ise
% m_i = km_i*T_i*dir_i, dir=(0,0,-1) => m_z = -km*T < 0 (burun sola).
% Yani model, uc rotorun da yaw reaksiyon torkunu TERS isaretle
% tasiyordu. SITL'de nicel olarak dogrulandi: arm aninda olculen tepe
% yaw ivmesi iki bagimsiz kosuda +6.45 / +6.56 rad/s^2; duzeltilmis
% modelin acik-cevrim ongorusu +6.2 rad/s^2, eski modelin ongorusu ~0.
% Adim 11'in itki eslemesi hatasiyla AYNI SINIFTAN: plant ve kontrolcu
% ayni p.rotor.km'yi paylastigi icin saf MATLAB bu hatayi yapisal olarak
% goremez (ikisi birlikte yanlis -> testler kendi icinde tutarli kalir).
% Bkz. sitl/WLS_LOCKUP_INVESTIGATION_REPORT.md Adim 12.
p.rotor.km  = [ -0.06, 0.06, -0.06 ];

% Motor itki katsayisi ve limitler (SDF MulticopterMotorModel)
p.rotor.kf     = 2.0e-5;                 % N/(rad/s)^2  (motorConstant)
p.rotor.w_max  = 1500;                   % rad/s        (maxRotVelocity)
p.rotor.Tmax   = p.rotor.kf * p.rotor.w_max^2;   % ~45 N / rotor
p.rotor.Tmin   = 0.0;                    % N
% Tmin_cruise -- SEYIRDE KANAT ROTORLARINA ITKI TABANI (Adim 50, 2026-08-04).
% Bu bir ayar degil, IKI BAGIMSIZ OLCUMUN isaret ettigi yapisal bir kisittir:
%   (a) Adim 49, pitch hedefi 0 iken: arac TAM T0 = 0.00 N'de iraksadi
%       (u 15.9 -> 27.6 m/s, 2 s icinde).
%   (b) Adim 49, hedef -12 N iken: T0 ~1.4 N dibinde salinip iraksadi.
% Sebep etkinlik matrisinde yaziyor: tilt sutunu
%       dtau/ddelta = (r x ddir + km*ddir) * T_i
% yani ITKIYLE CARPILIDIR. Itkisi sifir olan bir tilt rotorunun kontrol
% otoritesi de TAM SIFIRDIR -- tahsisat o aktuatoru tamamen kaybeder. Yuzeyler
% roll/pitch'i devralabiliyor ama yaw'i yalnizca zayif rudder tasiyor ve Fx'in
% hicbir yuzeyi yok.
%
% Dogru arac bir amac terimi DEGIL bir KUTU KISITIDIR (Adim 31'in tilt
% tavaniyla ayni mekanizma ve ayni gerekce: tahsisatin tercihi manevranin
% tersine bakiyorsa onu ancak bir kutu degistirebilir).
%
% *** DENENDI (4.0 N) VE GERI ALINDI -- Adim 50, 2026-08-04. ***
% Teshis (T = 0 olan tilt rotorunun otoritesi yok) DOGRU; ama bu ARAC yanlis
% cikti ve daha once KARARLI olan bir noktayi (v_sp = 16, pitch kapali) bozdu:
% t = 26.9 s'de iraksama. Zaman serisi sebebi acikca gosterdi:
%
%   t=25:  T0=3.39 T1=2.76 T2=14.19  d0=87.5 deg  Fz_sp=-16.31
%   t=26:  T0=2.99 T1=2.99 T2= 0.01  d0=90.0 deg  Fz_sp= -5.20
%   t=27:  T0=3.08 T1=3.08 T2= 0.00  d0=90.0 deg  Fz_sp=+96.42  -> iraksama
%
% Kanat rotorleri tam tabanda cakili, ama OLEN aktuator KUYRUK rotoru. Zincir:
% taban, kanat rotorlerini 90 DERECE TILTTE itki uretmeye zorluyor; orada itki
% hicbir TASIMA uretmez, yalnizca ileri kuvvet. Arac hizlaniyor -> kanat fazla
% tasiyor -> irtifa dongusu tek dikey aktuatoru (T2) sifira kesiyor -> dikey
% kanal kayboluyor -> Fz_sp +96 N'e patliyor (hicbir aktuatorun uretemeyecegi
% bir asagi kuvvet talebi).
%
% GENEL DERS: bir aktuatoru "hayatta tutmak" icin konan taban, o aktuatorun
% o anki YONELIMINI hesaba katmazsa yalnizca istenmeyen bir kuvvet enjekte
% eder. 90 derecede bir tilt rotoru dikey kanal icin zaten OLUDUR; tabanin
% korudugu sey otorite degil, sadece ileri itki oldu.
%
% 0.0 = ozellik kapali, davranis Adim 49'daki gibi. Kod bilerek birakildi ki
% ayni fikir bir daha korlemesine denenmesin (SURF_ENABLE ile ayni disiplin).
p.rotor.Tmin_cruise = 0.0;               % N -- DENENDI, GERI ALINDI
p.rotor.tau_up   = 0.0125;               % s, gaz artirma zaman sabiti
p.rotor.tau_down = 0.0250;               % s, gaz azaltma zaman sabiti

%% --- Tilt servo (CA_SV_TL*_MINA/MAXA = 0..90 deg) ---
p.tilt.min      = 0;                     % rad, dikey (hover)
p.tilt.max       = pi/2;
% KUYRUK ROTORU ICIN AYRI TILT TAVANI (2026-08-30, Adim 133).
% FIZIKSEL KISIT, bir ayar degil: 0.10 m yaricapli disk 90 derecede alt ucu
% z = motor_z - 0.10 = -0.055'e iner ve kuyruk cubugunun ICINDEN gecer
% (olculdu: check_model_clearance.py). Alternatif motoru yukseltmekti; denendi
% ve GERI ALINDI (yukaridaki p.rotor.pos notu -- geri gecişte BIG_M 0 -> 3843).
% 20 DERECE OLCUME DAYANIR: tam gorevde kuyruk tilt'i en fazla 2.5 derece
% kullanildi (ULog 2026-08-30/05_47_06), yani sinir gercek kullanimin 8 kati.
% O acida cubuga aciklik 24 mm. model.sdf motor_2_joint <upper> ile SENKRON.
p.tilt.max_tail = deg2rad(20.0);                 % rad, yatay (cruise)
p.tilt.tau       = 0.15;                 % s, servo zaman sabiti (1. derece yaklasim)
p.tilt.rate_max  = 3.0;                  % rad/s, FIZIKSEL servo slew limiti (plant clamp)

% AYRISTIRILDI (2026-07-29, Adim 27) — PX4'teki Adim 22 ayriminin MATLAB'a
% tasinmasi. Onceden `p.tilt.rate_max` iki AYRI seyi birden ifade ediyordu:
%   (1) plant'in fiziksel servo clamp'i   -> tiltrotor_plant_deriv.m:42
%   (2) WLS tahsisatinin tek tick'te isteyebilecegi artis (kutu)
%       -> indi_attitude_controller.m, sf_wls_alloc.m
% Bunlar ayni nicelik degil; birlestikleri surece ikisi de anlamli sekilde
% ayarlanamiyordu (PX4'te Adim 14 tam bu yuzden yanlis sonuca goturdu:
% ikisi birden oynatildi).
%
% DEGERIN SECIMI: PX4 ile eslesen sey NOMINAL hiz degil, TICK BASINA kutudur --
% cunku `u_cmd = u_actual + du` oldugundan golge/plant her tick'te du'nun ancak
% dt/tau kadarini ilerletir, yani etkin slew = (kutu_tick_basina)/p.tilt.tau
% (Adim 26). PX4: 3.00*(1/250) = 0.0120 rad/tick @ 250 Hz.
% MATLAB 400 Hz'de ayni tick-basi kutu icin: 0.0120/0.0025 = 4.8 rad/s.
% Nominal sayilarin farkli olmasi (4.8 vs 3.00) dogrudur ve yalnizca dongu
% hizi farkini (400 vs 250 Hz) yansitir; etkin slew ikisinde de 0.080 rad/s.
%
% OLCULDU (run_box_bind_check.m, 2026-07-29): bu kutu MATLAB'da ADIM SONRASI
% TICK'LERIN %25-28'inde BAGLIYOR -- yani aktif bir kisit. (Rapor Adim 21d'nin
% "MATLAB'da tilt kutusu hic baglamaz" ifadesi bu olcumle CURUDU.) 4.8'e
% cikarinca baglama %1.6-3.6'ya dusuyor ve +30 deg asim %24.0 -> %14.5,
% oturma 3.68 -> 3.01 s iyilesiyor; madde (P) yon asimetrisi de 3.1x -> 1.3x
% daraliyor. Plant clamp'i 3.0 -> 2.0 yapmak ise HICBIR sey degistirmiyor
% (fiziksel clamp MATLAB'da hic baglamiyor) -- bu kisim Adim 21d/26a'da dogruydu.
p.tilt.slew_box_rate = 4.8;              % rad/s, YALNIZCA WLS tahsisat kutusu

% --- DENENDI VE GERI ALINDI: tilt bias tercihi (2026-07-29, Adim 28) ---
% Asagidaki gerekce DOGRU teshis degildi. delta1'i tabanda tutan sey bir
% "tercih zayifligi" degil, tahsisatin KISITSIZ cozumunun negatif olmasiydi
% (du_free(5) = -0.0089, yani arac delta1'i 0'in ALTINA indirmek istiyor);
% asil surucu nu_des(Fx) = -2.91 N. Bias 0/5/10 deg ve bias_tau 3.0/1.0/0.5/0.2
% (kutunun 5 katina cikan pull dahil) denendi: delta1 HER durumda 0.00 ve taban
% orani %100 -- yani mekanizma tamamen etkisiz. Cozum p.ctrl.fx_trim'e tasindi.
% Bu blok, ayni hipotezin tekrar denenmemesi icin kayit olarak birakildi;
% p.tilt.bias ARTIK KULLANILMIYOR (indi_attitude_controller.m du_pref = 0).
%
% (eski gerekce, tarihsel):
% Rapor §4 (P): hover trim'inde delta1 == 0, yani tek yonlu tilt araliginin
% (p.tilt.min = 0) TABANINDA oturuyor. tau_z = -0.25*T0*sin(d0) + 0.25*T1*sin(d1)
% oldugundan -yaw yalnizca d0'i yukseltmeyi ister (serbest), +yaw ise d1'i
% tabandan kaldirmayi ister; d1 kalkinca Fx->0 terimi onu hemen geri
% cakiyor (Adim 18: salinim sirasinda 12 kalkis-carpma olayi). Sonuc: yon
% asimetrisi.
%
% Adim 19'un dersi: ARTIMLI bir tahsisatta kalici bir aktuator konfigurasyonu
% BASLANGIC KOSULUYLA dayatilamaz -- amac fonksiyonu ya da kisit degismeli.
% (Trim'i d_bias ile tohumlamak denendi, hepsi ayni dengeye yakinsadi.)
% Kisiti degistirmek (p.tilt.min > 0) ise ise yaramaz: delta1 bu sefer yeni
% tabana oturur, asimetri sadece yon degistirir.
%
% Bu yuzden AMAC FONKSIYONU degistiriliyor: WLS'in du_pref'i tilt kanallarinda
% artik 0 degil, delta'yi bias'a dogru ceken kucuk bir artis. Denge boylece
% fizibil bolgenin ICINDE kaliyor ve yaw otoritesi iki yonde de simetrik olur.
%
% BEDELI ve NEDEN ANCAK SIMDI YAPILABILIYOR: ortalama tilt yukselince kalici
% bir +Fx doguyor (~3 N). Yatay pozisyon dongusu (madde (N), position_loop.m)
% OLMADAN bu dogrudan surukleneme demekti -- yani (P)'nin bu cozumu (N)'in
% cozulmus olmasina BAGLI. Sirali bagimlilik: once (N), sonra (P).
p.tilt.bias     = [0.0873; 0.0873; 0];   % rad (5 deg kanat rotorleri; kuyruk 0
                                         % cunku PY=0, yaw'a katkisi yok, yalnizca
                                         % +Fx ve cos-itki kaybi eklerdi)
p.tilt.bias_tau = 3.0;                   % s, bias'a cekme zaman sabiti (yumusak:
                                         % kontrol eylemiyle yarismasin)

% Kuyruk rotoru (rotor 2) tilt'i hover torkuna dahil edilmez (CA_SV_TL2_CT=0):
% merkez hatta oldugu icin diferansiyel tilt yaw uretmez. WLS agirliklandirmasinda
% bu rotorun tilt'i roll/yaw icin "tercih edilmeyen" (yuksek Wu) tutulur.
p.rotor.tilt_yaw_effective = [true, true, false];

%% --- Kanat/kuyruk aerodinamigi: BES PANEL (SDF LiftDrag eklentileri) ---
% Adim 46 (2026-08-03), madde (V). Kontrolcu bu modeli BILMEZ; plant'ta bir
% "bozucu" olarak uygulanir ve LESO'nun telafi etmesi beklenen model
% belirsizligini temsil eder. Yasa gz-sim LiftDrag'in birebir portudur, bkz.
% aero_panels.m.
%
% ESKI MODELIN IKI KUSURU (ikisi de Adim 46'da OLCULDU, varsayilmadi):
%   1) TEK 0.5 m^2 panel, y = 0'da, kontrol yuzeyi YOK. Gercekte kanat IKI
%      yarim panel (her biri 0.5 m^2, cp y = +-0.30) ve bes panelin BESI DE
%      birer kontrol eklemi tasiyor. Yuzeysiz plant'te bir yuzey sapmasi tam
%      sifir etki uretir; yani madde (V) MATLAB'da yapisal olarak sinanamazdi.
%   2) TASIMA ISARETI TERSTI: 15 m/s'de burun yukari 43-67 N ASAGI kuvvet.
%      alpha_eski = -alpha_std ve Cl = cla*(alpha - a0) birlikte, dogru
%      degerin tam negatifini veriyordu. Ayrintili turetme aero_panels.m'de.
% Ayrica gz'de cd SABIT DEGIL: cd = |cda*alpha|. Seyir hucum acisinda (~0.15
% rad) bu 0.096'dir; eski model 0.6417 sabitiyle ~7 kat fazla surukluyordu.
%
% Panel sirasi = servo sirasi (her panelin TAM BIR kontrol eklemi var):
%   1 sol elevon / sol kanat yarisi   (servo_0, left_elevon_joint)
%   2 sag elevon / sag kanat yarisi   (servo_1, right_elevon_joint)
%   3 sol elevator / sol yatay kuyruk (servo_2, left_elevator_joint)
%   4 sag elevator / sag yatay kuyruk (servo_3, right_elevator_joint)
%   5 rudder / dikey kuyruk           (servo_4, rudder_joint)
% cp ve up vektorleri model.sdf'ten alinip FLU -> FRD (x, -y, -z) cevrimi
% UYGULANARAK yazildi -- ayni cevrimin atlanmasi madde B4'te aylarca suren bir
% "isaret celiskisi" yanilsamasi uretmisti (Adim 34).
p.aero.n     = 5;
p.aero.rho   = 1.2041;              % kg/m^3  (SDF: air_density)
p.aero.cla   = 4.752798721;         % 1/rad
p.aero.cda   = 0.6417112299;
p.aero.alpha_stall = 0.3391428111;  % rad
p.aero.cla_stall   = -3.85;
p.aero.cda_stall   = -0.9233984055;
% a0: kanat panellerinde oturma acisi, kuyrukta -0.2 (burun-agir bir ucagi
% trimlemek icin gereken kuyruk asagi-yuku), dikey kuyrukta 0.
p.aero.a0    = [0.05984281113, 0.05984281113, -0.2, -0.2, 0.0];
p.aero.area  = [0.5, 0.5, 0.048, 0.048, 0.032];         % m^2
p.aero.rad_to_cl = [-4.0, -4.0, -12.0, -12.0, -6.0];    % 1/rad, kontrol eklemi
% cp, FRD (SDF FLU degerlerinin (x,-y,-z) cevrimi):
%   FLU (-0.05, +0.30, +0.05) (-0.05,-0.30,+0.05) (-0.70,+0.15,-0.04)
%       (-0.70,-0.15,-0.04)   (-0.74, 0, +0.12)
% RUDDER/FIN GERIYE ALINDI (2026-08-30, Adim 132): -0.74 -> -0.86. Sebep
% geometrik: disk tilt=0'da (hover boyunca) fininin icinden geciyordu. Yan
% etkisi kontrol lehine -- yaw kolu %16 uzuyor.
p.aero.cp    = [ -0.05  -0.05  -0.70  -0.70  -0.78;
                 -0.30  +0.30  -0.15  +0.15   0.00;
                 -0.05  -0.05  +0.04  +0.04  -0.10];
% up ("upward"), FRD: kanat/kuyruk FLU (0,0,1) -> (0,0,-1); rudder FLU (0,1,0)
% -> (0,-1,0), yani "tasimasi" bir YAN kuvvettir ve x = -0.74'te yaw uretir.
p.aero.up    = [  0      0      0      0      0;
                  0      0      0      0     -1;
                 -1     -1     -1     -1      0];

%% --- Kontrol yuzeyleri (madde V, Adim 44) -- bkz. effectiveness_matrix.m ---
% TEK KAYNAK KURALI (Adim 46): asagidaki uc alan p.aero panellerinden TURETILIR,
% elle yazilmaz. Eskiden ayni sayilar iki yere kopyalanmisti; kontrolcunun
% modeli ile plant'in fizigi arasinda sessiz bir sapma o sekilde mumkun olur ve
% bu projede tam olarak o sinif hata (Adim 11/21/27, kontrolcu-plant arayuzu)
% en pahali hatalardir. Simdi sapma yapisal olarak IMKANSIZ.
p.surf.n   = p.aero.n;
p.surf.cp  = p.aero.cp;
p.surf.dir = p.aero.up;                         % kucuk-alpha'da tasima yonu
% k = alan * rad_to_cl  (N per Pa per rad). Isaret dahildir.
p.surf.k   = p.aero.area .* p.aero.rad_to_cl;

%% --- Kontrol dongusu zamanlamasi ---
p.Ts_ctrl = 1/400;    % s, INDI/rate loop (400 Hz, Cube Black STM32F427 icin makul)
p.Ts_leso = 1/200;    % s, LESO guncelleme hizi (200 Hz — sohbette onerilen 100-250 Hz araligi)
p.Ts_att  = 1/200;    % s, disaridaki attitude (P) dongusu
p.Ts_pos  = 1/50;     % s, irtifa/pozisyon dis dongusu (50 Hz — PX4'teki tipik pozisyon
                      % dongusu hizi; rate loop'un 1/8'i, ek CPU yuku ihmal edilebilir)

%% --- Dis attitude dongusu hiz setpoint doygunu (eksen bazli) ---
% (2026-07-27, Adim 13) Eskiden tek skalerdi (3.0, tum eksenler icin).
% Roll/pitch bunu rahat takip ediyor, ama YAW EDEMEZ: yaw torku yalnizca
% diferansiyel kanat tilt'inden gelir ve tilt slew limitiyle sinirlidir.
% SITL'de olculdu (Adim 12g): arac donerken yaw hatasi +-180'de sarmalandigi
% icin 3.0'lik limit, hiz setpoint'inin surekli isaret degistirmesine yol
% aciyor ve dis dongu ic dongunun donusu sonumlemesini engelliyor. Limit
% gercek donus hizlarinin (1-3.5 rad/s olculdu) ALTINDA tutulursa hata
% isaretini olculen hiz belirler ve rate dongusu HER ZAMAN sonumleme yonunde
% calisir. Yaw icin 0.5 rad/s (~29 deg/s): 90 deg'lik bir donus ~3 s surer,
% bu arac icin fazlasiyla yeterli.
p.ctrl.rate_sp_limit = [3.0; 3.0; 0.5];   % rad/s, [roll; pitch; yaw]

% --- Fx trim: madde (P)'nin cozumu (2026-07-29, Adim 28) ---
% Tek yonlu tilt araligi ([0, pi/2]) yuzunden hover'da Fx >= 0'dir ve yaw
% trim'inin diferansiyel tilt'i ~2.9 N kalici ileri kuvvet uretir. Tahsisattan
% Fx = 0 istemek, aracin YAPISAL OLARAK iptal edemeyecegi bir kuvveti iptal
% etmesini istemek demek: kisitsiz cozum ucu tilt icin de negatif cikiyor ve
% delta1/delta2 TILT_MIN = 0 tabanina kalici olarak cakiliyor -- madde (P)'nin
% yon asimetrisi tam olarak budur (-yaw serbest, +yaw sinira dayali).
% Bu terim o ulasilamaz talebi kaldirir. OLCULDU (+-30 deg yaw adimi):
%   0   -> asim %14.8/%10.9, asimetri 1.35x, delta1 tabanda %100
%   2.9 -> asim %13.7/%13.0, asimetri 1.05x, delta1 tabanda %0
%   4.0 -> asimetri 1.03x ama Fx 3.84 N (marjinal kazanc, artan bedel)
% 2.9 = dogal denge degeri (olculdu), uydurma bir sayi degil.
% Bedeli kalici +Fx oldugundan yatay pozisyon dongusunu GEREKTIRIR (madde (N)).
p.ctrl.fx_trim = 2.9;   % N, gecis boyunca sched.smooth ile sonduruluyor

%% --- Geri gecis (back-transition), madde B5 -- bkz. backtrans_loop.m ---
% Adim 31 (2026-07-29). Her dort deger de SITL'de OLCULDU; MATLAB bu manevrayi
% yeniden uretemedigi icin (tek boylamsal yuzey, 12 m/s'de ~25 N tasima)
% buradaki degerler PX4'un TiltrotorIndiParams.hpp BT_* sabitleriyle
% SENKRON KALMALIDIR ve dogrulamalari orada yapilir.
%
% retract_rate: tavanin izlenebilecegi ust sinir 4.58 deg/s'dir
%   (TILT_SLEW_BOX_RATE*TS_BOX/TILT_TAU = 3.0*(1/250)/0.15 = 0.080 rad/s);
%   2.0 bunun rahat altinda ve uc ucusta 15 -> ~8 m/s'yi irtifayi tutarak
%   (bant <= 0.86 m) ve itki doyumu %0.0 ile getirdi.
% ceil_floor: 9 deg. Daha asagisi yaw'i ac birakiyor -- taban 9/7/5 deg
%   supurmesinde yaw sapmasi +0.0205/+0.0308/+0.0384 rad/s (supurme sirasi
%   ters cevrilen ikinci ucusta dogrulandi), ve 5 deg'de arac kendiliginden
%   kacti. Sebep yapisal: delta1 p.tilt.min'de cakili oldugu icin diferansiyel
%   TAM OLARAK tavana esittir, ve delta1 = 0 iken tau_z = -0.25*Fx -- yaw trim
%   torku ile kalan ileri kuvvet AYNI buyukluktur.
% release_v / brake_v_full / handoff_v: birakma esiginin altinda gövde pitch'i
%   gercekten frenliyor (olculdu: 5.74 -> 3.01 -> 0.10 m/s, pitch 0/+2/+4 deg,
%   vz +-0.23 icinde). handoff_v, PX4'teki POS_ENGAGE_V_MAX ile ayni olmalidir.
p.bt.retract_rate = deg2rad(2.0);  % rad/s, tavanin inme hizi
p.bt.ceil_floor   = deg2rad(9.0);  % rad, tavanin tabani (hover trim civari)
% release_v: 8.0 IDI, 2026-07-31 Adim 38'de 10.0'a CIKARILDI -- madde (R).
% Eski deger bir DENGENIN SINIRINA konmustu. Taban tavanda (9 deg) pitch = 0
% ile arac bir terminal hiza asimptotik yaklasir; adim 37 bunu iki ucusta
% olctu -- 8.0-9.3 m/s, yani release_v = 8.0'in TA KENDISI ya da USTU. Sonuc:
% cikis kosulu hic saglanmayabiliyor (bir ucusta RETRACT 200.3 s boyunca
% cikmadi ve arac 8.97 m/s'nin altina inmedi). Tamamlanan ucuslar esigi
% ILK YAVASLAMA GECICISINDE gecmisti, oturmus durumdan degil: 8 ucusun 5'i.
% Asagidaki yorumun eski "terminal hiz 5.7-7.9 m/s" kaydi da bu yuzden
% DUZELTILDI -- o adim 31'in ucuslarindan gelen eksik bir orneklemeydi.
% 10.0, olculen aralikligin en kotusune (9.3) 0.7 m/s pay birakir.
p.bt.release_v    = 10.0;          % m/s, tavanin YUKSELTILDIGI hiz
% floor_dwell: AERO-BAGIMSIZ EMNIYET, Adim 38'de eklendi. release_v'yi
% yukseltmek madde (R)'yi SITL'de kapatir ama DONANIMDA kapatmaz: 10.0 sayisi
% Gazebo'nun bes lift-drag yuzeyinden olculmus bir terminal hiza gore secildi
% ve gercek kanat farkli tasirsa esik yine dengenin icine dusebilir (B2/B3 ile
% ayni sinif risk). Bu yuzden cikis ikinci, hiza HIC bakmayan bir terime de
% baglandi: tavan tabana vardiktan sonra bu sure gecerse BRAKE'e yine gecilir.
% NEDEN IPTAL DEGIL DE BRAKE: bekleme tam olarak RETRACT'in verebilecegi EN
% DUSUK hizda dolar (hiz asimptota oturmustur, beklemek artik bir sey
% kazandirmaz), ve BRAKE tavani 20 deg'e YUKSELTEREK yaw'in modulasyon
% otoritesini geri verir -- yani takilmanin ASIL zararini (200 s'de +117.7 deg
% yaw suruklenmesi) dogrudan kaldirir. IDLE'a iptal yaw'i yine kurtarirdi ama
% araci seyirde, eve donus yolu olmadan birakirdi.
% 20 s secimi olcumden: taban tavana varildiktan sonra hiz 16.7 s'de 9.50 ->
% 8.62, 33.4 s'de 8.29, 89.1 s'de 8.00 -- yani 20 s'de arac terminal hizin
% ~0.4 m/s yakinindadir, daha uzun beklemek kazanc getirmez. Maruz kalinan
% yaw suruklenmesi de sinirlanir: olculen ~0.59 deg/s x 20 s ~ 12 deg,
% olcutun (<= 45 deg) rahat altinda.
p.bt.floor_dwell  = 20.0;          % s, tabanda beklenip yine de BRAKE'e gecis
% brake_ceil: BRAKE'te tavan KALDIRILMAZ, YUKSELTILIR. Ilk otomatik ucusta
% (2026-07-29) 8.4 m/s'de tamamen birakildi ve manevra KENDINI BOZDU: Fz
% kacisi yeniden basladi, tilt 24 -> 90 deg gitti, hiz 6.3 -> 12 m/s'ye cikti.
% Elle kosular tam birakmayi ancak DAHA GEC, oturmus 5.7-6.9 m/s'de yaptiklari
% icin atlatmisti (kanat tasimasi ~1.5x kucuk). Duzeltme yalnizca esigi
% dusurmek degil -- tabandaki terminal hiz esigi kilitleyebilir (bu satirin
% eski hali terminal hizi "5.7-7.9 m/s" diye kaydediyordu; adim 37 onu
% 8.0-9.3 m/s olctu ve madde (R) tam olarak buradan cikti -- bkz. release_v).
% Gercek teshis: yaw'i ac birakan sey tavanin
% VARLIGI degil, tavanin TRIM DEGERINDE oturup delta0'a hic yer birakmamasiydi.
% Trim'in belirgin ustunde bir tavan yaw modulasyonunu tamamen geri verir
% (delta1 p.tilt.min'de kaldigi icin diferansiyel = delta0, [0, ceil] arasinda
% serbest) ve kacisi yine de sinirlar. 20 deg ucus verisinin destekledigi deger:
% basarili elle kosuda tilt birakma sonrasi gecici 18.8 deg'e cikip ~10'a dondu.
p.bt.brake_ceil   = deg2rad(20.0); % rad, BRAKE'teki tavan (kaldirma degil)
p.bt.pitch_max    = deg2rad(4.0);  % rad, frenleme burun yukari tavani
% handoff_v: 2026-08-03 Adim 39, madde (S) -- DEGER degil, BAKILAN SINYAL
% degisti. Esik artik ISARETLI govde ileri hizina uygulaniyor (v_fwd), yatay
% hiz buyuklugune degil: fren yasasi yalnizca ileri ekseni kontrol ediyor,
% yanal eksende BRAKE boyunca hicbir kontrol yok, ve olculdu ki buyuklugu
% esigin ustunde tutan sey yanal surukleme olabiliyor (min|v_h| = 3.08 iken
% u = -0.51, v = +3.04 m/s) -- o durumda handoff hic istenmiyor ve sonmeyen
% fren pitch'i araci GERI yonde 12.8 m/s'ye kaciriyor. Bkz. backtrans_loop.m
% madde (S) notu.
% DIKKAT: bu deger POS_ENGAGE_V_MAX ile ayni SAYI olmaya devam ediyor ama artik
% ayni SINYALE uygulanmiyor -- pos_hold'un kapisi hala bir buyukluk kapisi.
% Yani handoff istendiginde hold hemen kabul etmeyebilir (yanal surukleme
% varsa); istek her tick yeniden denenir. Bilincli ve olculecek bir kalinti.
p.bt.handoff_v    = 3.0;           % m/s, pos_hold istegi (ileri hiz esigi)
% brake_v_full: 5.0 IDI, 2026-07-30 Adim 37'de handoff_v'ye BAGLANDI.
% Fren yasasi pitch'i hizla soncelerken (pitch = pitch_max*v_h/brake_v_full)
% aracin ustesinden gelmesi gereken ILERI kuvvet SABIT: delta1 = delta2 = 0'da
% takili oldugu icin (madde (P), tek yonlu tilt araligi) yaw trim'i delta0'da
% ~10.5 deg tutuyor ve bu ~3.1 N ileri itki uretiyor. Yani "durabilmek icin"
% gereken burun yukari acisi asin(3.13/(m*g)) = 3.66 deg -- pitch_max'in
% (4.0 deg) hemen altinda. brake_v_full = 5.0 iken yasa 3.5 m/s'de yalnizca
% 2.80 deg = 2.42 N veriyordu, yani gereken 3.13 N'un ALTINDA: manevra
% 3.2-3.5 m/s'de KARARLI bir dengede takilip kaldi ve handoff_v'ye (3.0)
% hic inemedi -- iki bagimsiz ucusta olculdu (2026-07-30, 10_46_58 ve
% 10_51_38: 90+ s boyunca 3.54 +- 0.04 ve 3.64 m/s).
% Adim 31'in iki ucusu esigi ancak SURUKLENME sayesinde gecmisti (birinde son
% 0.5 m/s 12.1 s surdu) -- yani "dogrulandi" denen manevra aslinda MARJINALDI.
% Soncelemenin kendisi dogru (durmus araca +6 deg onu GERIYE 5.77 m/s'ye
% hizlandiriyor), yanlis olan NEREDE soncelendigi: tam guc frenleme en az
% devir hizina kadar surmeli, cunku onun altinda zaten pos_hold pitch'in
% sahibi oluyor. Bagimlilik yoruma degil, koda yazildi.
% Adim 39 (madde S): bu terim de artik v_fwd ile olceklenir, max(0, v_fwd) ile
% -- arac geri gidiyorsa frenleme marji SIFIRDIR (geri kacisi durduran terim).
p.bt.brake_v_full = p.bt.handoff_v; % m/s, pitch tavanina ulasildigi ILERI hiz
p.bt.min_alt      = 15.0;          % m, altinda geri gecise girilmez

%% --- Kontrol yuzeyi limitleri (madde V) ---
% Geometri/etkinlik alanlari (n, cp, dir, k) yukarida p.aero panellerinden
% TURETILIR; burada yalnizca limitler var.
% Sapma limitleri model.sdf'teki EKLEM limitleridir (yazilim limiti degil).
p.surf.max = [ 0.78, 0.78, 0.52, 0.52, 0.52 ];   % rad
% Servo hizi: tilt servolariyla ayni mertebede varsayildi (SDF'te ayrica
% belirtilmemis). Kutu kisiti bundan turer, yani muhafazakar olmasi guvenlidir.
p.surf.rate_max = 4.0;             % rad/s
p.surf.tau      = 0.05;            % s, 1. derece servo gecikmesi (plant)
% ele_trim -- KUYRUK ASAGI YUKUNU IPTAL EDEN SABIT ELEVATOR OFSETI (Adim 52).
% Kuyruk panelleri SDF'te a0 = -0.2 rad ile kurulmus ("a0 = -0.2 gives the
% download a conventional tail needs to trim a nose-heavy aircraft"). Bu, HIZ
% KARESIYLE buyuyen kalici bir ASAGI kuvvettir:
%   16 m/s -> 14.1 N,  20 m/s -> 22.0 N,  25 m/s -> 34.3 N  (agirligin %70'i)
% Kontrolcu bu kuvveti GORMEZ (aero plant'te, modelde degil), dolayisiyla
% tahsisat onu iptal etmeyi hic denemez; kuvveti sessizce ROTORLAR tasir.
% Olculdu (Adim 51): seyirde elevator sapmasi yalnizca 0.11 deg, yani yuzey
% bosta dururken kuyruk rotoru ~15 N'de sabit kaliyor ve kanat rotorleri
% sifira dogru eziliyor.
%
% TURETME: kuyruk panelinin tasima katsayisi  cl = cla*(alpha + a0) + k_c*delta
% ve alpha = 0'da sifirlanmasi icin
%   4.7528*(-0.2) + (-12)*delta = 0  ->  delta = -0.0792 rad = -4.54 deg
% IKI TERIM DE qbar ile olceklendigi icin bu ofset TUM HIZLARDA gecerlidir --
% bir kazanc degil, bir montaj acisi duzeltmesidir.
%
% Bu, madde (P)'nin (`p.ctrl.fx_trim`) yuzey tarafindaki karsiligidir: tahsisatin
% goremedigi kalici bir bias, ancak bir trim terimiyle kapatilabilir.
% ALLOCATOR'A GORUNMEZ olmasi KASITLIDIR: iptal ettigi kuvvet de modelde yok,
% yani ikisi plant'te birbirini goturur. Sanal aktuator koordinati bu ofsetin
% USTUNDEKI sapmadir.
p.surf.ele_trim = -0.0792;         % rad, sabit elevator ofseti (servo_2/3)
p.rho = 1.225;                     % kg/m^3, dinamik basinc icin (kontrolcu)
% --- SANAL YUZEY AKTUATORLERININ WLS CEZALARI (Adim 46) ---------------------
% Bunlar TERCIH degil TURETMEdir. gain_schedule.m'de belgelenen kural: WLS bir
% aktuatoru digerine "birim etki basina ceza" oraniyla tercih eder, Wu_i/|G_i|.
% Yuzeylerin |G|'si qbar ile buyudugu icin SABIT bir wu otomatik olarak
% "hover'da pahali, seyirde ucuz" davranisi verir -- devir noktasi bir mod
% anahtari degil, bir ORANIN kesisimidir. Ayri durum makinesi gerekmemesinin
% sebebi budur.
%
% Adim 45'te tek bir wu_surf = 80 vardi ve bes yuzeyin HEPSINE uygulaniyordu.
% Uc sanal aktuatorun etkinlikleri mertebe farkli oldugu icin tek skaler
% yanlisti; her biri ayri turetildi.
%
% Referans (seyirde): rotor tilt'in birim-etki cezasi. T ~ 9 N, wu_tilt_cruise
% = 1.5:  roll |dtau_x/ddelta| ~ T*0.05 = 0.45 -> 1.5/0.45 = 3.33
%
% (1) wu_ail -- AILERON. Devir noktasi FT_CRUISE_V = 8 m/s'e konumlandi:
%       qbar(8) = 0.5*1.225*64 = 39.2 Pa
%       |G_tau_x| = 1.2*qbar = 47.0 N*m/rad
%       esitlik: wu_ail/47.0 = 3.33  ->  wu_ail = 157  -> 160 secildi
%     ALT SINIR YOK: aileron sutununun Fz'si TAM SIFIR oldugu icin ne kadar
%     ucuz olursa olsun tilt mekanizmasini calamaz (Adim 45/deneme 1'in hata
%     modu bu sutunda yapisal olarak imkansizdir).
%
% (2) wu_ele -- ELEVATOR. Bu tek RISKLI olan, cunku Fz'si gercek. Iki YANLI
%     sinir var ve wu ikisinin ARASINDA olmali (15 m/s, qbar = 137.8):
%       ALT SINIR (Fz'yi CALMAMASI icin): |G_Fz| = 1.152*qbar = 158.7 N/rad,
%         tilt'in Fz etkinligi = T*sin(delta) ~ 9*0.7 = 6.3 N/rad, birim-Fz
%         cezasi 1.5/6.3 = 0.238.  wu_ele/158.7 > 0.238 -> wu_ele > 37.8
%       UST SINIR (pitch icin SECILEBILMESI icin): |G_tau_y| = 0.806*qbar =
%         111.1 N*m/rad, tilt pitch etkinligi ~ 0.106*T = 0.95 N*m/rad, birim
%         cezasi 1.5/0.95 = 1.58.  wu_ele/111.1 < 1.58 -> wu_ele < 175
%     PENCERE: 37.8 < wu_ele < 175. Iki tarafa da ~2.2x marj birakan 80 secildi
%     (alt sinira 2.1x, ust sinira 2.2x). Bu pencerenin VAR OLMASI, elevator'un
%     elevondan farkli olmasinin sayisal ifadesidir: elevonun ayni penceresi
%     BOSTUR (0.05 m kol -> alt sinir ust sinirin uzerine cikar), yani simetrik
%     elevon hicbir agirlikta guvenli degildir. Aktuator olmamasinin sebebi bu.
%
% (3) wu_rud -- RUDDER. Devir noktasi yine 8 m/s:
%       |G_tau_z| = 0.142*qbar = 5.57 N*m/rad
%       esitlik: wu_rud/5.57 = 3.33  ->  wu_rud = 18.5  -> 20 secildi
%     DIKKAT -- "yaw'a daha fazla otorite ver" ailesinde UC deneme basarisiz
%     oldu (Adim 7 Ws_yaw, Adim 10 Kp_yaw, ve digeri). AMA UCU DE otoriteyi
%     ROLL ILE PAYLASILAN aktuator (diferansiyel tilt) uzerinden veriyordu;
%     cakisma oradaydi. Rudder AYRI bir aktuatordur ve roll kuplaji kucuktur
%     (tau_x/tau_z = 0.023/0.142 = 0.16). Yani bu, o ucunun tekrari DEGIL --
%     ama yine de SITL'de olculmeden "dogru" sayilmaz.
%
% !! Uc sayi da SITL'de olculmeden dogrulanmis sayilmaz: Adim 7'de Ws_yaw'i
% mertebe degistirmek Fx talebini -28 N'e patlatmisti. Ilk ucusta yuzey
% sapmalarinin ve doygunlugun izlenmesi sart.
p.wls.wu_ail = 160.0;
p.wls.wu_ele =  80.0;
p.wls.wu_rud =  20.0;
% Hangi sanal eksenlerin tahsisata girecegi [aileron; elevator; rudder].
% Kapatilan eksen sutunu KALDIRILIR (bkz. surf_virtual_map.m).
p.wls.surf_enable = [true; true; true];

%% --- Seyir hiz denetimi (TECS karsiligi), madde V -- cruise_speed_loop.m ---
% Adim 47 (2026-08-03). Adim 46 olctu: fx >= 12 N'de rejim YUZEYLER KAPALIYKEN
% BILE marjinal, cunku aracin hizini sinirlayan sey bir yasa degil kanat
% tilt'inin 90 derecelik MEKANIK DURAGI. Yuzeyler o kazara korumayi kaldiriyor.
% Yani fx bir IVME komutuydu; hiz hicbir zaman denetlenmiyordu.
% Tam gerekce ve kazanc turetmesi cruise_speed_loop.m basliginda.
p.tecs.enable = true;
% v_sp: olculen zarftan. Adim 46'da fx = 8..14 N'in HEPSI 16-17 m/s civarinda
% dengeleniyordu (surukleme egimi dik), ve SITL'de olculen seyir 15.2 m/s.
% 16.0 ikisinin de icinde ve tilt'i duraktan uzak tutuyor.
p.tecs.v_sp   = 16.0;              % m/s, hedef govde ileri hizi
% Kazanclar: c = dD/dv ~ 2.3 N/(m/s) olculdu (fx 8->10 N, v 16.01->16.89 m/s).
% Kp = 1.0 -> kapali dongu tau = m/(c+Kp) = 1.5 s; duruş dongusunden ~6x yavas.
% Ki = Kp/5 -> integral zaman sabiti ~5 s.
p.tecs.kp     = 1.0;               % N per (m/s)
p.tecs.ki     = 0.2;               % N per (m/s) per s
% fx_max: EMNIYET. Adim 46'nin taramasinda fx = 14 N iraksadi, 12 N marjinaldi.
% 14'un ALTINDA, ve seyir trimi (~12 N) uzerinde makul bir manevra payi birakan
% bir tavan. Doygunluk anti-windup ile ele alinir.
p.tecs.fx_max = 13.0;              % N

% --- TECS'in PITCH (enerji dagilimi) yarisi, Adim 49 -- cruise_pitch_loop.m --
% Adim 47 olctu: theta = 0'da kanadin tam agirligi tasidigi hiz
%   V_wb = sqrt(W/(0.5*rho*S*cla*a0)) = 16.925 m/s
% ve pitch setpoint'i her yerde sabit sifirdi, yani arac o hizin ustune
% cikamiyordu (yuzeysiz her kosu 16.85-16.90'da duruyordu). Bu yasa Fz_sp'yi
% (aktuatorlerin dikey yuku) sifira surerek hucum acisini trimler.
p.tecs.pitch_enable = true;
% Kapi: adim 29'un rejiminde (~5-6 m/s) otorite TAM SIFIR olmali. 13 -> 16 m/s
% arasinda smoothstep ile acilir; V_wb'nin (16.925) hemen altinda tam yetkili.
p.tecs.pitch_v_on   = 13.0;        % m/s
p.tecs.pitch_v_full = 16.0;        % m/s
% Ki: dL/dtheta = qbar*S*cla = 841 N/rad @ 17 m/s -> tau = 1/(841*Ki).
% Irtifa dongusunun tau'su ~1.7 s (Kp_z = 0.6); trim ondan ~10x yavas olmali
% -> tau ~ 20 s -> Ki = 5.9e-5. 5e-5 secildi (tau = 23.8 s @ 17 m/s).
p.tecs.pitch_ki  = 5e-5;           % rad / (N*s)
% Sinir: gereken trim araligi HESAPLANABILIR, theta = W/(qbar*S*cla) - a0:
%   12 m/s -> +3.4 deg,  16.9 -> 0,  20 -> -1.0 deg.  6 deg bunun cok ustunde
%   ama hala kucuk bir otorite.
p.tecs.pitch_max = deg2rad(6.0);   % rad
% pitch_fz_sp: HEDEF aktuator dikey yuku (N, FRD, negatif = aktuatorler yukari
% itiyor). SIFIR DEGIL -- ve bu bir tercih degil OLCULMUS bir sinirdir.
% Adim 49'un ilk denemesi hedefi 0 aldi; yasa calisti (Fz_sp -16.7 -> -9.6 N,
% tilt 41 -> 65 deg, kanat %93 -> %106, irtifa/hiz sabit, pitch 0.8 deg) ama
% rotor itkisi 0.00 N'e degdigi anda arac iraksadi (u 15.9 -> 27.6 m/s).
% Sebep yapisal: etkinlik matrisinde tilt sutunu ITKIYLE carpilidir, yani
% T = 0 olan bir rotorun otoritesi TAM SIFIRDIR ve Fx kanalinin hicbir yuzeyi
% yoktur. Olculen: -12 N -> T0 ~ 4.95 N (saglikli), -10 -> 1.83, -8 -> kirilma.
% -12 secildi; orada kanat zaten agirligin %105'ini tasiyor.
p.tecs.pitch_fz_sp = -12.0;        % N

%% --- Ileri gecis (forward transition), madde V -- bkz. forwardtrans_loop.m ---
% Adim 42 (2026-08-03). Gorev TAM OTONOM olmali; ileri yonde bugune kadar bir
% yasa yoktu, yalnizca elle rampalanan bir `fx_sp` vardi.
% fx_cruise / fx_rate: run_transition_test'in OLCULEN profilinden alindi --
%   0 -> 10 N, 12 s rampa, 10.86 m/s'ye ulasti, itki doyumu %0.0, pos_hold
%   birakma handoff'u temiz (hiz aktivitesi DUSTU: p RMS 0.2123 -> 0.1503).
%   Geri gecis testlerinde 15 N ile 15-16.5 m/s olculdu; 12 N ikisinin arasinda
%   ve tilti 45 deg civarina goturuyor (olculen: 15 m/s'de 45-54 deg).
p.ft.fx_cruise = 12.0;             % N, seyir govde-x kuvvet komutu
p.ft.fx_rate   = 10.0/12.0;        % N/s, olculen 0->10 N / 12 s rampasi
% cruise_v: RAMP'in "vardim" esigi. 8 m/s, geri gecisin release_v'sinin (10.0)
%   ALTINDA secildi ki iki manevra ayni hizda birbirini kovalamasin.
p.ft.cruise_v  = 8.0;              % m/s, seyir sayilan yatay hiz
% allow_abort: GEREKSINIM (2026-08-04) -- gorev profili tek parcadir ve ILERI
% GECIS IPTAL OLMAZ. Emniyet dedektorleri calismaya devam eder ve `warn_code`
% ile loglanir; yalnizca EYLEMLERI kalkar. Tam gerekce ve bunun bedeli/kazanci
% forwardtrans_loop.m basliginda. true yapmak eski davranisi BIREBIR geri getirir
% -- ama once FT_MIN_ALT - FT_ALT_BAND > BT_MIN_ALT marji kapatilmalidir
% (bugun 20 - 5 = 15 = 15, yani sifir).
p.ft.allow_abort = false;
% alt_band: EMNIYET 1. Adim 29'un kacis tirmanisi 35 s'de 44 m yapmisti; 5 m
%   onun cok altinda ve olculen normal gecisin (irtifa degisimi < 1 m) cok
%   ustunde, yani ne kacirir ne yanlis alarm verir.
p.ft.alt_band  = 5.0;              % m, giris irtifasindan sapma -> IPTAL
% timeout_s: EMNIYET 2, AERO-BAGIMSIZ (adim 38'in dersi). 12 s'lik rampa +
%   hizlanma payi; 30 s icinde 8 m/s'ye cikamiyorsak kanat beklendigi gibi
%   tasimiyor demektir ve manevra sinirsiz surmemeli.
p.ft.timeout_s = 30.0;             % s, RAMP suresi tavani -> IPTAL
p.ft.min_alt   = 20.0;             % m, altinda ileri gecise girilmez
% NOT: iptal `fx_cmd = 0` DEMEK DEGILDIR -- adim 30 bunu denedi ve arac
% yavaslamadi, cunku fx cok zayif bir amac terimi olarak tiltleri geri cekmez.
% Iptal, GERI GECISI istemektir (req_abort -> bt_enable).


% --- YERE YAKIN KANAT ITKI FARKI SINIRI (2026-08-29) ---
% NE COZUYOR: inis sirasinda arac yere degdikten sonra hafif egik oturuyor
% (olculen kalici roll hatasi 0.18 deg). Zemin araci tuttugu icin
% omega_dot_meas = 0 kalir, dolayisiyla INDI'nin artimi
%     domega_dot = omega_dot_des_adj - omega_dot_meas
% sonmez ve u_cmd = u_actual + du HER TIK ayni yonde birikir. Olculen sonuc
% (ULog 09_27_25): T0 34 N'ye tirmanir, T1 sifir rayina cakilir, ve tek
% calisan EGIK rotor karsiligi olmayan bir yaw momenti uretir -> 6793 deg
% yaw kacisi, arac hic inemez.
%
% NEDEN KUTU/KIRPMA, NEDEN AGIRLIK DEGIL: bu deponun kendi kurali
% (p.rotor.Tmin_cruise notu, Adim 50): "tahsisatin tercihi manevranin tersine
% bakiyorsa onu ancak bir KUTU degistirebilir". Ws/Wu ile oynamak ayrica
% Adim 7'de SITL'de Fx talebini patlatmisti.
%
% NEDEN FARK, NEDEN TUTUM KONTROLUNU KAPATMAK DEGIL: ilk tasarim "yerdeyken
% tutum artimini dondur" idi ve TEHLIKELI cikti -- yer etkisinde 1.29 m'de
% ASILI arac ile yerde OTURAN arac, irtifa ve acisal hiz bakimindan ayirt
% edilemiyor (ikisinde de |w| ~= 0.001 olculdu). Yanlis tarafa dusmek
% havadaki aractan tutum kontrolunu almak demekti. Fark sinirlamak kontrolu
% CANLI birakir ve yalnizca cokusu imkansiz kilar.
%
% ORTALAMA KORUNUR: kirpma T0/T1'in ORTALAMASINA dokunmaz, yalnizca farki
% siler. Dikey kanal (Fz) ortalamadan gelir, yani irtifa dongusu hic
% etkilenmez -- ki olcum onun ZATEN dogru davrandigini gosterdi (talep
% -25 N'e inmisti, izlenmeyen sey tahsisatti).
%
% DEGER: saglikli iniste olculen fark ort 0.40 N / max 0.95 N (ULog 10_32_31);
% arizada 45.00 N (tam olcek). 10 N, saglikli calismanin 10 katindan fazla pay
% birakir ve yine de cokusun cok altinda. 0.25 m kolda ~2.5 Nm roll torku eder.
% Adim 145: kapi altinda dikey itki tavani (x|Fz_sp|). Gerekce ve olculen
% esik ayrimi TiltrotorIndiParams.hpp LAND_TZ_MAX notunda; PX4 ve
% sf_wls_alloc.m ile SENKRON KALMALI.
p.ctrl.land_tz_max = 2.0;
% Adim 160: kuyruk itki tabani (PX4 LAND_TAIL_FLOOR_FRAC ile SENKRON).
p.ctrl.land_tail_floor_frac = 0.5;

% INIS DIZISI (Adim 153/160). TiltrotorIndiParams.hpp LAND_* ve
% sf_landing_sequence.m ile SENKRON KALMALI. Degerler run_mission_test.py'den
% birebir tasindi; gerekceleri orada ve C++ tarafinda yazili.
p.ctrl.land_step_m = 1.0;              % m,  1.5 m 13 BIG_M uretmisti
p.ctrl.land_step_s = 1.5;              % s
p.ctrl.land_flare_alt = 1.5;           % m AGL
p.ctrl.land_touch_z = 0.15;            % m, yer datumunun ALTI
p.ctrl.land_done_alt = 0.25;           % m AGL
p.ctrl.land_ground_thrust_frac = 0.5;  % temas: dikey itki < bu x agirlik
p.ctrl.land_touch_dwell = 1.5;         % s, kesintisiz temas suresi

% GOREV DIZICISI (Adim 154/160). TiltrotorIndiParams.hpp MSN_* ve
% sf_mission_sequencer.m ile SENKRON KALMALI.
p.ctrl.msn_climb_alt = 40.0;           % m AGL
p.ctrl.msn_climb_tol = 2.0;            % m
p.ctrl.msn_settle_s = 12.0;            % s
p.ctrl.msn_cruise_s = 8.0;             % s
p.ctrl.msn_fw_cruise_s = 10.0;         % s
p.ctrl.msn_land_vh = 1.0;              % m/s
p.ctrl.msn_timeout_s = 60.0;           % s
p.ctrl.msn_home_r = 8.0;               % m
p.ctrl.msn_return_timeout_s = 400.0;   % s
p.ctrl.msn_fw_phase = true;
p.ctrl.land_diff_max = 10.0;   % N, |T0-T1| ust siniri (yalnizca yere yakinken)
% Bu irtifanin USTUNDE mekanizma TAMAMEN ETKISIZ. 2.0 m, olculen butun yer
% olaylarinin (oturma 0.03-0.6 m, yer etkisinde asilma 1.17-1.29 m) ustunde,
% ama her ucus rejiminin cok altinda.
% ADIM 116 -- "AGL" BURADA HARFI HARFINE YERDEN YUKSEKLIKTIR. Kestirimcinin
% yerel orijininden yukseklik (-z) DEGILDIR: 23 SITL kosumunda o sinyalin
% datum ofseti -0.67 .. +1.77 m olculdu, yani asagidaki esikle AYNI MERTEBEDE.
% PX4 portu 116'ya kadar -z besliyordu; sonuc, kapinin rastgele armanmasi ve
% olculen en kotu kosumda (ULog 11_26_27) 0.64 m'de yere degen aracin
% mekanizmayi hic devreye sokmadan farki 45 N'e (tam olcek) goturmesiydi.
% Esigi buyuterek "cozmek" bu hatayi olcmek yerine ortmek olur.
p.ctrl.land_diff_alt = 2.0;    % m AGL
% 0 = kapali (davranis 2026-08-29 oncesi ile birebir ayni).
p.ctrl.land_diff_enable = 1;
% TEMAS ESIGI: bu roll acisinin USTUNDE kanat farki tamamen silinir (sinir 0).
% 8 deg, olculen saglikli inisin (|roll| < 1 deg) sekiz kati ustunde ve
% olculen temas olayinin (-19.78 deg) cok altinda. Serbest hover'da tutum
% dongusu bu acıya asla izin vermez, dolayisiyla ucusta tetiklenemez.
p.ctrl.land_contact_roll = 8.0*pi/180;   % rad
% IKINCI TEMAS OLCUTU (Adim 118) -- roll esigi DUZ inisi hic yakalamiyor.
% Olculdu: uc kosumda da fark 10 N sinirinda DOYDU ve orada kaldi, ama roll
% 0.18-0.54 derecede DONMUSTU, yani yukaridaki dal hic atesLenmedi; kalici yaw
% momenti araci dondurdu. 8 derecelik esigin gerekcesi olculmus bir TAKLA
% olayiydi (-19.78 deg), duz bir inis degil.
%
% OLCUT: "buyuk diferansiyel KOMUT, ama acisal ivme YOK". Serbest ucusta bu
% fiziksel olarak imkansizdir -- 6 N'lik fark 0.25 m kolda ~1.5 Nm eder ve
% govdeyi ivmelendirmek ZORUNDADIR. Yerde ise zemin momenti karsilar.
%
% NEDEN Adim 109'un ELENEN OLCUTU DEGIL: o "|w| ~ 0 ise temas" diyordu ve arac
% yalnizca sakin oldugunda da (yer etkisinde asili) atesleniyordu. Bu olcut
% ayrica BUYUK BIR KOMUT ister; sakin ama serbest bir arac bu komutu almaz.
%
% AYRIM OLCULDU (26 tam gorev logu: hover, gecis, seyir, sabit kanat, inis):
%   AGL > 2 m'de olcutun kesintisiz surdugu en uzun sure : 0.01 s (tek ornek)
%   AGL < 1.5 m'de                                       : 3.28 s'ye kadar
%   saglikli inislerde (temiz 6 kosum)                   : 0.00 s -- hic
% 0.20 s bekleme, olculen en kotu havada-yanlis-pozitifin 20 KATI.
p.ctrl.land_contact_diff  = 6.0;         % N, |T0-T1| bu degerin ustundeyken
p.ctrl.land_contact_acc   = 0.05;        % rad/s^2, |pdot| bu degerin altindaysa
p.ctrl.land_contact_dwell = 0.20;        % s, bu sure kesintisiz surerse TEMAS

end
