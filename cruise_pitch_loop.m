function [pitch_sp, state_out] = cruise_pitch_loop(v_fwd, Fz_sp, state_in, p)
%CRUISE_PITCH_LOOP  TECS'in ENERJI DAGILIMI (pitch) yarisi: hucum acisi trimi.
%
% Adim 49 (2026-08-04), madde (V). Adim 47'nin OLCTUGU duvarin dogrudan cevabi.
%
% NEDEN VAR -- TEK BIR DENKLEM
% ----------------------------
% Adim 47 olctu: yuzeysiz arac, fx ne olursa olsun (8/10/12/14 N) ve v_sp ne
% olursa olsun (17/18/19) HEP 16.85-16.90 m/s'de duruyordu. Bu bir tilt duragi
% ya da acik dongu fx degil; su denklemin kendisiydi:
%
%   theta = 0'da kanadin TAM AGIRLIGI tasidigi hiz
%     V_wb = sqrt( W / (0.5*rho*S*cla*a0) ) = 16.925 m/s
%
% `att_sp`in pitch bileseni her yerde SABIT SIFIRDI. V_wb ustunde seviye ucus
% NEGATIF hucum acisi ister; arac theta = 0'a civili oldugu icin kanat
% agirliktan fazlasini uretir, tirmanmak zorunda kalir, irtifa dongusu buna
% "tasimayi azalt" diye karsi koyar ve bir sey doyana kadar direnir. Yuzeyler
% kapaliyken arac o hiza HIC cikamadigi icin bu goze hic gorunmedi.
%
% NEDEN BU, ADIM 47'NIN "pitch = 0" TERCIHININ GERI ALINMASI DEGIL
% ----------------------------------------------------------------
% Adim 47 pitch'i 0'da tutmayi Adim 29'a dayandirdi ve o dayanak YANLIS
% GENELLENMISTI: adim 29'un olcumu ~5-6 m/s rejimindedir (orada burun yukari
% gercekten bir TIRMANMA komutudur ve irtifa dongusu kanat tasimasiyla bas
% edemez). Bu yasa o rejimde KAPALIDIR -- kapi v_on = 13 m/s'de acilmaya baslar.
% Yani ayni degisken, iki farkli rejimde iki farkli sey ifade ediyor; celiski
% yok, eksik olan REJIM AYRIMIYDI.
%
% AMAC FONKSIYONU: AKTUATORLERIN DIKEY YUKUNU BIR HEDEFE SURMEK
% -------------------------------------------------------------
% Klasik TECS pitch'i "enerji dagilimina" baglar. Bu mimaride ayni seyin daha
% dogrudan bir ifadesi var: dis dongu zaten GOVDE EKSENLI KUVVET komut ediyor,
% ve `Fz_sp` tam olarak "aktuatorlerin tasimasi gereken dikey kuvvet"tir.
% Yasa:
%
%   theta_dot = -Ki * (Fz_sp - Fz_hedef)      (kapiyla olceklenmis, sinirlanmis)
%
% Fz_sp hedeften daha NEGATIF (aktuatorler fazla yukleniyor, kanat yetmiyor)
% -> burun YUKARI. Daha pozitif (kanat fazla tasiyor) -> burun ASAGI. V_wb
% ustunde gereken sey tam olarak ikincisidir.
%
% *** VE HEDEF SIFIR DEGILDIR -- BU OLCULDU (Adim 49, ilk deneme). ***
% Ilk yazimda hedef 0 idi ("tam kanat-tasimali ucus, tanim geregi Fz_sp = 0").
% Yasa tam olarak bunu yapti ve calisti: 12 s icinde Fz_sp -16.7 -> -9.6 N,
% tilt 41 -> 65 deg, kanat yuk payi %93 -> %106, irtifa ve hiz sabit, gereken
% pitch yalnizca 0.8 deg. AMA rotor itkisi 5.12 -> 0.63 -> 0.00 N'e indi ve
% TAM 0'a degdigi anda arac iraksadi (u 15.9 -> 27.6 m/s, 2 s icinde).
%
% Sebep sayisal degil YAPISALDIR: etkinlik matrisinde tilt sutunu
%   dtau/ddelta = (r x ddir + km*ddir) * T_i
% yani ITKIYLE CARPILIDIR. Itkisi sifir olan bir tilt rotorunun kontrol
% otoritesi de TAM SIFIRDIR -- tahsisat o aktuatoru tamamen kaybeder. Yuzeyler
% roll/pitch/yaw'i devralabiliyor ama **Fx kanalinin hicbir yuzeyi yok**; geriye
% kalan tek rotor Fz uretmek icin kullanildiginda kacinilmaz olarak T*sin(delta)
% kadar da Fx uretiyor ve bunu iptal edecek ikinci bir aktuator kalmiyor.
% Bu, Adim 43-45'in kok nedeninin ta kendisidir: bu airframe'de butun durus
% otoritesi rotorlardan gelir ve **rotorlar tamamen kapatilamaz.**
%
% Dolayisiyla hedef, rotorlari HAYATTA tutan kucuk bir yuktur. Olculen degerler
% (ayni izden): Fz_sp = -12 N -> T0 ~ 4.95 N (saglikli otorite), -10 -> 1.83,
% -8 -> 0.00 (kirilma). -12 secildi: kirilma noktasindan 2 kademe uzakta, ve
% orada kanat zaten agirligin %105'ini tasiyor, tilt 59 deg. Yani "tam sabit
% kanat" pratikte bu noktadir; sifir itki bir hedef degil, bir ucurumdur.
%
% ILERI BESLEME (Adim 53, 2026-08-04) -- DUVARI KALDIRAN SEY BU
% ==============================================================
% Adim 49-52 boyunca ~17 m/s'lik bir "duvar" vardi ve dort ayri aday cürütüldü
% (wu_ele, hiz dongusu kazanci, itki tabani, tilt tavani); Adim 52'nin kuyruk
% trimi onu 2.8x GECIKTIRDI ama kaldirmadi. Adim 53 duvarin son parcasini
% kanal bazinda olctu (istenen ve BASARILAN sanal kontrol ayri ayri kaydedildi)
% ve sebep tek bir cumleyle su cikti:
%
%   *** GEREKEN TRIM ZATEN KUCUKTU VE SINIRIN COK ICINDEYDI; YASA ORAYA
%       ZAMANINDA VARAMIYORDU. ***
%
% 19 m/s'de iraksamadan hemen once olculen: gereken theta = -0.71 deg (sinir
% 6 deg, yani otoritenin %12'si), ulasilan theta = -0.59 deg. Yani otorite de
% doygunluk da yoktu -- YARIS KAYBEDILMISTI. Sebebi yapisal: yasa SAF
% INTEGRALDI (theta_dot = -Ki*hata) ve tau = 1/(qbar*S*cla*Ki) ~ 19-24 s.
% Hiz 16 -> 19 m/s'ye cikarken gereken trim de hareket ediyor; saf integral
% hareketli bir hedefi tanim geregi GERIDEN izler. Bu arada kanat fazla
% tasidigi icin irtifa dongusu Fz_sp'yi POZITIFE (asagi kuvvet talebi)
% suruyor -- rotorlarin URETEMEYECEGI bir isaret -- ve tahsisat kanat
% rotorlerinin itkisini sifira eziyor; T = 0 olunca (dtau/ddelta ~ T) o
% rotorlerin otoritesi de sifirlaniyor ve arac gidiyor.
%
% Cozum bir kazanc degil, EKSIK TERIM: gereken trim ZATEN KAPALI FORMDA
% BILINIYOR. run_cruise_pitch_loop_test'in D2 denetimi tam bu ifadeyi
% (`th_hand`) Adim 49'dan beri elle turetip yasanin dengesiyle karsilastiriyordu
% -- yani dogru cevap testin icinde yaziliydi, yasanin icinde degil:
%
%   theta_ff(V) = (W + Fz_hedef) / (qbar*S*cla)  -  a0
%
% Bu terim eklenince integral yalnizca MODEL HATASINI kapatmakla kalir, hedefi
% kovalamakla degil. Dolayisiyla:
%   * Ki DEGISMEDI (5e-5) -- irtifa dongusuyle zaman olcegi ayrimi garantisi
%     (asagidaki turetme) oldugu gibi duruyor. Duvar bir kazanc artirimiyla da
%     kalkiyordu (Ki x4 olculdu, 19 ve 20 m/s kararli) ama o, ayrim garantisini
%     3.5x'e dusururdu; ileri besleme ayni sonucu BEDAVA veriyor.
%   * Olculen sonuc (kapali dongu, hiz + pitch + yuzeyler, 120 s):
%       ff YOK : 16,17,18 kararli / 19 t=39.8 s / 20 t=39.4 s'de iraksiyor
%       ff VAR : 16,17,18,19,20,22,24 KARARLI -- tilt 51.3 -> 77.1 deg,
%                Fz_sp her hizda tam hedefte (-11.8..-12.0 N), T0 ~ 7.3 N
%       26+ komut edilince arac 25.1 m/s'de DENGEYE oturuyor (iraksama yok):
%       hizi sinirlayan sey artik bir kararsizlik degil p.tecs.fx_max = 13 N,
%       yani KASITLI bir tavan. Duvarin kalkmasinin tanimi budur.
%
% MODEL HATASINA KARSI: theta_ff aero modeline (S, cla, a0, rho) baglidir ve
% donanimda bunlar belirsizdir. Iki koruma var ve ikisi de yapisal: (1) terim
% pitch_max ile SINIRLI, ve gereken araligin kendisi kucuk (12 m/s -> +3.4 deg,
% 20 -> -1.0 deg), yani %100'luk bir model hatasi bile birkac derecelik bir
% komuttur; (2) integral hala oradadir ve kalan sapmayi turetilen tau ile
% kapatir -- ileri besleme onun YERINE degil, ONUNDE calisir.
%
% NEDEN INTEGRAL, VE NEDEN YAVAS (CAKISMA OLMAMASININ SEBEBI)
% -----------------------------------------------------------
% Bu bir TRIM'dir, bir manevra dongusu degil. Irtifayi hala irtifa dongusu
% tutar (Fz kanali); bu yasa yalnizca onun KALICI yukunu alip kanada devreder
% -- otomatik pilotun servo yukunu izleyen elevator trimi gibi. Ikisinin
% cakismamasinin sarti ZAMAN OLCEGI AYRIMIDIR.
%
% KAZANC TURETMESI (uydurulmadi):
%   Kanat tasimasinin pitch'e duyarliligi:  dL/dtheta = qbar*S*cla
%     17 m/s -> 0.5*1.225*289 * 1.0 * 4.7528 = 841 N/rad
%   Aktuator yuku dogrudan bunun tersidir:  dFz_sp/dtheta = +841 N/rad
%   Kapali dongu:  Fz_sp_dot = -(dFz_sp/dtheta)*Ki*Fz_sp  ->  tau = 1/(841*Ki)
%   Irtifa dongusunun dis bant genisligi Kp_z = 0.6 1/s (tau ~ 1.7 s). Trim
%   ondan ~10x YAVAS olmali: tau ~ 20 s  ->  Ki = 1/(841*20) = 5.9e-5
%   5e-5 secildi (tau = 23.8 s @ 17 m/s). tau ~ 1/qbar oldugu icin dusuk hizda
%   kendiliginden daha da yavaslar (35 s @ 14 m/s) -- yani kapinin acildigi
%   yerde en temkinli, seyirde en canli. Bu ISTENEN yondur.
%
% SINIR: gereken trim araligi kucuktur ve bu HESAPLANABILIR --
%   theta_trim = W/(qbar*S*cla) - a0:  12 m/s -> +3.4 deg, 16.9 -> 0, 20 -> -1.0
% p.tecs.pitch_max = 6 deg bunun cok ustunde ama hala kucuk bir otoritedir.
%
% Girisler:
%   v_fwd    (m/s) GOVDE ILERI hizi (isaretli -- madde (S)'nin dersi)
%   Fz_sp    (N)   irtifa dongusunun urettigi govde-z kuvvet komutu (FRD,
%                  pozitif = ASAGI). Bu yasanin p.tecs.pitch_fz_sp'ye surdugu
%                  buyukluk.
%   state_in [I]   integrator (rad)
%   p        tiltrotor_params()
% Ciktilar:
%   pitch_sp  (rad) duruş referansinin pitch bileseni
%   state_out [I]

I = state_in(1);

if ~p.tecs.pitch_enable
    pitch_sp  = 0.0;
    state_out = 0.0;
    return;
end

% --- KAPI: yalnizca kanat-tasimali rejimde ---------------------------------
% smoothstep, gain_schedule.m ile ayni bicim: ayri bir durum makinesi degil,
% surekli bir karisim. v_on altinda otorite TAM SIFIRDIR (adim 29'un rejimi).
s = (v_fwd - p.tecs.pitch_v_on) / (p.tecs.pitch_v_full - p.tecs.pitch_v_on);
s = max(0, min(1, s));
w = 3*s^2 - 2*s^3;

if w <= 0
    % Kapi kapali: integratoru de sifira cek ki kapi acildiginda birikmis bir
    % gecmisle baslamasin (bumpless giris).
    I         = 0.0;
    pitch_sp  = 0.0;
    state_out = I;
    return;
end

lim = p.tecs.pitch_max;

% --- ILERI BESLEME: ANALITIK TRIM (Adim 53) --------------------------------
% theta_ff = (W + Fz_hedef)/(qbar*S*cla) - a0.  Gerekce basliktadir. Hiz
% tabani kapinin alt ucudur: w > 0 zaten v_fwd > v_on demek, ama bolme
% guvenligi ifadenin kendisinde dursun (kapi ileride degisirse burasi
% sessizce sonsuza gitmesin).
v_ff    = max(v_fwd, p.tecs.pitch_v_on);
qbar_ff = 0.5 * p.aero.rho * v_ff^2;          % p.aero.rho: turetme AERODINAMIK
dLdth   = qbar_ff * (2*p.aero.area(1)) * p.aero.cla;   % N/rad, iki kanat yarisi
th_ff   = (p.m*p.g + p.tecs.pitch_fz_sp)/dLdth - p.aero.a0(1);
th_ff   = max(min(th_ff, lim), -lim);

% --- INTEGRAL TRIM: yalnizca MODEL HATASI icin -----------------------------
err    = Fz_sp - p.tecs.pitch_fz_sp;
I_cand = I - p.tecs.pitch_ki * err * p.Ts_pos;

% Anti-windup: kosullu integrasyon, ve olcut TOPLAM komuttur (th_ff + I) --
% integratorun kendisi degil. Adim 53'ten once ileri besleme yoktu ve ikisi
% ayni seydi; simdi ayrilar, ve dogru olcut fiilen komut edilen degerdir.
% Integratoru dondurmak, sinir birakildiginda birikmis bir hatanin geri
% tepmesini onler -- cruise_speed_loop.m ile ayni disiplin.
th_tot = th_ff + I_cand;
if th_tot > lim || th_tot < -lim
    pushing_out = (th_tot >  lim && err < 0) || ...
                  (th_tot < -lim && err > 0);
    if pushing_out
        I_cand = I;
    end
end
% Sert kirpma da TOPLAMA gore yazilir (I icin [-lim-th_ff, lim-th_ff]), yani
% kirpma ile kosullu integrasyon AYNI sinira bakar. Ikisi ayri sinira bakarsa
% (orn. I dogrudan +-lim'e kirpilirsa) ileri besleme ters isaretliyken sert
% kirpma once bagladigi icin toplam, sinira HIC ULASAMAZ -- otorite sessizce
% |th_ff| kadar kisilir. 20 m/s'de bu 6.0 yerine 4.4 derece demekti.
I = max(min(I_cand, lim - th_ff), -lim - th_ff);

pitch_sp  = w * max(min(th_ff + I, lim), -lim);
state_out = I;

end
